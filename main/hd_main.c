/*
esp32_hd
Copyright (c) 2018 ys1797 (yuri@rus.net)

License (MIT license):
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include "freertos/xtensa_api.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp32/rom/rtc.h"
#include "esp_system.h"
#include "esp_idf_version.h"
#include "esp_event.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "driver/ledc.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "soc/ledc_struct.h"
#include "math.h"
#include "time.h"
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>
#include <cJSON.h>
#include "lwip/apps/sntp.h"
#include "ds.h"
#include "hd_bmp180.h"
#include "esp_platform.h"
#include "config.h"
#include "hd_spi_i2c.h"
#include "hd_wifi.h"
#include "hd_main.h"
#include "cgiupdate.h"
#include "cgiwebsocket.h"
#include "esp_request.h"



//#define GPIO_INPUT_PIN_SEL  (1<<GPIO_DETECT_ZERO) 
#define GPIO_OUTPUT_PIN_SEL  (1<<GPIO_BEEP)

volatile int32_t Hpoint = HMAX;

#define TIMER_DIVIDER   80		/*!< Hardware timer clock divider */
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (0*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */ 
#define TIMER_INTERVAL_SEC   (0.001)   /*!< test interval for timer */ 


char *Hostname;		// Имя хоста
char *httpUser;		// Имя пользователя для http
char *httpPassword;	// Пароль для http
int httpSecure;		// Спрашивать пароль
int wsPeriod=5;		// Период обновления данных через websocket

unsigned char ds1820_devices;                  // Количество датчиков ds18b20

// клапана
unsigned char klp_gpio[4] =  {26, 27, 32, 33};
klp_list Klp[MAX_KLP];		// Список клапанов.
xQueueHandle valve_cmd_queue; // очередь команд клапанов
void valveCMDtask(void *arg);
void cmd2valve (int valve_num, valve_cmd_t cmd);

/* Время */
time_t CurrentTime;
struct tm CurrentTm;
volatile uint32_t tic_counter;
volatile uint32_t uptime_counter;
volatile int gpio_counter = 0;
volatile uint32_t setDelay = 5;

/* Данные режима работы */
main_mode MainMode = MODE_IDLE;		// Текущий режим работы
int16_t MainStatus=START_WAIT;		// Текущее состояние (в зависимости от режима)
alarm_mode AlarmMode = NO_ALARM;	// Состояние аварии
int16_t CurPower;			// Текущая измерянная мощность
int16_t SetPower;			// Установленная мощность
int16_t CurVolts;			// Текущее измеренное напряжение
volatile int16_t CurFreq;		// Измерянное число периодов в секунду питающего напряжения
int16_t WaterOn =-1;			// Флаг включения контура охлаждения
float TempWaterIn = -1;			// Температура воды на входе в контур
float TempWaterOut = -1; 		// Температура воды на выходе из контура
int16_t WaterFlow=-1;			// Значения датчика потока воды.

// Динамические параметры
double tempTube20Prev;		// Запомненое значение температуры в колонне
uint32_t secTempPrev;		// Отметка времени измеренной температуры
double  tempStabSR;		// Температура, относительно которой стабилизируется отбор СР
int16_t ProcChimSR = 0;		// Текущий процент отбора СР
double  startPressure = -1;	// Атмосферное давление в начале процесса
#define COUNT_PWM	15	// Размер таблицы автоподбора шим

typedef struct {
	double temp;
	int16_t pwm;
} autopwm;
autopwm autoPWM[COUNT_PWM] = {
	{0, -1},
	{88.0, 68},
	{96.0, 25},
	{100, 0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0},
	{0,0}
};

#ifdef DEBUG
TickType_t xOpenTime=0,xCloseTime=0;
#endif


int16_t rect_timer1=0;		// Таймер для отсчета секунд 1
int16_t timer_sec2=0;		// Таймер для отсчета секунд 2
int16_t timer_sec3=0;		// Таймер для отсчета секунд 3
int32_t SecondsEnd;		// Время окончания процесса
RESET_REASON resetReason;	// Причина перезагрузки
nvs_handle nvsHandle;

static volatile int beeperTime=0;
static volatile bool beepActive = false;

// Включаем бипер
void myBeep(bool lng)
{
	if (beepActive) return;
	if (lng) beeperTime = 1000;	
	else beeperTime = 500;
	beepActive = true;
	gpio_set_level(GPIO_BEEP, 1);	
}

void shortBeep(void)
{
	if (beepActive) return;
	beeperTime = 150;
	beepActive = true;
	gpio_set_level(GPIO_BEEP, 1);
}

double roundX (double x, int precision)
{
   int mul = 10;
   
   for (int i = 0; i < precision; i++)
      mul *= mul;
   if (x > 0)
      return floor(x * mul + .5) / mul;
   else
      return ceil(x * mul - .5) / mul;
}

extern uint8_t PZEM_Version;	// Device version 3.0 in use ?

void diffOffTask(void *arg){
	openKlp(klp_diff);
	vTaskDelay(5000/portTICK_PERIOD_MS);
	closeKlp(klp_diff);
	vTaskDelete(NULL);
}

bool is_diffOffCondition(void){
	return (	((AlarmMode & ALARM_OVER_POWER)&&(getIntParam(DEFL_PARAMS, "alarmDIFFoffP")))
			   ||
			        ((AlarmMode & ALARM_TEMP)&&(getIntParam(DEFL_PARAMS, "alarmDIFFoffT"))));
}

void alarmControlTask(void *arg){
	int vDIFFoffDelaySec;
	TickType_t vDIFFoffTime;

	while(1) {
		if  ( is_diffOffCondition() )	{
			vDIFFoffDelaySec = getIntParam(DEFL_PARAMS, "DIFFoffDelay");
			ESP_LOGE(__func__,"start DIFF-OFF proc. Delay (%d sec)",vDIFFoffDelaySec);
			if (AlarmMode & ALARM_OVER_POWER)	 	ESP_LOGE(__func__,"			overPower");
			if (AlarmMode & ALARM_TEMP) 					ESP_LOGE(__func__,"			overTemerature");

			vDIFFoffTime = xTaskGetTickCount () + SEC_TO_TICKS(vDIFFoffDelaySec);
			while (xTaskGetTickCount ()<vDIFFoffTime) {// задержка выключения диф-автомата
				shortBeep();
				vTaskDelay(SEC_TO_TICKS(1));
			}
			if (is_diffOffCondition()){ // если ситуация не исправилась
				ESP_LOGE(__func__,"DIFF turned off");
				openKlp(klp_diff); 											// подаем напряжение (клапан 4)
				vTaskDelay(SEC_TO_TICKS(5));
				closeKlp(klp_diff);											// через 5 сек - снимаем напряжение с клапана
			}
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void pzem_task(void *arg)
{
	int16_t maxPower;
	TickType_t overPowerAlarmTime;
	bool  flag_overPower=0;
	float v;
	int cnt =0;


	PZEM_init();
	maxPower = getIntParam(DEFL_PARAMS, "maxPower");

	while(1) {

		if (PZEM_Version)  PZEMv30_updateValues();

		v = PZEM_voltage();
		if (-1 == v) CurVolts = 0;
		else CurVolts = v;
		v = PZEM_power();
		if (-1 == v) CurPower = 0;
		else CurPower = v;
		if (CurVolts<10) AlarmMode |= ALARM_FREQ;
		else AlarmMode &= ~(ALARM_FREQ);

		if (AlarmMode & ~(ALARM_FREQ|ALARM_NOLOAD)) {
			// При аварии выключаем нагрев
			myBeep(false);
			setPower(0);
		}

		if (SetPower) {
			if (MainMode == MODE_IDLE) setPower(0);	// В режиме монитора выключаем нагрев
			if (PROC_END == MainStatus) setPower(0); // Режим окончания работы - отключение нагрузки
		}

		if (CurPower<5 && Hpoint<=TRIAC_GATE_MAX_CYCLES && SetPower > 0) {
			// Отслеживание отключенной нагрузки
			cnt ++;
			if (cnt>10) AlarmMode |= ALARM_NOLOAD;
		} else {
			cnt = 0;
			AlarmMode &= ~(ALARM_NOLOAD);
		}

		//контроль пробития триака
		if (((CurPower- SetPower)*100L/maxPower)>DELTA_TRIAK_ALARM_PRC){ // лимиты тревоги превышены
			myBeep(true);
			if (!flag_overPower){  // первое детектирование
				overPowerAlarmTime = xTaskGetTickCount () + SEC_TO_TICKS(TRIAK_ALARM_DELAY_SEC);// фиксируем время включения аларма (в тиках)
				flag_overPower = true;
			}
			else {
				if (xTaskGetTickCount () > overPowerAlarmTime){
					AlarmMode |= ALARM_OVER_POWER;
				}
			}
		}
		else { //превышения мощности нет, сбрасываем флаги и бит аларма
			flag_overPower = false;
			AlarmMode &= ~ALARM_OVER_POWER;
		}


		if (SetPower <= 0) {
			Hpoint = HMAX;
		} else {
			// Проверка рассинхронизации мощности
			int delta = abs(SetPower - CurPower);
//			int p5 = SetPower/20;
			char inc = SetPower > CurPower;

			if (delta > 200 ) {
				if (inc) Hpoint -= 50; 
				else Hpoint += 50;
			} else if (delta > 50) {
				if (inc) Hpoint -= 10; 
				else Hpoint += 10;
			} else  if (delta > 10) {
				if (inc) Hpoint -=2; 
				else Hpoint +=2;
			} else if (delta >= 5) {
				if (inc) Hpoint --; 
				else Hpoint ++;
			}
		}
		if (SetPower > 0 && Hpoint >= (HMAX - TRIAC_GATE_MAX_CYCLES) ) {
			Hpoint = HMAX - 1 - TRIAC_GATE_MAX_CYCLES;
		}
		if (Hpoint<TRIAC_GATE_MAX_CYCLES) Hpoint=TRIAC_GATE_MAX_CYCLES;

		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}


const char *getMainModeStr(void)
{
	switch (MainMode) {
        case MODE_IDLE:	return "Монитор";
        case MODE_POWEERREG: return "Регулятор мощности";
	case MODE_DISTIL: return "Дистилляция";
	case MODE_RECTIFICATION: return "Ректификация";
	case MODE_TESTKLP: return "Тестирование клапанов";
	default: return "Неизвестно";
	}
}

const char *getMainStatusStr(void)
{
	switch (MainMode) {
        case MODE_IDLE:
	case MODE_TESTKLP:
		return "Ожидание команды";
        case MODE_POWEERREG:
		switch (MainStatus) {
		case START_WAIT: return "Ожидание запуска процесса";
		case PROC_START: return "Стабилизация мощности";
		default: return "Завершение работы";
		}
		break;
	case MODE_DISTIL:
		switch (MainStatus) {
		case START_WAIT: return "Ожидание запуска процесса";
		case PROC_START: return "Начало процесса";
		case PROC_RAZGON: return "Разгон до рабочей температуры";
		case PROC_DISTILL: return "Дистилляция";
		case PROC_WAITEND: return "Отключение нагрева, подача воды для охлаждения";
		default: return "Завершение работы";
		}
		break;
	case MODE_RECTIFICATION:
		switch (MainStatus) {
		case START_WAIT: return "Ожидание запуска процесса";
		case PROC_START: return "Начало процесса";
		case PROC_RAZGON: return "Разгон до рабочей температуры";
		case PROC_STAB: return "Стабилизация температуры";
		case PROC_GLV: return "Отбор головных фракций";
		case PROC_T_WAIT: return "Ожидание стабилизации температуры";
		case PROC_SR: return "Отбор СР";
		case PROC_HV: return "Отбор хвостовых фракций";
		case PROC_WAITEND: return "Отключение нагрева, подача воды для охлаждения";
		default: return "Завершение работы";
		}
		break;
	default:
		return "Неизвестно";
	}

}

const char *getAlarmModeStr(void)
{
	static char str[256];
	int cnt;

	if (!AlarmMode) return "<b class=\"green\">Не зафиксированo</b>";
	strcpy(str, "<b class=\"red\">");

	if (AlarmMode & ALARM_TEMP) {
		cnt = sizeof(str) - strlen(str);
		strncat(str, "Превышение температуры", cnt);
	}
	if (AlarmMode & ALARM_WATER) {
		cnt = sizeof(str) - strlen(str);
		strncat(str, " Нет охлаждения", cnt);
	}
	if (AlarmMode & ALARM_FREQ) {
		cnt = sizeof(str) - strlen(str);
		strncat(str, "Нет напряжение сети", cnt);
	}
	if (AlarmMode & ALARM_NOLOAD) {
		cnt = sizeof(str) - strlen(str);
		strncat(str,"  Нет нагрузки", cnt);
	}
	if (AlarmMode & ALARM_EXT) {
		cnt = sizeof(str) - strlen(str);
		strncat(str,"  Сработал аварийный датчик", cnt);
	}
	if (AlarmMode & ALARM_OVER_POWER) {
		cnt = sizeof(str) - strlen(str);
		strncat(str,"  ВЫСОКАЯ МОЩНОСТЬ !", cnt);
	}
	strcat(str,"</b>");
	return str;
}


// Получение строки о причине перезагрузки
const char *getResetReasonStr(void)
{

	switch (resetReason) {
        case POWERON_RESET: return "Vbat power on reset";
        case SW_RESET: return "Software reset digital core";
        case OWDT_RESET: return "Legacy watch dog reset digital core";
        case DEEPSLEEP_RESET: return "Deep Sleep reset digital core";
        case SDIO_RESET: return "Reset by SLC module, reset digital core";
        case TG0WDT_SYS_RESET: return "Timer Group0 Watch dog reset digital core";
        case TG1WDT_SYS_RESET: return "Timer Group1 Watch dog reset digital core";
        case RTCWDT_SYS_RESET: return "RTC Watch dog Reset digital core";
        case INTRUSION_RESET: return "Instrusion tested to reset CPU";
        case TGWDT_CPU_RESET: return "Time Group reset CPU";
        case SW_CPU_RESET: return "Software reset CPU";
        case RTCWDT_CPU_RESET: return "RTC Watch dog Reset CPU";
        case EXT_CPU_RESET: return "For APP CPU, reseted by PRO CPU";
        case RTCWDT_BROWN_OUT_RESET: return "Reset when the vdd voltage is not stable";
        case RTCWDT_RTC_RESET: return "RTC Watch dog reset digital core and rtc module";
	default: return "Uncnown reason";
	}
}

/* valve program PWM task  (one task for each valve)
 * @*arg is (int valve number)
 */
void valvePWMtask(void *arg){
	int num=(int)arg;
	TickType_t xLastWakeTime=xTaskGetTickCount ();
	DBG("======started v:%d",num);
	while(1) {
		if (Klp[num].is_pwm) {
			DBG("pwmON |%04.1f sec|",Klp[num].open_time);
			if (Klp[num].open_time>0.2) { //if time less 0.2 sec do nothing
				cmd2valve (num, cmd_open);				//turn-on valve
				vTaskDelayUntil( &xLastWakeTime, SEC_TO_TICKS(Klp[num].open_time));
			}

			if (Klp[num].is_pwm){
				DBG("pwmOFF|%04.1f sec|",Klp[num].close_time);
				if (Klp[num].close_time>0.2) { //if time less 0.2 sec do nothing
					cmd2valve (num, cmd_close); //turn-off valve
					vTaskDelayUntil( &xLastWakeTime, SEC_TO_TICKS(Klp[num].close_time));
				}
			}
		}

		if (!Klp[num].is_pwm)	{  // if no pwm
			DBG("suspend v:%d",num);
			vTaskSuspend(NULL);	// stop the task (it will be resumed when pwm is turn-on in func startKlpPwm() )
			DBG("resume  v:%d",num);
			xLastWakeTime = xTaskGetTickCount ();
		}
	}
}


void IRAM_ATTR timer0_group0_isr(void *para)
{
	int timer_idx = (int) para;
	uint32_t intr_status = TIMERG0.int_st_timers.val;

	if (intr_status & BIT(timer_idx)) {

	        /*Timer will reload counter value*/
	        TIMERG0.hw_timer[timer_idx].update = 1;
		/*We don't call a API here because they are not declared with IRAM_ATTR*/
		TIMERG0.int_clr_timers.t1 = 1;
		/* For a auto-reload timer, we still need to set alarm_en bit if we want to enable alarm again.*/
		TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;

		if (beepActive) {
			// Обработка пищалки
			beeperTime--;
			if (beeperTime <= 0) {
				beeperTime = 0;
				beepActive = false;
				GPIO.out_w1tc = (1 << GPIO_BEEP);
			}
		}	

		tic_counter++;
		if (tic_counter >= 1000) {
			tic_counter=0;
			uptime_counter++;
			if (rect_timer1>0) rect_timer1--; // Таймер для отсчета секунд 1
			else rect_timer1=0;
			if (timer_sec2>0) timer_sec2--;	// Таймер для отсчета секунд 2
			else timer_sec2=0;
			if (timer_sec3>0) timer_sec3--;	// Таймер для отсчета секунд 3
			else timer_sec3=0;
			CurFreq = gpio_counter;
			gpio_counter=0;
			//xQueueSendFromISR(timer_queue, &intr_status, NULL);
		}

	}
}

/**
 * Настройка аппаратного таймера из group0
 */
static void tg0_timer0_init()
{
	int timer_group = TIMER_GROUP_0;
	int timer_idx = TIMER_1;
	timer_config_t config;

	//timer_queue = xQueueCreate(10, sizeof(uint32_t));

	config.alarm_en = true;
	config.auto_reload = 1;
	config.counter_dir = TIMER_COUNT_UP;
	config.divider = TIMER_DIVIDER;
	config.intr_type = TIMER_INTR_LEVEL;
	config.counter_en = TIMER_PAUSE;

    /* Конфигурация таймера */
    timer_init(timer_group, timer_idx, &config);

    /* Ставим таймер на паузу */
    timer_pause(timer_group, timer_idx);

    /* Загружаем значение таймера */
    timer_set_counter_value(timer_group, timer_idx, 0);

    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, (TIMER_INTERVAL_SEC * TIMER_SCALE) - TIMER_FINE_ADJ);
    /* Разрешаем прерывания таймера */
    timer_enable_intr(timer_group, timer_idx);

    /* Устанавливаем обработчик прерывания */
    timer_isr_register(timer_group, timer_idx, timer0_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    /* Запускаем отсчет таймера */
    timer_start(timer_group, timer_idx);

    xTaskCreate(valvePWMtask, "valvePWMtask", 8192, NULL, 5, NULL);
}

// ISR triggered by GPIO edge at the end of each Alternating Current half-cycle.
// Used to reset the PWM timer, which synchronize the TRIAC operation with
// the mains frequency.  Also used to perform fading of PWM phase.
void IRAM_ATTR gpio_isr_handler(void* arg) 
{ 
	uint32_t intr_st = GPIO.status;
	if (intr_st & (1 << GPIO_DETECT_ZERO)) {
//                for (int i = 0; i < 100; ++i) {}	// delay
		if (!(GPIO.in & (1 << GPIO_DETECT_ZERO))) {
			// Zero the PWM timer at the zero crossing.
			LEDC.timer_group[0].timer[0].conf.rst = 1;
			LEDC.timer_group[0].timer[0].conf.rst = 0;
		
			if (Hpoint >= HMAX - TRIAC_GATE_MAX_CYCLES) { 
				// If hpoint if very close to the maximum value, ie mostly off, simply turn off 
				// the output to avoid glitch where hpoint exceeds duty. 
				LEDC.channel_group[0].channel[0].conf0.sig_out_en = 0; 
			} else { 
				LEDC.channel_group[0].channel[0].hpoint.hpoint = Hpoint; 
				LEDC.channel_group[0].channel[0].conf0.sig_out_en = 1; 
				LEDC.channel_group[0].channel[0].conf1.duty_start = 1; 
			} 

			gpio_counter++;
		}
	} else if (intr_st & (1 << GPIO_ALARM)) {
		// Авария от внешнего источника
		AlarmMode |= ALARM_EXT;
	}
	GPIO.status_w1tc = intr_st;
} 

/* Настройка и установка состояния GPIO для работы */
void setProcessGpio(int on)
{
	static char is_configured = 0;
	uint32_t pin_sel = 0;
	uint32_t gpio = getIntParam(DEFL_PARAMS, "processGpio");
	if (gpio <= 0 && gpio > 64)   return;

	pin_sel = 1<<gpio;

	if (!is_configured) {
		// Configure output gpio
		gpio_config_t io_conf;
		io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
		io_conf.mode = GPIO_MODE_OUTPUT;
		io_conf.pin_bit_mask = pin_sel;
		io_conf.pull_down_en = 0;
		io_conf.pull_up_en = 0;
		gpio_config(&io_conf);
		is_configured++;
	}
	gpio_set_level(gpio, on);	

}

/* Загрузка и установка параметров работы */
int paramSetup(void)
{
	int16_t pw;
	if (!nvsHandle) return -1;

	nvs_get_i16(nvsHandle, "SetPower", &pw);
	setPower(pw);
	nvs_get_i16(nvsHandle, "MainMode", (int16_t*)&MainMode);
	nvs_get_i16(nvsHandle, "MainStatus", &MainStatus);

	if (MODE_RECTIFICATION == MainMode && resetReason > POWERON_RESET) {
		// Восстановление при аварийной перезагрузке
		uint64_t v;
		ESP_ERROR_CHECK(nvs_get_u64(nvsHandle, "tempStabSR", &v));
		tempStabSR = v;
		nvs_get_u64(nvsHandle, "tempTube20Prev", &v);
		tempTube20Prev = v;
		nvs_get_i16(nvsHandle, "ProcChimSR", &ProcChimSR);
	}

	// Загрузка параметров
	if (param_load(DEFL_PARAMS, RECT_CONFIGURATION) < 0) {
		// Файл не найден - заполняем значениями по умолчанию
		return param_default(DEFL_PARAMS, RECT_CONFIGURATION);
	}
	return ESP_OK;
}

cJSON* getInformation(void)
{
	char data[80];
	const char *wo;
	cJSON *ja, *j, *jt;

	CurrentTime = time(NULL);
	localtime_r(&CurrentTime, &CurrentTm);
	ja = cJSON_CreateObject();
	cJSON_AddItemToObject(ja, "cmd", cJSON_CreateString("info"));
	snprintf(data, sizeof(data)-1, "%02d:%02d", CurrentTm.tm_hour, CurrentTm.tm_min);
	cJSON_AddItemToObject(ja, "time", cJSON_CreateString(data));
	snprintf(data, sizeof(data)-1, "%02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
	cJSON_AddItemToObject(ja, "MainMode", cJSON_CreateNumber(MainMode));
	cJSON_AddItemToObject(ja, "MainModeStr", cJSON_CreateString(getMainModeStr()));
	cJSON_AddItemToObject(ja, "MainStatus", cJSON_CreateNumber(MainStatus));
	cJSON_AddItemToObject(ja, "MainStatusStr", cJSON_CreateString(getMainStatusStr()));
	cJSON_AddItemToObject(ja, "uptime", cJSON_CreateString(data));
	cJSON_AddItemToObject(ja, "CurVolts", cJSON_CreateNumber(CurVolts));
	cJSON_AddItemToObject(ja, "CurPower", cJSON_CreateNumber(CurPower));
	cJSON_AddItemToObject(ja, "SetPower", cJSON_CreateNumber(SetPower));
	cJSON_AddItemToObject(ja, "CurFreq", cJSON_CreateNumber(CurFreq/2));
	cJSON_AddItemToObject(ja, "AlarmMode", cJSON_CreateString(getAlarmModeStr()));
	if (WaterOn<0) wo = "No data";
	else if (0 == WaterOn) wo = "Off";
	else wo = "On";
	cJSON_AddItemToObject(ja, "WaterOn", cJSON_CreateString(wo));
	cJSON_AddItemToObject(ja, "TempWaterIn", cJSON_CreateNumber(TempWaterIn));
	cJSON_AddItemToObject(ja, "TempWaterOut", cJSON_CreateNumber(TempWaterOut));
	cJSON_AddItemToObject(ja, "WaterFlow", cJSON_CreateNumber(WaterFlow));
	cJSON_AddItemToObject(ja, "heap", cJSON_CreateNumber(esp_get_free_heap_size()));
	if (bmpTemperature > 0 && bmpTruePressure > 0) {
		cJSON_AddItemToObject(ja, "bmpTemperature", cJSON_CreateNumber(bmpTemperature));
		snprintf(data, sizeof(data)-1, "%0.2f", bmpTruePressure/133.332);
		cJSON_AddItemToObject(ja, "bmpTruePressure", cJSON_CreateString(data));
		cJSON_AddItemToObject(ja, "bmpPressurePa", cJSON_CreateNumber(bmpTruePressure));
	}

	j = cJSON_CreateArray();
	cJSON_AddItemToObject(ja, "sensors", j);

	for (int i=0; i<MAX_DS; i++) {
		DS18 *d = &ds[i];
		if (!d->is_connected) continue;
		jt = cJSON_CreateObject();
		cJSON_AddItemToArray(j, jt);
		cJSON_AddItemToObject(jt, "id", cJSON_CreateNumber(d->id));
		cJSON_AddItemToObject(jt, "descr", cJSON_CreateString(d->description?d->description:""));
		cJSON_AddItemToObject(jt, "type_str", cJSON_CreateString(getDsTypeStr(d->type)));
		cJSON_AddItemToObject(jt, "type", cJSON_CreateNumber(d->type));
		snprintf(data, sizeof(data)-1, "%02.2f", d->Ce);
		cJSON_AddItemToObject(jt, "temp", cJSON_CreateString(data));

	}

	j = cJSON_CreateArray();
	cJSON_AddItemToObject(ja, "klapans", j);
	for (int i=0; i<MAX_KLP; i++) {
		int pwm = Klp[i].open_time+Klp[i].close_time;
		float pwm_percent = 0;
		if (Klp[i].open_time>0) {
			float p = pwm/Klp[i].open_time;
			if (p) pwm_percent = roundX(100/p,2);
		}

		jt = cJSON_CreateObject();
		cJSON_AddItemToArray(j, jt);
		cJSON_AddItemToObject(jt, "id", cJSON_CreateNumber(i));
		cJSON_AddItemToObject(jt, "is_pwm", cJSON_CreateNumber(Klp[i].is_pwm));
		cJSON_AddItemToObject(jt, "is_open", cJSON_CreateNumber(Klp[i].is_open));
		cJSON_AddItemToObject(jt, "pwm_time", cJSON_CreateNumber((int)pwm));
		cJSON_AddItemToObject(jt, "pwm_percent", cJSON_CreateNumber((int)(pwm_percent+0.5)));
	}

	if (MODE_RECTIFICATION == MainMode) {
		// Режим ректификации
		cJSON_AddItemToObject(ja, "rect_p_shim", cJSON_CreateNumber(ProcChimSR));
		float timeStabKolonna = fabs(getFloatParam(DEFL_PARAMS, "timeStabKolonna"));
		if (MainStatus == PROC_STAB) {
			snprintf(data, sizeof(data)-1, "%02d/%02.0f sec", uptime_counter-secTempPrev, timeStabKolonna);
			cJSON_AddItemToObject(ja, "rect_time_stab", cJSON_CreateString(data));
		}
		if (MainStatus == PROC_T_WAIT) {
			snprintf(data, sizeof(data)-1, "%02d sec", rect_timer1);
			cJSON_AddItemToObject(ja, "rect_timer1", cJSON_CreateString(data));
			snprintf(data, sizeof(data)-1, "%02.1f <-- %02.1f C", tempStabSR, getTube20Temp());
			cJSON_AddItemToObject(ja, "rect_t_stab", cJSON_CreateString(data));
		} else {
			snprintf(data, sizeof(data)-1, "%02.1f C", tempStabSR);
			cJSON_AddItemToObject(ja, "rect_t_stab", cJSON_CreateString(data));
		}
		if (bmpTruePressure > 0 && startPressure > 0) {
			double diff = startPressure - bmpTruePressure;
			snprintf(data, sizeof(data)-1, "%02.1f", diff);
			cJSON_AddItemToObject(ja, "pressureDiff", cJSON_CreateString(data));
		}

	}
	return ja;
}

// Отправка SMS
void sendSMS(char *text)
{
	request_t *req;
	int ret;
	char *post, *user, *hash, *phones;
	int s;
	if (!getIntParam(NET_PARAMS, "useSmsc")) return;

	user = getStringParam(NET_PARAMS, "smscUser");
	hash = getStringParam(NET_PARAMS, "smscHash");
	phones = getStringParam(NET_PARAMS, "smscPhones");


	if (!user || !hash || !phones) return;
	if (strlen(user)<=0 || strlen(hash)<=30 || strlen(phones)<=0) return;
	s = strlen(user) + strlen(hash) + strlen(phones) + strlen(text);
	post = malloc(s+30);
	if (!post) return;
	sprintf(post, "login=%s&psw=%s&phones=%s&mes=%s", user, hash, phones, text);

	ESP_LOGI(TAG, ">> SMS start");
	req = req_new("https://smsc.ru/sys/send.php"); 
	req_setopt(req, REQ_SET_METHOD, "POST");
	req_setopt(req, REQ_SET_POSTFIELDS, post);
	ret = req_perform(req);
	if (ret/100 > 2) {
		ESP_LOGI(TAG, "sms failed, error code: %d", ret);
	}
	req_clean(req);
	free(post);
	ESP_LOGI(TAG, "<< Sms Done");
}


// Установка рабочей мощности
void setPower(int16_t pw)
{
	int16_t mp;
	mp = getIntParam(DEFL_PARAMS, "maxPower");
	if (pw > mp) SetPower = mp;
	else SetPower = pw;

	if (pw > mp/2) {
		LEDC.channel_group[0].channel[0].duty.duty = (TRIAC_GATE_IMPULSE_CYCLES*3) << 4;
	} else {
		LEDC.channel_group[0].channel[0].duty.duty = TRIAC_GATE_IMPULSE_CYCLES << 4;
	}

	if (pw>0) setProcessGpio(1);
	else setProcessGpio(0);

	if (nvsHandle) {
		nvs_set_i16(nvsHandle, "SetPower", SetPower);
	}
}                                                   

static void setNewProcChimSR(int16_t newValue)
{
	ProcChimSR = newValue;
	if (nvsHandle) {
		nvs_set_i16(nvsHandle, "ProcChimSR", ProcChimSR);
	}

}

static void setTempStabSR(double newValue)
{
	uint64_t v;
	tempStabSR = newValue;
	if (nvsHandle) {
		v = (uint64_t) tempStabSR;
		ESP_ERROR_CHECK(nvs_set_u64(nvsHandle, "tempStabSR", v));
	}
}

static void setTempTube20Prev(double newValue)
{
	uint64_t v;
	tempTube20Prev = newValue;
	if (nvsHandle) {
		v = (uint64_t) tempTube20Prev;
		ESP_ERROR_CHECK(nvs_set_u64(nvsHandle, "tempTube20Prev", v));
	}

}

void setNewMainStatus(int16_t newStatus)
{
	MainStatus = newStatus;
	if (nvsHandle) {
		nvs_set_i16(nvsHandle, "MainStatus", MainStatus);
	}
}


// Установка нового режима работы
void setMainMode(int nm)
{
	main_mode new_mode = (main_mode) nm;
	if (new_mode == MainMode) return; // Не изменился
	MainMode = new_mode;
	if (nvsHandle) {
		nvs_set_i16(nvsHandle, "MainMode", nm);
	}

	switch (MainMode) {
	case MODE_IDLE:
		// Режим мониторинга
		ESP_LOGI(TAG, "Main mode: Idle.");
		setPower(0);
		setNewMainStatus(START_WAIT);
		break;
	case MODE_POWEERREG:
		// Режим регулятора мощности
		ESP_LOGI(TAG, "Main mode: Power reg.");
		setNewMainStatus(PROC_START);
		setPower(getIntParam(DEFL_PARAMS, "ustPowerReg"));
		break;
	case MODE_DISTIL:
		// Режим дистилляции
		ESP_LOGI(TAG, "Main mode: Distillation.");
		setNewMainStatus(START_WAIT);
		break;
	case MODE_RECTIFICATION:
		// Режим ректификации
		ESP_LOGI(TAG, "Main mode: Rectification.");
		setNewMainStatus(START_WAIT);
		break;
	case MODE_TESTKLP:
		// Режим тестирования клапанов
		ESP_LOGI(TAG, "Main mode: Test klp.");
		setNewMainStatus(START_WAIT);
		break;
	}
	myBeep(false);
}

// Ручная установка состояния конечного автомата
void setStatus(int next)
{
	if (next && MainStatus>=PROC_END) return;
	if (!next && MainStatus<= START_WAIT) return;

	switch (MainMode) {
	case MODE_DISTIL:
		// Режим дистилляции
		if (next) {
			if (MainStatus == START_WAIT) {
				setNewMainStatus(PROC_START);
			} else if (MainStatus == PROC_START) {
				startPressure = bmpTruePressure; // Фиксация атм. давления.
				setPower(getIntParam(DEFL_PARAMS, "maxPower"));	//  максимальная мощность для разгона
				setNewMainStatus(PROC_RAZGON);
			} else if (MainStatus == PROC_RAZGON) {
				openKlp(klp_water);		// Открытие клапана воды
				setPower(getIntParam(DEFL_PARAMS, "powerDistil"));	// Мощность дистилляции
				setNewMainStatus(PROC_DISTILL);
			} else if (MainStatus == PROC_DISTILL) {
				setPower(0);		// Снятие мощности с тэна
				secTempPrev = uptime_counter;
				setNewMainStatus(PROC_WAITEND);
			} else if (MainStatus == PROC_WAITEND) {
				closeAllKlp();		// Закрытие всех клапанов.
				setNewMainStatus(PROC_END);
			}
		} else {
			if (MainStatus == PROC_RAZGON) {
				setPower(0);		// Снятие мощности с тэна
				closeAllKlp();		// Закрытие всех клапанов.
				setNewMainStatus(START_WAIT);
			} else if (MainStatus == PROC_DISTILL) {
				closeAllKlp();		// Закрытие всех клапанов.
				setPower(getIntParam(DEFL_PARAMS, "maxPower"));	//  максимальная мощность для разгона
				setNewMainStatus(PROC_RAZGON);
			} else if (MainStatus == PROC_WAITEND) {
				openKlp(klp_water);		// Открытие клапана воды
				setPower(getIntParam(DEFL_PARAMS, "powerDistil"));	// Мощность дистилляции
				setNewMainStatus(PROC_DISTILL);
			} else if (MainStatus == PROC_END) {
				openKlp(klp_water);		// Открытие клапана воды
				secTempPrev = uptime_counter;
				setNewMainStatus(PROC_WAITEND);
			}
		}
		break;

	case MODE_RECTIFICATION:
		// Режим ректификации
		if (next) {
			if (MainStatus == START_WAIT) {
				setNewMainStatus(PROC_START);
			} else if (MainStatus == PROC_START) {
				startPressure = bmpTruePressure; // Фиксация атм. давления.
				setPower(getIntParam(DEFL_PARAMS, "maxPower"));	//  максимальная мощность для разгона
				setNewMainStatus(PROC_RAZGON);
			} else if (MainStatus == PROC_RAZGON) {
				openKlp(klp_water);	// Открытие клапана воды
				setPower(getIntParam(DEFL_PARAMS, "powerRect"));    // Устанавливаем мощность ректификации
				setNewMainStatus(PROC_STAB); // Ручной переход в режим стабилизации
			} else if (MainStatus == PROC_STAB) {
				setPower(getIntParam(DEFL_PARAMS, "powerRect"));   // Устанавливаем мощность ректификации
				secTempPrev = uptime_counter;
				closeKlp(klp_sr);	// Отключение клапана отбора товарного продукта
				// Устанавливаем медленный ШИМ клапан отбора хвостов и голов в соответвии с установками
				start_valve_PWMpercent(klp_glwhq,
					getFloatParam(DEFL_PARAMS, "timeChimRectOtbGlv"),
					getFloatParam(DEFL_PARAMS, "procChimOtbGlv"));

				setTempStabSR(getTube20Temp() ); // температура, относительно которой будем стабилизировать отбор
				setNewMainStatus(PROC_GLV); // Ручной переход в режим отбора голов
			} else if (MainStatus == PROC_GLV) {
				setTempStabSR(getTube20Temp());	// температура, относительно которой будем стабилизировать отбор
				closeKlp(klp_glwhq);  // Отключение клапана отбора голов/хвостов
				setNewProcChimSR( getIntParam(DEFL_PARAMS, "beginProcChimOtbSR") ); // Устанавливаем стартовый % отбора товарного продукта
				setNewMainStatus(PROC_T_WAIT);
			} else if (MainStatus == PROC_T_WAIT) {
				setNewMainStatus(PROC_SR);
			} else if (MainStatus == PROC_SR) {
				closeKlp(klp_sr); // Отключение клапана продукта
				// Устанавливаем 90% медленный ШИМ клапан отбора хвостов и голов
				start_valve_PWMpercent(klp_glwhq,
						getFloatParam(DEFL_PARAMS, "timeChimRectOtbGlv"),
						100);

				setNewMainStatus(PROC_HV);
			} else if (MainStatus == PROC_HV) {
				setPower(0);		// Снятие мощности с тэна
				closeKlp(klp_glwhq); 	// Отключение клапана отбора голов/хвостов
				setNewMainStatus(PROC_WAITEND);
			} else if (MainStatus == PROC_WAITEND) {
				setPower(0);		// Снятие мощности с тэна
				closeAllKlp();		// Закрытие всех клапанов.
				setNewMainStatus(PROC_END);
			}
		} else {
			if (MainStatus == PROC_RAZGON) {
				// Из разгона в режим ожидания запуска
				setPower(0);		// Снятие мощности с тэна
				closeAllKlp();		// Закрытие всех клапанов.
        			setNewMainStatus(START_WAIT);
			} else if (MainStatus == PROC_STAB) {
				// Из стабилизации в режим разгона
				setPower(getIntParam(DEFL_PARAMS, "maxPower"));	//  максимальная мощность для разгона
				setNewMainStatus(PROC_RAZGON);
			} else if (MainStatus == PROC_GLV) {
				setNewMainStatus(PROC_STAB);
			} else if (MainStatus == PROC_T_WAIT) {
				setNewMainStatus(PROC_GLV);
			} else if (MainStatus == PROC_SR) {
				setNewMainStatus(PROC_GLV);
			} else if (MainStatus == PROC_HV) {
				setNewMainStatus(PROC_SR);
			} else if (MainStatus == PROC_WAITEND) {
				// Переходим к отбору хвостов
				setPower(getIntParam(DEFL_PARAMS, "powerRect"));	// Устанавливаем мощность ректификации
				closeKlp(klp_sr);	// Отключение клапана отбора товарного продукта
				// Устанавливаем 90% медленный ШИМ клапан отбора хвостов и голов
				start_valve_PWMpercent(klp_glwhq,
						getFloatParam(DEFL_PARAMS, "timeChimRectOtbGlv"),
						100);

				setNewMainStatus(PROC_HV);
			} else if (MainStatus == PROC_END) {
				openKlp(klp_water);	// Открытие клапана воды
				secTempPrev = uptime_counter;
				setNewMainStatus(PROC_WAITEND);
			}
		}
		break;
	default:
		break;
	}
	if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(false);
}

/*
 * Функция возвращает значение ШИМ для отборв по "шпоре" (температуре)
 */
int16_t GetSrPWM(void)
{
	int16_t found = ProcChimSR;
	double t = getTube20Temp();

	for (int i=1; i<getIntParam(DEFL_PARAMS, "cntCHIM"); i++) {
		if (autoPWM[i-1].temp <= t && autoPWM[i].temp > t) {
			if (autoPWM[i-1].pwm > 0) {
				found = (t - autoPWM[i-1].temp) * 
					(autoPWM[i].pwm - autoPWM[i-1].pwm) / 
					(autoPWM[i].temp - autoPWM[i-1].temp) + autoPWM[i-1].pwm;
				return found;
			}
		}	
	}
	return found;
}

bool end_condition_SR(void){
	float tempEndRectOtbSR = getFloatParam(DEFL_PARAMS, "tempEndRectOtbSR");
	if (tempEndRectOtbSR>0)					// контроль по Т куба
		return (getCubeTemp() >= tempEndRectOtbSR);
	else							// контроль по Т20
		return (getTube20Temp() >= fabs(tempEndRectOtbSR) );
}

bool end_condition_head(void){
	float tEndRectOtbGlv =getFloatParam(DEFL_PARAMS, "tEndRectOtbGlv");
	if (tEndRectOtbGlv>0) 																				// если tEndRectOtbGlv положительное->это температура куба завершения отбора голов
		return  (getCubeTemp() >= tEndRectOtbGlv); 				// если Т куба достигла заданной Т окончания отбора голов
	else																											// если tEndRectOtbGlv отрицательное -> это кол-во минут времени отбора голов
		return ( (uptime_counter-secTempPrev) >= (fabs(tEndRectOtbGlv)*60) );	// время отбора голов истекло
}


// Обработка состояний в режиме ректификации
void Rectification(void)
{
	double t;
	float tempEndRectRazgon;
	char b[80];

	switch (MainStatus) {
	case START_WAIT:
		// Ожидание запуска процесса
		break;
	case PROC_START:
		// Начало процесса
		setNewMainStatus(PROC_RAZGON);
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
		setPower(getIntParam(DEFL_PARAMS, "maxPower"));	//  максимальная мощность для разгона
		 /* fall through */

	case PROC_RAZGON:
		// Разгон до рабочей температуры
		tempEndRectRazgon = getFloatParam(DEFL_PARAMS, "tempEndRectRazgon");
		startPressure = bmpTruePressure; // Фиксация атм. давления.
		if (tempEndRectRazgon > 0) t = getCubeTemp();
		else t = getTube20Temp();
		if (-1 == t) break;
		if (t < fabs(tempEndRectRazgon)) break;

		// Переход в режим стабилизации колонны
		openKlp(klp_water);	// Открытие клапана воды
		setPower(getIntParam(DEFL_PARAMS, "powerRect"));	// Устанавливаем мощность ректификации
		// Запоминаем температуру и время
		t = getTube20Temp();
                setTempStabSR(t);
		setTempTube20Prev(t);
		secTempPrev = uptime_counter;
		setNewMainStatus(PROC_STAB);
#ifdef DEBUG
		ESP_LOGI(TAG, "Switch state to stabilization.");
#endif
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
		 /* fall through */

	case PROC_STAB:
		{
		// Стабилизация колонны
		t = getTube20Temp();
		if (-1 == t) {
			ESP_LOGE(TAG, "Can't get cube or 20%% tube temperature!!");
			break;
		}

		if (t < 70) {
			// Колонна еще не прогрелась.
#ifdef DEBUG
			ESP_LOGI(TAG, "Cube or 20%% tube temperature less 70 dg.");
#endif
			break;
		}
		float timeStabKolonna = getFloatParam(DEFL_PARAMS, "timeStabKolonna");
		if (timeStabKolonna > 0) {
			// Время относительно последнего изменения температуры
			if (fabs(t - tempTube20Prev) < 0.2) {
				// Если текущая температура колонны равна температуре,
				// запомненной ранее в пределах погрешности в 0.2 градуса C
#ifdef DEBUG
				ESP_LOGI(TAG, "Stabillization %d of %02.0f sec.", uptime_counter-secTempPrev, fabs(timeStabKolonna));
#endif

				if (uptime_counter-secTempPrev<timeStabKolonna) {
					// С момента последнего измерения прошло меньше заданого
					// времени.
					break;
				}
				// Если с момента последнего измерения прошло больше
				// cекунд чем нужно, считаем, что температура в колонне
				// стабилизировалась и переходим к отбору голов
			} else {
#ifdef DEBUG
				ESP_LOGI(TAG, "Stab. temp. changed from %0.2f to %0.2f. Reseting timer.", tempTube20Prev, t);
#endif

				// Рассогласование температуры запоминаем
				// температуру и время
		                setTempStabSR(t);
				setTempTube20Prev(t);
				secTempPrev = uptime_counter;
				break;
			}
		} else {
			// Абсолютное значение времени
#ifdef DEBUG
			ESP_LOGI(TAG, "Stabillization %d of %0.0f.", uptime_counter-secTempPrev, fabs(timeStabKolonna));
#endif
			if (uptime_counter-secTempPrev < fabs(timeStabKolonna)) {
				// Если с момента начала стабилизации прошло меньше
				// заданного количества секунд - продолжаем ждать
				break;
			}
		}

		// Переходим к следующему этапу - отбору голов.
		setNewMainStatus(PROC_GLV);
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
		secTempPrev = uptime_counter;
		setPower(getIntParam(DEFL_PARAMS, "powerRect"));	// Устанавливаем мощность ректификации
		closeKlp(klp_sr);	// Отключение клапана отбора товарного продукта
		// Устанавливаем медленный ШИМ клапан отбора хвостов и голов в соответвии с установками
		start_valve_PWMpercent(klp_glwhq,
			getFloatParam(DEFL_PARAMS, "timeChimRectOtbGlv"),
			getFloatParam(DEFL_PARAMS, "procChimOtbGlv"));

		setTempStabSR(getTube20Temp());	// температура, относительно которой будем стабилизировать отбор
#ifdef DEBUG
		ESP_LOGI(TAG, "Switch to `glv` stage");
#endif

		}
		 /* fall through */

	case PROC_GLV:
		// Отбор головных фракций
		//  проверим: не пора ли завершать отбор голов?
		if (!end_condition_head())	break;
		// Окончание отбора голов
		closeKlp(klp_glwhq); 			// Отключение клапана отбора голов/хвостов
		setNewMainStatus(PROC_T_WAIT);		// Переходим к стабилизации температуры
		setNewProcChimSR(getIntParam(DEFL_PARAMS, "beginProcChimOtbSR")); // Устанавливаем стартовый % отбора товарного продукта
		setTempStabSR(getTube20Temp()); // температура, относительно которой будем стабилизировать отбор
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
#ifdef DEBUG
		ESP_LOGI(TAG, "Switch to `T wait` stage");
#endif
		 /* fall through */

	case PROC_T_WAIT:
		// Ожидание стабилизации температуры
		if (tempStabSR <= 0) setTempStabSR(28.5);

		if (0 == rect_timer1 && getIntParam(DEFL_PARAMS, "timeRestabKolonna") > 0) {
			// Если колонна слишком долго находится в режиме стопа,
			// то температуру стабилизации примем за новую
			setTempStabSR(getTube20Temp());
			if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
#ifdef DEBUG
			ESP_LOGI(TAG, "New temperature for stabilization: %0.2f", tempStabSR);
#endif

		}

		//---проверим: не пора ли завершить тело и перейти к хвостам?
		if (end_condition_SR())	{
			// Переходим к отбору хвостов
			closeKlp(klp_sr);	// Отключение клапана отбора товарного продукта
			// Устанавливаем ШИМ клапан отбора хвостов и голов
			start_valve_PWMpercent(klp_glwhq,
					getFloatParam(DEFL_PARAMS, "timeChimRectOtbGlv"),
					100); // хвост отбираем на все 100 !
       			setNewMainStatus(PROC_HV);
			if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
			break;
		}
		
		if (getTube20Temp() > tempStabSR) {
			break;
		}

		// Переход к отбору товарного продукта

		// Реализуется отбор по-шпоре, что в функции прописано то и будет возвращено.
		setNewProcChimSR(GetSrPWM());
		closeKlp(klp_glwhq); // Отключение клапана отбора голов/хвостов
		// Устанавливаем медленный ШИМ клапана продукта
		start_valve_PWMpercent
		  ( klp_sr, // клапан продукта
			getFloatParam(DEFL_PARAMS, "timeChimRectOtbSR"),// период ШИМ в сек
			ProcChimSR //%
		  );

		secTempPrev = uptime_counter;	// Запомним время, когда стабилизировалась температура
		setNewMainStatus(PROC_SR);	// Переход к отбору продукта
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
#ifdef DEBUG
		ESP_LOGI(TAG, "Switch state to SR. Temperature: %0.2f", tempStabSR);
#endif
                 /* fall through */

	case PROC_SR:
		// Отбор СР
		t = getTube20Temp();

		int minProcChimOtbSR = getIntParam(DEFL_PARAMS, "minProcChimOtbSR");

		if (t < tempStabSR) {	//если температура стала ниже Т стабилизации, то принимаем ее за новую Т стабилизации (ПБ)
			setTempStabSR(t);
		}

		if (t >= tempStabSR + getFloatParam(DEFL_PARAMS, "tempDeltaRect")) {
			// Температура превысила температуру стабилизации
			closeKlp(klp_sr); // Отключение клапана продукта
			int cntCHIM = getIntParam(DEFL_PARAMS, "cntCHIM");

			if (cntCHIM < 0) {
				// Запоминаем температуру, когда произошел стоп за вычетом 0.1 градуса.
				autoPWM[-cntCHIM].temp = t - 0.1;
				autoPWM[-cntCHIM].pwm = ProcChimSR;
				if (-cntCHIM < COUNT_PWM-1) cntCHIM--;
			}
			int decrementCHIM = getIntParam(DEFL_PARAMS, "decrementCHIM");
			if (decrementCHIM>=0) { 
				// Тогда уменьшаем  ШИМ указанное число процентов в абсолютном выражении
				setNewProcChimSR(ProcChimSR-decrementCHIM);
			} else {
				uint16_t v = (ProcChimSR * (-decrementCHIM))/100;
				// Процентное отношение может быть очень мало,
				// поэтому если получилось нулевое значение, то вычтем единицу.
				if (v<=0) v=1;
				setNewProcChimSR(ProcChimSR - v); // Тогда увеличиваем ШИМ на число процентов в относительном выражении
			}
			if (ProcChimSR < minProcChimOtbSR) setNewProcChimSR(minProcChimOtbSR);
			rect_timer1 = getIntParam(DEFL_PARAMS, "timeRestabKolonna"); // Установка таймера стабилизации
			setNewMainStatus(PROC_T_WAIT); // Переходим в режим стабилизации
			if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
#ifdef DEBUG
			ESP_LOGI(TAG, "Switch state to temperature re-stabilization.");
#endif
			break;
		} else {
			int timeAutoIncCHIM = getIntParam(DEFL_PARAMS, "timeAutoIncCHIM");
        		if (timeAutoIncCHIM>0 && (uptime_counter - secTempPrev) > timeAutoIncCHIM) {
				// Если температура не выросла более, чем за 10 минут, прибавим ШИМ на 5%
				if (ProcChimSR > minProcChimOtbSR) {
					// Шим прибавляем только если не дошли до минимального
					int incrementCHIM = getIntParam(DEFL_PARAMS, "incrementCHIM");
					if (incrementCHIM>=0) {
						// Абсолютное значение
						setNewProcChimSR(ProcChimSR + incrementCHIM);
					} else {
						// Проценты
						setNewProcChimSR(ProcChimSR + (ProcChimSR*(-incrementCHIM))/100);
					}
					if (ProcChimSR>95) setNewProcChimSR(95);
					// Устанавливаем 90% медленный ШИМ клапан отбора хвостов и голов
					start_valve_PWMpercent
					  ( klp_sr, // клапан продукта
						getFloatParam(DEFL_PARAMS, "timeChimRectOtbSR"),// период ШИМ в сек
						ProcChimSR //%
					  );

				}
				secTempPrev = uptime_counter;
			}
		}

		// проверим - можно ли продолжать отбор тела?
		if (! end_condition_SR())	{
			break;  // продолжаем отбор тела
		}

		// Температура в кубе превысила температуру при которой надо отбирать СР
		closeKlp(klp_sr); 			// Отключение клапана продукта
			start_valve_PWMpercent(klp_glwhq,
					getFloatParam(DEFL_PARAMS, "timeChimRectOtbGlv"),
					100);
		setNewMainStatus(PROC_HV);			// Переход к отбору хвостов
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
		 /* fall through */

	case PROC_HV:
		// Отбор хвостовых фракций
		t = getCubeTemp();
		if (t < getFloatParam(DEFL_PARAMS, "tempEndRect")) {
			break;
		}
		// Температура достигла отметки окончания ректификации
		// Переход к окончанию процесса
		setPower(0);		// Снятие мощности с тэна
		closeKlp(klp_glwhq); 	// Отключение клапана отбора голов/хвостов
		secTempPrev = uptime_counter;
		setNewMainStatus(PROC_WAITEND);
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
#ifdef DEBUG
		ESP_LOGI(TAG, "Switch state to wait End of Rectification.");
#endif
		 /* fall through */

	case PROC_WAITEND:
		// Отключение нагрева, подача воды для охлаждения
		if (uptime_counter - secTempPrev > 180) {
			setPower(0);		// Снятие мощности с тэна
			closeAllKlp();		// Закрытие всех клапанов.
			SecondsEnd = uptime_counter;
			sprintf(b, "Rectification complete, time: %02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
			sendSMS(b);
        		setNewMainStatus(PROC_END);
			if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
#ifdef DEBUG
			ESP_LOGI(TAG, "%s", b);
#endif
		}
		break;

	case PROC_END:
		// Окончание работы
		if (getIntParam(DEFL_PARAMS, "DIFFoffOnStop")) {
			xTaskCreate(&diffOffTask, "diff Off task", 4096, NULL, 1, NULL); // выключаем дифф
		}
		break;
	}
}

// Обработка состояний в режиме дистилляции
void Distillation(void)
{
	double t;
	char b[80];

	switch (MainStatus) {
	case START_WAIT:
		// Ожидание запуска процесса
		break;
	case PROC_START:
		// Начало процесса
		startPressure = bmpTruePressure; // Фиксация атм. давления.
		setNewMainStatus(PROC_RAZGON);
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
		setPower(getIntParam(DEFL_PARAMS, "maxPower"));	//  максимальная мощность для разгона
		 /* fall through */

	case PROC_RAZGON:
		// Разгон до рабочей температуры
		t = getCubeTemp();
		if (-1 == t) break;
		if (t < getFloatParam(DEFL_PARAMS, "tempEndRectRazgon")) break;

		// Открытие клапана воды
		openKlp(klp_water);

		setNewMainStatus(PROC_DISTILL);
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
		setPower(getIntParam(DEFL_PARAMS, "powerDistil"));	// Мощность дистилляции
		 /* fall through */

	case PROC_DISTILL:
		// Процесс дистилляции
		t = getCubeTemp();
		if (t < getFloatParam(DEFL_PARAMS, "tempEndDistil")) {
			break;
		}

		secTempPrev = uptime_counter;
		setNewMainStatus(PROC_WAITEND);			// Переход к окончанию процесса
		if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
		 /* fall through */

	case PROC_WAITEND:
		// Отключение нагрева, подача воды для охлаждения
		setPower(0);		// Снятие мощности с тэна
		if (uptime_counter - secTempPrev > 180) {
			setNewMainStatus(PROC_END);
			SecondsEnd = uptime_counter;
			if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(true);
		}
		break;

	case PROC_END:
		// Окончание работы
		setPower(0);		// Снятие мощности с тэна
		closeAllKlp();		// Закрытие всех клапанов.
		sprintf(b, "Distillation complete, time: %02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
		sendSMS(b);
		if (getIntParam(DEFL_PARAMS, "DIFFoffOnStop")) {
			xTaskCreate(&diffOffTask, "diff Off task", 4096, NULL, 1, NULL); // выключаем дифф
		}
		break;
	}
}


/*
 * send command (ON/OFF) to valvePWMtask
 */
void cmd2valve (int valve_num, valve_cmd_t cmd){
	static valveCMDmessage_t cmd_message;
	DBG("v:%d cmd:%d",valve_num, cmd);
	if (valve_num>=MAX_KLP) {
		ESP_LOGE(__func__, "incorrect valve num %d", valve_num);
		return;
	}
	if (valve_cmd_queue){
		cmd_message.cmd = cmd;
		cmd_message.valve_num=valve_num;
		if (xQueueSend( valve_cmd_queue, ( void * ) &cmd_message, ( TickType_t ) 10 )!= pdPASS){
			ESP_LOGE(__func__,"timeout of cmd sending");
		}
	}
	else {
		ESP_LOGE(__func__,"CMD queue doesn't exist");
	}
}

/*
 * Закрытие всех клапанов.
 */
void closeAllKlp(void)
{
	for (int i=0; i<MAX_KLP; i++)	closeKlp(i);
}

/*
 * Открытие клапана с выключением программного ШИМ
 */
void openKlp(int i)
{
	cmd2valve(i, cmd_open);
	Klp[i].is_pwm = false;
}

/*
 * Закрытие клапана с выключением программного ШИМ
 */
void closeKlp(int i)
{
	cmd2valve (i, cmd_close);
	Klp[i].is_pwm = false;
}

/*
 * Запуск шима клапана
 */
void startKlpPwm(int i, float topen, float tclose)
{
	if (i>=MAX_KLP) return;
	ESP_LOGI(TAG, "PWM klp %d %04.1f/%04.1f", i, topen, tclose);
	Klp[i].open_time = topen;	// Время в течении которого клапан открыт
	Klp[i].close_time = tclose;	// Время в течении которого клапан закрыт
	Klp[i].is_pwm = true;	// Запускаем медленный Шим режим

	if (! Klp[i].pwm_task_Handle){
		xTaskCreate(valvePWMtask, "valvePWMtask", 8192, (void *)i, 5, &Klp[i].pwm_task_Handle);	//запускаем задачу программного ШИМ клапана
		vTaskDelay( 100/portTICK_PERIOD_MS );
	}
	else
		vTaskResume( Klp[i].pwm_task_Handle);
}

/* Запуск программного ШИМ с параметрами
* @клапан
* @период в сек
* @процент времени открытия
*/
void start_valve_PWMpercent(int valve_num, int period_sec, int percent_open){
	float topened= (period_sec*percent_open+50)/100l;
	float tclosed= period_sec-topened;
	if ((topened < 0)||(topened<0)||(period_sec==0)){
		ESP_LOGE("startPWN", "incorrect param,period %d open %05.2f close %05.2f", period_sec, topened, tclosed);
		return;
	}
	startKlpPwm(valve_num, topened, tclosed);
}

static struct {
    struct arg_str *value;
    struct arg_end *end;
} set_args;

static int set_ct(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &set_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, set_args.end, argv[0]);
		return 1;
	}
	const char *values = set_args.value->sval[0];
	testCubeTemp = atof(values);
	emulate_devices = 1;
	ESP_LOGI(TAG, "New Cube temp: %f\n", testCubeTemp);
	return 0;
}

const esp_console_cmd_t set_ct_cmd = {
        .command = "t",
        .help = "Emulate cube temp",
        .hint = NULL,
        .func = &set_ct,
        .argtable = &set_args
};

/* 'version' command */
static int get_version(int argc, char **argv)
{
	esp_chip_info_t info;
	esp_chip_info(&info);
	printf("IDF Version:%s\r\n", esp_get_idf_version());
	printf("Chip info:\r\n");
	printf("\tmodel:%s\r\n", info.model == CHIP_ESP32 ? "ESP32" : "Unknow");
	printf("\tcores:%d\r\n", info.cores);
	printf("\tfeature:%s%s%s%s%d%s\r\n",
           info.features & CHIP_FEATURE_WIFI_BGN ? "/802.11bgn" : "",
           info.features & CHIP_FEATURE_BLE ? "/BLE" : "",
           info.features & CHIP_FEATURE_BT ? "/BT" : "",
           info.features & CHIP_FEATURE_EMB_FLASH ? "/Embedded-Flash:" : "/External-Flash:",
           spi_flash_get_chip_size() / (1024 * 1024), " MB");
	printf("\trevision number:%d\r\n", info.revision);
	return 0;
}

static void register_version()
{
    const esp_console_cmd_t cmd = {
        .command = "version",
        .help = "Get version of chip and SDK",
        .hint = NULL,
        .func = &get_version,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/** 'restart' command restarts the program */

static int restart(int argc, char **argv)
{
	ESP_LOGI(TAG, "Restarting");
	esp_restart();
}

static void register_restart()
{
	const esp_console_cmd_t cmd = {
		.command = "restart",
		.help = "Software reset of the chip",
		.hint = NULL,
		.func = &restart,
	};
	ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}



void console_task(void *arg)
{
	const char* prompt = LOG_COLOR_I "hd> " LOG_RESET_COLOR;



	/* Выключаем буферизацию stdin и stdout */
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	// Настройка консоли
	esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
	esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
	ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
	// Tell VFS to use UART driver
	esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
	// Инициализация консоли
	esp_console_config_t console_config = {
		.max_cmdline_args = 8,
		.max_cmdline_length = 256,
		.hint_color = atoi(LOG_COLOR_CYAN)
	};
	ESP_ERROR_CHECK(esp_console_init(&console_config));

	linenoiseSetMultiLine(1);
	linenoiseSetCompletionCallback(&esp_console_get_completion);
	linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);
	linenoiseHistorySetMaxLen(100);
	linenoiseHistoryLoad(HISTORY_PATH);
	/* Register commands */
	esp_console_register_help_command();

	int probe_status = linenoiseProbe();
	if (probe_status) {
	        linenoiseSetDumbMode(1);
	}

	set_args.value = arg_str1("v", "value", "<value>", "value to be stored");
	set_args.end = arg_end(2);
	ESP_ERROR_CHECK( esp_console_cmd_register(&set_ct_cmd) );
	register_version();
	register_restart();

	while (true) {
		char* line;
		int ret;
		line = linenoise(prompt);
		if (line) {
			int res;
			linenoiseHistoryAdd(line); // add to history
			linenoiseHistorySave(HISTORY_PATH); // save history
			ret = esp_console_run(line, &res);
			if (ret == ESP_ERR_NOT_FOUND) {
				printf("Unrecognized command\n");
			} else if (ret == ESP_OK && res != ESP_OK) {
				printf("Command returned non-zero error code: 0x%x\n", res);
			} else if (ret != ESP_OK) {
				printf("Internal error: 0x%x\n", ret);
			}
			linenoiseFree(line); // free line
		}
	}

}

// Основная точка входа в программу
void app_main(void)
{
	gpio_config_t io_conf;
	esp_err_t ret;

	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
        ESP_LOGI(TAG, "RAM left %d", esp_get_free_heap_size());


	ESP_ERROR_CHECK(ret = nvs_open("storage", NVS_READWRITE, &nvsHandle));
	if (ret != ESP_OK) nvsHandle = 0;

	// I2C init
	I2C_Init(I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
	task_i2cscanner();

	ESP_LOGI(TAG, "Initializing SPIFFS");
	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/s",
		.partition_label = NULL,
		.max_files = 5,
		.format_if_mount_failed = true
	};
	ret = esp_vfs_spiffs_register(&conf);
	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%d)", ret);
		}
		return;
	}

	// Получение причины (пере)загрузки
	resetReason = rtc_get_reset_reason(0);
	ESP_LOGI(TAG, "Reset reason: %s\n", getResetReasonStr());


	TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
	TIMERG0.wdt_feed=1;
	TIMERG0.wdt_wprotect=0;

	/* Запуск консоли */
	xTaskCreate(&console_task, "console_task", 8192, NULL, 1, NULL);

	/* Получаем конфигурацию сетевого доступа */
	param_load(NET_PARAMS, NET_CONFIGURATION);
	Hostname = getStringParam(NET_PARAMS, "host");
	httpUser = getStringParam(NET_PARAMS, "user");
	httpPassword = getStringParam(NET_PARAMS, "pass");
	httpSecure = getIntParam(NET_PARAMS, "secure");
	wsPeriod = getIntParam(NET_PARAMS, "wsPeriod");

	/* Настройка wifi */
	wifiSetup();

	/* Чтение настроек */
	paramSetup();

	/* Поиск и настройка датиков температуры и периодический опрос их */
	xTaskCreate(&ds_task, "ds_task", 4096, NULL, 1, NULL);

	/* Инициализация датчика давления bmp 180 */
	initBMP085();

	/* Настройка gpio детектора нуля сетевого напряжения */
	gpio_set_direction(GPIO_DETECT_ZERO, GPIO_MODE_INPUT);
	gpio_set_intr_type(GPIO_DETECT_ZERO, GPIO_INTR_NEGEDGE);
	gpio_set_pull_mode(GPIO_DETECT_ZERO, GPIO_PULLUP_ONLY);

	if (getIntParam(DEFL_PARAMS, "useExernalAlarm")) {
		/* Настройка gpio внешнего аварийного детектора */
		gpio_set_direction(GPIO_ALARM, GPIO_MODE_INPUT);
		gpio_set_intr_type(GPIO_ALARM, GPIO_INTR_NEGEDGE);
		gpio_set_pull_mode(GPIO_ALARM, GPIO_PULLUP_ONLY);
	}

	/* Настройка прерываний */
	ESP_ERROR_CHECK(gpio_isr_register(gpio_isr_handler, NULL, ESP_INTR_FLAG_LEVEL2, NULL));

	// Configure output gpio
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	/* Запуск http сервера */
	hd_httpd_init();	

	/* Настройка таймера */
	tg0_timer0_init();

	/* Запуск отображения на дисплее */
	hd_display_init();

	/* Настройка PZEM */
	xTaskCreate(&pzem_task, "pzem_task", 2048, NULL, 1, NULL);

	ledc_timer_config_t ledc_timer = {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
		.duty_resolution = LEDC_TIMER_10_BIT,
#else
		.bit_num = LEDC_TIMER_10_BIT,
#endif
		.freq_hz = LED_HZ*2,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.timer_num = LEDC_TIMER_0
	};
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        ledc_channel_config_t ledc_channel = {
            .gpio_num = GPIO_TRIAC,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = 0,
            .timer_sel = LEDC_TIMER_0,
            .duty = (1 << LEDC_TIMER_10_BIT) - 1,
            .intr_type = LEDC_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

	LEDC.channel_group[0].channel[0].duty.duty = TRIAC_GATE_IMPULSE_CYCLES << 4;
        // Initial brightness of 0, meaning turn TRIAC on at very end:
        LEDC.channel_group[0].channel[0].conf0.sig_out_en = 1;
	LEDC.channel_group[0].channel[0].conf1.duty_start = 1;

	// Настройка клапанов управления
	for (int i=0; i<MAX_KLP; i++) {
		ledc_channel_config_t ledc_channel = {
			.gpio_num = klp_gpio[i],
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.channel = i+1,
			.timer_sel = LEDC_TIMER_0,
			.duty = 0,
			.intr_type = LEDC_INTR_DISABLE,
		};
		Klp[i].is_open = false;
		Klp[i].channel = i+1;
		Klp[i].pwm_task_Handle=NULL;
		ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
		LEDC.channel_group[0].channel[i+1].conf0.sig_out_en = 0;
	}
	ledc_fade_func_install(0);

	valve_cmd_queue = xQueueCreate(10, sizeof(valveCMDmessage_t));			//---очередь команд открытия/закрытия клапанов
	if (! valve_cmd_queue){
		ESP_LOGE(__func__,"error of QUEUE creating!");
	}
	else {
		xTaskCreate(valveCMDtask, "valveCMDtask", 8192, NULL, 5, NULL);	//---задача включения/выключения клапанов по командам
	}

	/* задача контроля флагов тревоги и выключения дифф-автомата*/
	xTaskCreate(&alarmControlTask, "alarmControl", 4096, NULL, 1, NULL);
	if (getIntParam(DEFL_PARAMS, "DIFFoffOnStart")) {// при настройке "выключать дифф при старте"
		xTaskCreate(&diffOffTask, "diff Off task", 4096, NULL, 1, NULL); // выключаем дифф
	}

	ESP_ERROR_CHECK(gpio_intr_enable(GPIO_DETECT_ZERO));
	ESP_LOGI(TAG, "Enabled zero crossing interrupt.\n");

	if (getIntParam(DEFL_PARAMS, "useExernalAlarm")) {
		ESP_ERROR_CHECK(gpio_intr_enable(GPIO_ALARM));
	}

	if (getIntParam(DEFL_PARAMS, "beepChangeState")) myBeep(false);

	while (true) {
		cJSON *ja = getInformation();
     		char *r=cJSON_Print(ja);
		cgiWebsockBroadcast("/ws", r, strlen(r), WEBSOCK_FLAG_NONE);
		cJSON_Delete(ja);
		if (r) free(r);
		if (MODE_RECTIFICATION == MainMode) Rectification();
		else if (MODE_DISTIL == MainMode) Distillation();
		vTaskDelay(wsPeriod*1000/portTICK_PERIOD_MS);
	}
}

void valveCMDtask(void *arg){
	valveCMDmessage_t qcmd;
	ledc_channel_t ch;
	TickType_t xLastWakeTime=0, prevValveSwitch=0;

	while (1){
		if (xQueueReceive(valve_cmd_queue, &qcmd, portMAX_DELAY)!=pdTRUE) // ждем события на открытие/закрытие клапана
			continue;																									// если таймаут - повторим
		ch = Klp[qcmd.valve_num].channel;															// ledc-канал  клапана
		LEDC.channel_group[0].channel[ch].conf0.sig_out_en = 1;
		DBG("v:%d(ch:%d) cmd:%d",qcmd.valve_num,ch,qcmd.cmd);
		switch (qcmd.cmd) {
			case cmd_open:
				if (! Klp[qcmd.valve_num].is_open) { // если клапан закрыт то открываем
					// -------логика "тихого" включения
					if (getIntParam(DEFL_PARAMS,"klpSilentNode")) {
						ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, ch, VALVE_DUTY, VALVE_ON_FADE_TIME_MS);
						ledc_fade_start(LEDC_HIGH_SPEED_MODE, ch, LEDC_FADE_NO_WAIT);
						vTaskDelay(VALVE_ON_FADE_TIME_MS/portTICK_PERIOD_MS);
					} else {
						ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, VALVE_DUTY);
						ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
					}
					xLastWakeTime = xTaskGetTickCount ();// системное время включения, в тиках
	#ifdef DEBUG
					DBG(" ON:%d(%d ms)",qcmd.valve_num, (xLastWakeTime-prevValveSwitch)*portTICK_PERIOD_MS );
					prevValveSwitch=xLastWakeTime;
	#endif
					Klp[qcmd.valve_num].is_open = true;
					// ---------логика снижения ШИМ клапана после его включения---------
					if ((qcmd.valve_num == klp_water)||((qcmd.valve_num == klp_diff))) break; 	//если вода или дифф - не снижаем

					if ((KEEP_KLP_PWM==0)||(KEEP_KLP_PWM==100)) break;								// если настройка ШИМ удержания 0 или 100 - не снижаем
					if (	(Klp[qcmd.valve_num].is_pwm)																//если клапан в ШИМ
							&&																										//и время открытого его состояния
							(KEEP_KLP_DELAY_MS >= (Klp[qcmd.valve_num].open_time*1000))			//меньше времени задержки до перехода на удержание
						)	{																											// то не снижаем
						break;
					}

					vTaskDelayUntil( &xLastWakeTime, KEEP_KLP_DELAY_MS/portTICK_PERIOD_MS );//ждем включения механики клапана
					ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, ((VALVE_DUTY*KEEP_KLP_PWM)/100ul));
					ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
				}
				else {
					DBG("ON ignored");
				}
				break;

			case cmd_close:
				if (Klp[qcmd.valve_num].is_open){ // закрываем если открыт
#ifdef DEBUG
					xLastWakeTime = xTaskGetTickCount ();
					DBG("OFF:%d(%d ms)", qcmd.valve_num, (xLastWakeTime-prevValveSwitch)*portTICK_PERIOD_MS);
					prevValveSwitch =xLastWakeTime;
#endif
					ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, 0);
					ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
					LEDC.channel_group[0].channel[ch].conf0.sig_out_en = 0;
					Klp[qcmd.valve_num].is_open = false;
				}
				else {
					DBG("cmd close ignored");
				}
				break;
			default:
				break;
		}
	}
}
