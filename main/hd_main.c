#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include "freertos/xtensa_api.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
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
#include "apps/sntp/sntp.h"
#include "sh1106.h"
#include "ds.h"
#include "esp_platform.h"
#include "config.h"
#include "hd_wifi.h"
#include "hd_spi_i2c.h"
#include "hd_main.h"
#include "cgiupdate.h"
#include "cgiwebsocket.h"
#include "esp_request.h"

/* I2C address */
#ifndef DISPLAY_I2C_ADDR
#define DISPLAY_I2C_ADDR         0x3C//0x78
#endif
#define screenW 128
#define screenH 64

#define GPIO_INPUT_PIN_SEL  (1<<GPIO_DETECT_ZERO) 
#define GPIO_OUTPUT_PIN_SEL  (1<<GPIO_BEEP)

uint8_t PZEM_ip[4] = {192,168,1,1};
volatile int32_t Hpoint = HMAX;


#define TIMER_DIVIDER   80		/*!< Hardware timer clock divider */
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (0*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */ 
#define TIMER_INTERVAL_SEC   (0.001)   /*!< test interval for timer */ 


char *Hostname;		// Имя хоста
char *httpUser;		// Имя пользователя для http
char *httpPassword;	// Пароль для http
int httpSecure;		// Спрашивать пароль
char *smscUser;		// Имя пользователя для smsc
char *smscHash;		// Хэш пароля smsc
char *smscPhones;	// Номер телефона для Smsc
int useSmsc;		// Использовать smsc

int clockCenterX = screenW/2;
int clockCenterY = ((screenH-16)/2)+16;   // top yellow part is 16 px height
int clockRadius = 23;

unsigned char ds1820_devices;                  // Количество датчиков ds18b20

// клапана
unsigned char klp_gpio[4] =  {26, 27, 32, 33};
klp_list Klp[MAX_KLP];		// Список клапанов.

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

int16_t maxPower;		// Номинальная мощность ТЭНов
int16_t ustPowerReg = 700;	// Мощность в режиме регулятора мощности
double tempEndRectRazgon;	// Температура окончания режима разнона
int16_t powerRect = 1000;	// Мощность ректификации
double tEndRectOtbGlv=83;       // Температура окончания отбора голов
int16_t timeChimRectOtbGlv=20;	// Период для шим'a отбора голов
int16_t procChimOtbGlv=5;	// Процент ШИМ отбора голов
int16_t minProcChimOtbSR=20;	// Минимальный процент ШИМ'а отбора СР
int16_t beginProcChimOtbSR=40;	// Начальный процент ШИМ отбора СР
int16_t timeChimRectOtbSR=10;	// Период для шим'a отбора СР
double tempDeltaRect=0.3;	// Дельта температуры для ректификации
double tempEndRectOtbSR=96.5;	// Температура окончания отбора СР
double tempEndRect=99.5;	// Температура в кубе, при которой прекращается ректификация
int16_t p_MPX5010=0;		// Поправка для датчика давления
int16_t timeStabKolonna=900;	// Время стабилизации колонны
int16_t timeRestabKolonna=1800;	// Таймаут для повторной стабилизации колонны
int16_t pShim=90;		// Процент 100 Гц шим, для защиты клапанов от перегрева
bool    klpSilentNode = true;	// Режим плавного открытия клапанов.
int16_t urovenProvodimostSR=0;	// Уровень проводимости, меньше которого переходим к СР
int16_t cntCHIM=-4;		// Количество значений в таблице обучения ШИМ
int16_t decrementCHIM=10;	// Шаг ушеньшения % ШИМ'a при отборе СР
int16_t incrementCHIM=5;	// Шаг увеличения % ШИМ'a при отборе СР
int16_t timeAutoIncCHIM=600;	// Макс время стабилизации до увеличения ШИМ'а СР 
int16_t alarmMPX5010=0;		// Давление при котором надо выдавать сингал тревоги
bool beepChangeState = true;	// Звуковой сигнал при смене состояния процесса 

int16_t powerDistil = 1000;	// Мощность дестиляции
double tempEndDistil=99.5;	// Температура в кубе, при которой прекращается дестиляции

// Динамические параметры
double tempTube20Prev;		// Запомненое значение температуры в колонне
uint32_t secTempPrev;		// Отметка времени измеренной температуры 
double  tempStabSR;		// Температура, относительно которой стабилизируется отбор СР
int16_t ProcChimSR;		// Текущий процент отбора СР
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

int16_t timer_sec1=0;		// Таймер для отсчета секунд 1
int16_t timer_sec2=0;		// Таймер для отсчета секунд 2
int16_t timer_sec3=0;		// Таймер для отсчета секунд 3
int32_t SecondsEnd;		// Время окончания процесса 

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


/*
 * Функция вывода информации на первый экран
 */
void mainFrame(OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	char b[80];
	double f;
	oledSetTextAlignment(TEXT_ALIGN_LEFT);
	oledSetFont(ArialMT_Plain_16);
	sprintf(b, "U: %02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
	oledDrawString(x , 14 + y, b);
	f = getCubeTemp();
	sprintf(b, "Cube: %0.1f", f);
	oledDrawString(x , 30 + y, b);
	sprintf(b, "P: %d", CurPower);
	oledDrawString(x , 46 + y, b);

}


void addressFrame(OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	wifi_mode_t mode;
	tcpip_adapter_if_t ifx = TCPIP_ADAPTER_IF_AP;
	tcpip_adapter_ip_info_t ip;
	char b[255];

	oledSetTextAlignment(TEXT_ALIGN_LEFT);
	oledSetFont(ArialMT_Plain_10);

	esp_wifi_get_mode(&mode);
	if (WIFI_MODE_STA == mode) ifx = TCPIP_ADAPTER_IF_STA;

	memset(&ip, 0, sizeof(tcpip_adapter_ip_info_t));
	if (tcpip_adapter_get_ip_info(ifx, &ip) == 0) {
		sprintf(b, "IP:"IPSTR, IP2STR(&ip.ip));
		oledDrawString(x , 13 + y, b);
		sprintf(b, "MASK:"IPSTR, IP2STR(&ip.netmask));
		oledDrawString(x , 24 + y, b);
		sprintf(b, "GW:"IPSTR, IP2STR(&ip.gw));
		oledDrawString(x , 35 + y, b);
	}
	sprintf(b, "Uptime: %02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
	oledDrawString(x , 46 + y, b);
}

void temperatureFrame(OLEDDisplayUiState* state, int16_t x, int16_t y)
{
	char b[255];
	uint8_t offset=1;

	oledSetTextAlignment(TEXT_ALIGN_LEFT);
	oledSetFont(ArialMT_Plain_10);
	for (int i=0; i<MAX_DS; i++) {
		DS18 *d = &ds[i];
		if (!d->is_connected) continue;
		sprintf(b, " %d %s: %.2f", d->id+1, getDsTypeStr(d->type), d->Ce);
		oledDrawString(x , offset*11 + y+2, b);
		offset++;
	}

}


// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = {mainFrame, addressFrame, temperatureFrame};
// how many frames are there?
int frameCount = 3;

void display_task(void *pvParameter)
{
	if (I2C_detect[DISPLAY_I2C_ADDR]) Display_Init(DISPLAY_I2C_ADDR);
	else Display_Init(0);
	setTargetFPS(10);
	setIndicatorPosition(TOP);
	setIndicatorDirection(LEFT_RIGHT);
	setFrameAnimation(SLIDE_LEFT);
	setFrames(frames, frameCount);
	oledFlipScreenVertically();

	while (1) {
		UIupdate();
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}







/* Расчет CRC для pzem */
static uint8_t PZEM_crc(uint8_t *data, uint8_t sz)
{
	uint16_t crc = 0;
	uint8_t i;
	for (i=0; i<sz; i++) crc += *data++;
	return (uint8_t)(crc & 0xFF);
}

static int PZEM_command(uint8_t cmd, uint8_t data, uint8_t resp, uint8_t *result)
{
	struct PZEMCommand pzem;
	uint8_t *bytes;
	uint8_t buf[PZEM_RESPONSE_SIZE];
	int len, i, readed = 0, ret = -2;

	pzem.command = cmd;
        for(i=0; i<sizeof(pzem.addr); i++) pzem.addr[i] = PZEM_ip[i];
        pzem.data = data;
        bytes = (uint8_t*)&pzem;
        pzem.crc = PZEM_crc(bytes, sizeof(pzem) - 1);

	uart_flush(UART_NUM_1);
	uart_write_bytes(UART_NUM_1, (const char*) bytes, sizeof(pzem));

	for (i = 0; i < 2; i++) {
		len = uart_read_bytes(UART_NUM_1, &buf[readed], sizeof(buf)-readed,
			PZEM_DEFAULT_READ_TIMEOUT / portTICK_RATE_MS);
                readed += len;
                if (readed < PZEM_RESPONSE_SIZE) continue;
                if (buf[6] != PZEM_crc(buf, readed-1)) {
                        ret = -1;
                        break;
                }
                if (buf[0] != resp) {
                        ret = -1;
                        break;
                }
		if (result) {
                        for (i=0; i<PZEM_RESPONSE_DATA_SIZE; i++) {
                                result[i] = buf[1 + i];
                        }
                }
                ret = 0;
                break;
	}
	return ret;
}

float PZEM_setAddress(void)
{
	return PZEM_command(PZEM_SET_ADDRESS, 0, RESP_SET_ADDRESS, NULL);
}

float PZEM_voltage(void)
{
        uint8_t data[PZEM_RESPONSE_DATA_SIZE];
        if (PZEM_command(PZEM_VOLTAGE, 0, RESP_VOLTAGE, data)) {
                return PZEM_ERROR_VALUE;
        }
        return (data[0] << 8) + data[1] + (data[2] / 10.0);
}

float PZEM_current(void)
{
        uint8_t data[PZEM_RESPONSE_DATA_SIZE];
        if (PZEM_command(PZEM_CURRENT, 0, RESP_CURRENT, data)) {
                return PZEM_ERROR_VALUE;
        }
        return (data[0] << 8) + data[1] + (data[2] / 100.0);
}

float PZEM_power(void)
{
        uint8_t data[PZEM_RESPONSE_DATA_SIZE];
        if (PZEM_command(PZEM_POWER, 0, RESP_POWER, data)) {
                return PZEM_ERROR_VALUE;
        }
        return (data[0] << 8) + data[1];
}

float PZEM_energy(void)
{
        uint8_t data[PZEM_RESPONSE_DATA_SIZE];
        if (PZEM_command(PZEM_ENERGY, 0, RESP_ENERGY, data)) {
                return PZEM_ERROR_VALUE;
        }
        return ((uint32_t)data[0] << 16) + ((uint16_t)data[1] << 8) + data[2];
}

void pzem_task(void *arg)
{
	float v;
	int cnt =0;

	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 122,
	};
	uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, PZEM_TXD, PZEM_RXD, -1, -1);
	uart_driver_install(UART_NUM_1, UART_FIFO_LEN+2, 0, 0, NULL, 0);

	PZEM_setAddress();
	while(1) {
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
			SetPower = 0;
		}

		if (SetPower) {
			if (MainMode == MODE_IDLE) SetPower = 0;	// В режиме монитора выключаем нагрев
			if (PROC_END == MainStatus) SetPower = 0; // Режим окончания работы - отключение нагрузки
		}

		if (CurPower<5 && Hpoint<=TRIAC_GATE_MAX_CYCLES && SetPower > 0) {
			// Отслеживание отключенной нагрузки
			cnt ++;
			if (cnt>10) AlarmMode |= ALARM_NOLOAD;
		} else {
			cnt = 0;
			AlarmMode &= ~(ALARM_NOLOAD);
		}

		if (SetPower <= 0) {
			Hpoint = HMAX;
		} else {
			// Проверка рассинхронизации мощности
			int delta = abs(SetPower - CurPower);
			int p5 = SetPower/20;
			char inc = SetPower > CurPower;

			if (delta > 200 ) {
				if (inc) Hpoint -= 50; 
				else Hpoint += 50;
			} else if (delta > 50) {
				if (inc) Hpoint -= 10; 
				else Hpoint += 10;
			} else if (delta > 10) {
				if (inc) Hpoint -=3; 
				else Hpoint +=3;
			} else if (delta >= p5) {
				if (inc) Hpoint --; 
				else Hpoint ++;
			}
		}
		if (Hpoint >= HMAX - TRIAC_GATE_MAX_CYCLES) {
			Hpoint = HMAX - 1 - TRIAC_GATE_MAX_CYCLES;
		}
		if (Hpoint<TRIAC_GATE_MAX_CYCLES) Hpoint=TRIAC_GATE_MAX_CYCLES;

	}
}


const char *getMainModeStr(void)
{
	switch (MainMode) {
        case MODE_IDLE:	return "Монитор";
        case MODE_POWEERREG: return "Регулятор мощности";
	case MODE_DISTIL: return "Дестилляция";
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
		case PROC_START: return "Стабилизация мощности";
		default: return "Завершение работы";
		}
		break;
	case MODE_DISTIL:
		switch (MainStatus) {
		case START_WAIT: return "Ожидание запуска процесса";
		case PROC_START: return "Начало процесса";
		case PROC_RAZGON: return "Разгон до рабочей температуры";
		case PROC_DISTILL: return "Дестилляция";
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
	strcat(str,"</b>");
	return str;
}


xQueueHandle timer_queue;

static void timer_example_evt_task(void *arg)
{
	uint32_t evt;
	while(1) {
		xQueueReceive(timer_queue, &evt, portMAX_DELAY);
		for (int i=0; i<MAX_KLP; i++) {
			if (!Klp[i].is_pwm) continue;
			Klp[i].timer_sec++;

			if (Klp[i].is_open) {
				// Текущее состояние - клапан открыт
				if (Klp[i].timer_sec > Klp[i].open_time) {
					// Время открытия клапана истекло
					if (Klp[i].close_time <= 0) continue;
					ESP_LOGI(TAG, "PWM klp %d -> close", i);
					// Закрываем клапан
					int ch = Klp[i].channel;
					ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, 0);
					ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
				        LEDC.channel_group[0].channel[ch].conf0.sig_out_en = 0;
					Klp[i].is_open = false;
					Klp[i].timer_sec = 0;
				}
			} else {
				// Текущее состояние - клапан закрыт
				if (Klp[i].timer_sec > Klp[i].close_time) {
					// Пора открывать клапан
					if (Klp[i].open_time <= 0) continue;
					ESP_LOGI(TAG, "PWM klp %d -> open", i);
					// Открываем клапан
					int ch = Klp[i].channel;
				        LEDC.channel_group[0].channel[ch].conf0.sig_out_en = 1;
					if (klpSilentNode) {
						ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, ch, 1023, 1000);
						ledc_fade_start(LEDC_HIGH_SPEED_MODE, ch, LEDC_FADE_NO_WAIT);
					} else {
						ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, 1023);
						ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
					}
					Klp[i].is_open = true;
                			Klp[i].timer_sec = 0;
				}
			}
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
			if (timer_sec1>0) timer_sec1--;	// Таймер для отсчета секунд 1
			else timer_sec1=0;
			if (timer_sec2>0) timer_sec2--;	// Таймер для отсчета секунд 2
			else timer_sec2=0;
			if (timer_sec3>0) timer_sec3--;	// Таймер для отсчета секунд 3
			else timer_sec3=0;
			CurFreq = gpio_counter;
			gpio_counter=0;
			xQueueSendFromISR(timer_queue, &intr_status, NULL);
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

	timer_queue = xQueueCreate(10, sizeof(uint32_t));

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

    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
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
	}
	GPIO.status_w1tc = intr_st;
} 

/* Сброс параметров в значение по умолчанию */
int param_default(void)
{
	FILE *f = fopen(RECT_CONFIGURATION, "w");
	if (f) {
		cJSON *ja = cJSON_CreateObject();
		cJSON_AddItemToObject(ja, "maxPower", cJSON_CreateNumber(2000));
		cJSON_AddItemToObject(ja, "ustPowerReg", cJSON_CreateNumber(900));
		cJSON_AddItemToObject(ja, "tempEndRectRazgon", cJSON_CreateNumber(83.0));
		cJSON_AddItemToObject(ja, "powerRect", cJSON_CreateNumber(1000));
		cJSON_AddItemToObject(ja, "tEndRectOtbGlv", cJSON_CreateNumber(85.4));
		cJSON_AddItemToObject(ja, "timeChimRectOtbGlv", cJSON_CreateNumber(20));
		cJSON_AddItemToObject(ja, "procChimOtbGlv", cJSON_CreateNumber(5));
		cJSON_AddItemToObject(ja, "minProcChimOtbSR", cJSON_CreateNumber(20));
		cJSON_AddItemToObject(ja, "beginProcChimOtbSR", cJSON_CreateNumber(40));
		cJSON_AddItemToObject(ja, "timeChimRectOtbSR", cJSON_CreateNumber(10));
		cJSON_AddItemToObject(ja, "tempDeltaRect", cJSON_CreateNumber(0.3));
		cJSON_AddItemToObject(ja, "tempEndRectOtbSR", cJSON_CreateNumber(96.5));
		cJSON_AddItemToObject(ja, "tempEndRect", cJSON_CreateNumber(99.5));
		cJSON_AddItemToObject(ja, "p_MPX5010", cJSON_CreateNumber(0));
		cJSON_AddItemToObject(ja, "timeStabKolonna", cJSON_CreateNumber(900));
		cJSON_AddItemToObject(ja, "timeRestabKolonna", cJSON_CreateNumber(1800));
		cJSON_AddItemToObject(ja, "pShim", cJSON_CreateNumber(90));
		cJSON_AddItemToObject(ja, "klpSilentNode", cJSON_CreateBool(true));
		cJSON_AddItemToObject(ja, "urovenProvodimostSR", cJSON_CreateNumber(0));
		cJSON_AddItemToObject(ja, "cntCHIM", cJSON_CreateNumber(-4));
		cJSON_AddItemToObject(ja, "decrementCHIM", cJSON_CreateNumber(10));
		cJSON_AddItemToObject(ja, "incrementCHIM", cJSON_CreateNumber(5));
		cJSON_AddItemToObject(ja, "timeAutoIncCHIM", cJSON_CreateNumber(600));
		cJSON_AddItemToObject(ja, "alarmMPX5010", cJSON_CreateNumber(0));
		cJSON_AddItemToObject(ja, "beepChangeState", cJSON_CreateBool(true));

		cJSON_AddItemToObject(ja, "powerDistil", cJSON_CreateNumber(1000));
		cJSON_AddItemToObject(ja, "tempEndDistil", cJSON_CreateNumber(99.5));

		char *r=cJSON_Print(ja);
		fprintf(f, "%s", r);
		if (r) free(r);
		cJSON_Delete(ja);
		fclose(f);
	}
	return 0;
}

/* Загрузка и установка параметров работы */
int param_setup(void)
{
	esp_err_t err;
	nvs_handle nvs;
	struct stat st;
	char *buff = NULL;
	size_t size;
	FILE *f;
	cJSON *v;

	ESP_ERROR_CHECK(err = nvs_open("storage", NVS_READWRITE, &nvs));
	if (err != ESP_OK) return err;
	nvs_get_i16(nvs, "SetPower", &SetPower);
	nvs_get_i16(nvs, "MainMode", (int16_t*)&MainMode);
	nvs_get_i16(nvs, "MainStatus", &MainStatus);
	nvs_close(nvs);

	if (stat(RECT_CONFIGURATION, &st) != 0) {
		// Файл не найден - заполняем значениями по умолчанию
		return param_default();
	}
	f = fopen(RECT_CONFIGURATION, "r");
	buff = malloc(st.st_size+2);
	if (!buff) {
		fclose(f);
		return -2;
	}
	size = fread(buff, 1, st.st_size, f);
	buff[size]=0;
	fclose(f);

	cJSON *root = cJSON_Parse(buff);
	free(buff);
	if (!root) return -3;

	v = cJSON_GetObjectItem(root, "maxPower");
	if (v) maxPower = v->valueint;

	v = cJSON_GetObjectItem(root, "ustPowerReg");
	if (v) ustPowerReg = v->valueint;

	v = cJSON_GetObjectItem(root, "tempEndRectRazgon");
	if (v) tempEndRectRazgon = v->valuedouble;

	v = cJSON_GetObjectItem(root, "powerRect");
	if (v) powerRect = v->valueint;

	v = cJSON_GetObjectItem(root, "tEndRectOtbGlv");
	if (v) tEndRectOtbGlv = v->valuedouble;

	v = cJSON_GetObjectItem(root, "timeChimRectOtbGlv");
	if (v) timeChimRectOtbGlv = v->valueint;

	v = cJSON_GetObjectItem(root, "procChimOtbGlv");
	if (v) procChimOtbGlv = v->valueint;

	v = cJSON_GetObjectItem(root, "minProcChimOtbSR");
	if (v) minProcChimOtbSR = v->valueint;

	v = cJSON_GetObjectItem(root, "beginProcChimOtbSR");
	if (v) beginProcChimOtbSR = v->valueint;

	v = cJSON_GetObjectItem(root, "timeChimRectOtbSR");
	if (v) timeChimRectOtbSR = v->valueint;

	v = cJSON_GetObjectItem(root, "tempDeltaRect");
	if (v) tempDeltaRect = v->valuedouble;

	v = cJSON_GetObjectItem(root, "tempEndRectOtbSR");
	if (v) tempEndRectOtbSR = v->valuedouble;

	v = cJSON_GetObjectItem(root, "tempEndRect");
	if (v)  tempEndRect = v->valuedouble;

	v = cJSON_GetObjectItem(root, "p_MPX5010");
	if (v) p_MPX5010 = v->valueint;

	v = cJSON_GetObjectItem(root, "timeStabKolonna");
	if (v) timeStabKolonna = v->valueint;

	v = cJSON_GetObjectItem(root, "timeRestabKolonna");
	if (v) timeRestabKolonna = v->valueint;

	v = cJSON_GetObjectItem(root, "pShim");
	if (v) pShim = v->valueint;

	v = cJSON_GetObjectItem(root, "klpSilentNode");
	if (v) beepChangeState = v->valueint;

	v = cJSON_GetObjectItem(root, "urovenProvodimostSR");
	if (v) urovenProvodimostSR = v->valueint;

	v = cJSON_GetObjectItem(root, "cntCHIM");
	if (v) cntCHIM = v->valueint;

	v = cJSON_GetObjectItem(root, "decrementCHIM");
	if (v) decrementCHIM = v->valueint;

	v = cJSON_GetObjectItem(root, "incrementCHIM");
	if (v) incrementCHIM = v->valueint;

	v = cJSON_GetObjectItem(root, "timeAutoIncCHIM");
	if (v) timeAutoIncCHIM = v->valueint;

	v = cJSON_GetObjectItem(root, "alarmMPX5010");
	if (v) alarmMPX5010 = v->valueint;

	v = cJSON_GetObjectItem(root, "beepChangeState");
	if (v) beepChangeState = v->valueint;

	v = cJSON_GetObjectItem(root, "powerDistil");
	if (v) powerDistil = v->valueint;

	v = cJSON_GetObjectItem(root, "tempEndDistil");
	if (v)  tempEndDistil = v->valuedouble;


	cJSON_Delete(root);
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
		cJSON_AddItemToObject(jt, "temp", cJSON_CreateNumber(d->Ce));
	}

	j = cJSON_CreateArray();
	cJSON_AddItemToObject(ja, "klapans", j);
	for (int i=0; i<MAX_KLP; i++) {
		int pwm = Klp[i].open_time+Klp[i].close_time;
		float pwm_percent = 0;
		if (Klp[i].open_time>0) {
			float p = pwm/Klp[i].open_time;
			if (p) pwm_percent = 100/p;
		}

		jt = cJSON_CreateObject();
		cJSON_AddItemToArray(j, jt);
		cJSON_AddItemToObject(jt, "id", cJSON_CreateNumber(i));
		cJSON_AddItemToObject(jt, "is_pwm", cJSON_CreateNumber(Klp[i].is_pwm));
		cJSON_AddItemToObject(jt, "is_open", cJSON_CreateNumber(Klp[i].is_open));
		cJSON_AddItemToObject(jt, "pwm_time", cJSON_CreateNumber(pwm));
		cJSON_AddItemToObject(jt, "pwm_percent", cJSON_CreateNumber(pwm_percent));
	}

	if (MODE_RECTIFICATION == MainMode) {
		// Режим ректификации
	}
	return ja;
}

// Отправка SMS
void sendSMS(char *text)
{
	request_t *req;
	int ret;
	char *post;
	int s;

	if (!useSmsc || !smscUser || !smscHash || !smscPhones) return;
	s = strlen(smscUser) + strlen(smscHash) + strlen(smscPhones) + strlen(text);
	post = malloc(s+30);
	if (!post) return;
	sprintf(post, "login=%s&psw=%s&phones=%s&mes=%s", 
		smscUser, smscHash, smscPhones, text);

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
	nvs_handle nvs;
	if (pw > maxPower) SetPower = maxPower;
	else SetPower = pw;
	if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK) {
		nvs_set_i16(nvs, "SetPower", SetPower);
		nvs_close(nvs);
	}
}                                                   


// Установка нового режима работы
void setMainMode(int nm)
{
	main_mode new_mode = (main_mode) nm;
	nvs_handle nvs;	
	if (new_mode == MainMode) return; // Не изменился
	MainMode = new_mode;
	if (nvs_open("storage", NVS_READWRITE, &nvs) == ESP_OK) {
		nvs_set_i16(nvs, "MainMode", nm);
		nvs_close(nvs);
	}

	switch (MainMode) {
	case MODE_IDLE:
		// Режим мониторинга
		ESP_LOGI(TAG, "Main mode: Idle.");
		setPower(0);
		MainStatus = START_WAIT;
		break;
	case MODE_POWEERREG:
		// Режим регулятора мощности
		ESP_LOGI(TAG, "Main mode: Power reg.");
		MainStatus = PROC_START;
		setPower(ustPowerReg);
		break;
	case MODE_DISTIL:
		// Режим дестилляции
		ESP_LOGI(TAG, "Main mode: Distillation.");
		MainStatus = START_WAIT;
		break;
	case MODE_RECTIFICATION:
		// Режим ректификации
		ESP_LOGI(TAG, "Main mode: Rectification.");
		MainStatus = START_WAIT;
		break;
	case MODE_TESTKLP:
		// Режим тестирования клапанов
		ESP_LOGI(TAG, "Main mode: Test klp.");
		MainStatus = START_WAIT;
		break;
	}
	myBeep(false);
}

/*
 * Функция возвращает значение ШИМ для отборв по "шпоре" (температуре в кубе)
 */
int16_t GetSrPWM(void)
{


	int16_t found = ProcChimSR;
	double t = getCubeTemp();

	for (int i=1; i<cntCHIM; i++) {
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

// Обработка состояний в режиме ректификации
void Rectification(void)
{
	double t;
	float topen;
	char b[80];

	switch (MainStatus) {
	case START_WAIT:
		// Ожидание запуска процесса
		break;
	case PROC_START:
		// Начало процесса
		MainStatus = PROC_RAZGON;
		if (beepChangeState) myBeep(true);

	case PROC_RAZGON:
		// Разгон до рабочей температуры
		setPower(maxPower);	//  максимальная мощность для разгона
		if (tempEndRectRazgon > 0) t = getCubeTemp();
		else t = getTube20Temp();
		if (-1 == t) break;
		if (t < fabs(tempEndRectRazgon)) break;

		// Открытие клапана воды
		openKlp(klp_water);

		// Запоминаем температуру и время
		tempTube20Prev = getTube20Temp();
		secTempPrev = uptime_counter;
		MainStatus = PROC_STAB;
		if (beepChangeState) myBeep(true);

	case PROC_STAB:
		// Стабилизация температуры
		openKlp(klp_water);	// Открытие клапана воды (на всякий случай)

		setPower(powerRect);	// мощность ректификации
		t = getTube20Temp();
		if (-1 == t) break;
		if (t < 70) {
			// Колонна еще не прогрелась.
			break;
		}
		if (timeStabKolonna > 0) {
			// Время относительно последнего изменения температуры
			if (fabs(t - tempTube20Prev) < 0.2) {
				// Если текущая температура колонны равна температуре,
				// запомненной ранее в пределах погрешности в 0.2 градуса C
				if (uptime_counter-secTempPrev<timeStabKolonna) {
					// С момента последнего измерения прошло меньше заданого
					// времени.
					break;
				}
				// Если с момента последнего измерения прошло больше
				// cекунд чем нужно, считаем, что температура в колонне
				// стабилизировалась и переходим к отбору голов
				MainStatus = PROC_GLV;
				if (beepChangeState) myBeep(true);				
				secTempPrev = uptime_counter;
			} else {
				// Рассогласование температуры запоминаем
				// температуру и время
				tempTube20Prev = t;
				secTempPrev = uptime_counter;
				break;
			}
		} else {
			// Абсолютное значение времени
			if (uptime_counter-secTempPrev < fabs(timeStabKolonna)) {
				// Если с момента начала стабилизации прошло меньше
				// заданного количества секунд - продолжаем ждать
				break;
			}
			// Переходим к следующему этапу - отбору голов. 
			MainStatus = PROC_GLV;
			if (beepChangeState) myBeep(true);
			secTempPrev = uptime_counter;
		}

	case PROC_GLV:
		// Отбор головных фракций
		setPower(powerRect);	// мощность ректификации

		// Устанавливаем медленный ШИМ клапан отбора хвостов и голов в соответвии с установками
		topen = timeChimRectOtbGlv/100*procChimOtbGlv;
		startGlvKlp(topen, timeChimRectOtbGlv-topen);

		tempStabSR = getTube20Temp();	// температура, относительно которой будем стабилизировать отбор

		t = getCubeTemp();
		if (t < tEndRectOtbGlv) {
			// Пока температура в кубе не выросла выше окончания отбора голов, продолжаем это состояни
			break;       
		}

		closeKlp(klp_glwhq); 			// Отключение клапана отбора голов/хвостов
		MainStatus = PROC_T_WAIT;		// Переходим к стабилизации температуры
		ProcChimSR = beginProcChimOtbSR;	// Устанавливаем стартовый % отбора товарного продукта
		if (beepChangeState) myBeep(true);

	case PROC_T_WAIT:
		// Ожидание стабилизации температуры
		if (tempStabSR <= 0) tempStabSR =28.5;

		if (0 == timer_sec1 && timeRestabKolonna > 0) {
			// Если колонна слишком долго находится в режиме стопа,
			// то температуру стабилизации примем за новую
			tempStabSR = getTube20Temp();
			if (beepChangeState) myBeep(true);
		}
		closeKlp(klp_sr); 			// Отключение клапана продукта

		t = getCubeTemp();
		if (t >= tempEndRectOtbSR) {
			// Переходим к отбору хвостов
			MainStatus = PROC_HV;
			if (beepChangeState) myBeep(true);
		}
		
		t = getTube20Temp();
		if (t > tempStabSR) {
			break;
		}

		secTempPrev = uptime_counter;	// Запомним время, когда стабилизировалась температура
		MainStatus = PROC_SR;			// Переход к отбору продукта
		if (beepChangeState) myBeep(true);

	case PROC_SR:
		// Отбор СР
		closeKlp(klp_glwhq); 	// Отключение клапана отбора голов/хвостов
		setPower(powerRect);	// мощность ректификации

		t = getCubeTemp();

		// Реализуется отбор по-шпоре, что в функции прописано то и будет возвращено.
		ProcChimSR = GetSrPWM();
		// Устанавливаем медленный ШИМ клапана продукта
		topen = timeChimRectOtbSR*ProcChimSR/100;
		startSrKlp(topen, timeChimRectOtbSR-topen);

		if (t >= tempStabSR+tempDeltaRect) {
			// Температура превысила температуру стабилизации
			if (cntCHIM < 0) {
				// Запоминаем температуру, когда произошел стоп за вычетом 0.1 градуса.
				autoPWM[-cntCHIM].temp = t - 0.1;
				autoPWM[-cntCHIM].pwm = ProcChimSR;
				if (-cntCHIM < COUNT_PWM-1) cntCHIM--;
			}
			if (decrementCHIM>=0) { 
				// Тогда уменьшаем  ШИМ указанное число процентов в абсолютном выражении
				ProcChimSR -= decrementCHIM; 
			} else {
				uint16_t v = (ProcChimSR* (-decrementCHIM))/100;
				// Процентное отношение может быть очень мало,
				// поэтому если получилось нулевое значение, то вычтем единицу.
				if (v<=0) v=1;
				ProcChimSR -= v; // Тогда увеличиваем ШИМ на число процентов в относительном выражении
			}
			if (ProcChimSR < minProcChimOtbSR) ProcChimSR = minProcChimOtbSR;

			timer_sec1 = timeRestabKolonna; // Установка таймера стабилизации
			MainStatus = PROC_T_WAIT; // Переходим в режим стабилизации
			if (beepChangeState) myBeep(true);
		} else {
        		if (uptime_counter - secTempPrev > timeAutoIncCHIM) {
				// Если температура не выросла более, чем за 10 минут, прибавим ШИМ на 5%
				if (ProcChimSR > minProcChimOtbSR) {
					// Шим прибавляем только если не дошли до минимального
					if (incrementCHIM>=0) {
						// Абсолютное значение
						ProcChimSR += incrementCHIM;
					} else {
						// Проценты
						ProcChimSR += (ProcChimSR*(-incrementCHIM))/100;
					}
					if (ProcChimSR>95) ProcChimSR=95;

				}
				secTempPrev = uptime_counter;
			}
		}

		if (t < tempEndRectOtbSR) {
			break;
		}
		// Температура в кубе превысила температуру при которой надо отбирать СР
		MainStatus = PROC_HV;			// Переход к отбору хвостов
		closeKlp(klp_sr); 			// Отключение клапана продукта
		if (beepChangeState) myBeep(true);


	case PROC_HV:
		// Отбор хвостовых фракций
		setPower(powerRect);	// мощность ректификации

		topen = timeChimRectOtbGlv*90/100;
		startGlvKlp(topen, timeChimRectOtbGlv-topen);

		t = getCubeTemp();
		if (t < tempEndRect) {
			break;
		}
		// Температура достигла отметки окончания ректификации	
		secTempPrev = uptime_counter;
		MainStatus = PROC_WAITEND;			// Переход к окончанию процесса
		if (beepChangeState) myBeep(true);

	case PROC_WAITEND:
		// Отключение нагрева, подача воды для охлаждения
		setPower(0);		// Снятие мощности с тэна
		closeKlp(klp_glwhq); 	// Отключение клапана отбора голов/хвостов
		if (uptime_counter - secTempPrev > 180) {
			MainStatus = PROC_END;
			SecondsEnd = uptime_counter;
			if (beepChangeState) myBeep(true);
		}
		break;

	case PROC_END:
		// Окончание работы
		setPower(0);		// Снятие мощности с тэна
		closeAllKlp();		// Закрытие всех клапанов.
		sprintf(b, "Rectification complete, time: %02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
		sendSMS(b);

		break;
	}
}

// Обработка состояний в режиме дестилляции
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
		MainStatus = PROC_RAZGON;
		if (beepChangeState) myBeep(true);

	case PROC_RAZGON:
		// Разгон до рабочей температуры
		setPower(maxPower);	//  максимальная мощность для разгона
		t = getCubeTemp();
		if (-1 == t) break;
		if (t < tempEndRectRazgon) break;

		// Открытие клапана воды
		openKlp(klp_water);

		MainStatus = PROC_DISTILL;
		if (beepChangeState) myBeep(true);
		setPower(powerDistil);	// Мощность дестилляции

	case PROC_DISTILL:
		// Процесс дестилляции
		t = getCubeTemp();
		if (t < tempEndDistil) {
			break;
		}

		secTempPrev = uptime_counter;
		MainStatus = PROC_WAITEND;			// Переход к окончанию процесса
		if (beepChangeState) myBeep(true);

	case PROC_WAITEND:
		// Отключение нагрева, подача воды для охлаждения
		setPower(0);		// Снятие мощности с тэна
		if (uptime_counter - secTempPrev > 180) {
			MainStatus = PROC_END;
			SecondsEnd = uptime_counter;
			if (beepChangeState) myBeep(true);
		}

		break;

	case PROC_END:
		// Окончание работы
		setPower(0);		// Снятие мощности с тэна
		closeAllKlp();		// Закрытие всех клапанов.
		sprintf(b, "Distillation complete, time: %02d:%02d:%02d", uptime_counter/3600, (uptime_counter/60)%60, uptime_counter%60);
		sendSMS(b);
		break;

	}
}

/*
 * Закрытие всех клапанов.
 */
void closeAllKlp(void)
{
	for (int i=0; i<MAX_KLP; i++) {
		int ch = Klp[i].channel;
		ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, 0);
		ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
	        LEDC.channel_group[0].channel[ch].conf0.sig_out_en = 0;
		Klp[i].open_time = 0;
		Klp[i].is_open = false;
		Klp[i].is_pwm = false;
	}
}


/*
 * Открытие клапана воды
 */
void openKlp(int i)
{

	if (i>=MAX_KLP) return;
	int ch = Klp[i].channel;

	ESP_LOGI(TAG, "Open klp %d", i);
        LEDC.channel_group[0].channel[ch].conf0.sig_out_en = 1;
	if (klpSilentNode) {
		ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, ch, 1023, 1000);
		ledc_fade_start(LEDC_HIGH_SPEED_MODE, ch, LEDC_FADE_NO_WAIT);
	} else {

		ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, 1023);
		ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
	}
	Klp[i].is_pwm = false;
	Klp[i].is_open = true;
}

/*
 * Закрытие определенного клапана
 */
void closeKlp(int i)
{
	if (i>=MAX_KLP) return;
	ESP_LOGI(TAG, "Close klp %d", i);
	int ch = Klp[i].channel;
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, 0);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
        LEDC.channel_group[0].channel[ch].conf0.sig_out_en = 0;
	Klp[i].open_time = 0;
	Klp[i].timer_sec = 0;
	Klp[i].is_open = false;
	Klp[i].is_pwm = false;
}

/*
 * Запуск шима клапана
 */
void startKlpPwm(int i, float topen, float tclose)
{
	if (i>=MAX_KLP) return;
	ESP_LOGI(TAG, "PWM klp %d %f/%f", i, topen, tclose);
	Klp[i].open_time = topen;	// Время в течении которого клапан открыт
	Klp[i].close_time = tclose;	// Время в течении которого клапан закрыт
	Klp[i].timer_sec = 0;	// Начальное состояние - клапан закрыт
	Klp[i].is_open = false;
	Klp[i].is_pwm = true;	// Запускаем медленный Шим режим
}


// Запуск шима отбора голов
void startGlvKlp(float topen, float tclose)
{
	closeKlp(klp_sr);	// Отключение клапана отбора товарного продукта
	startKlpPwm(klp_glwhq, topen, tclose);
}

// Запуск шима отбора товарного продукта
void startSrKlp(float topen, float tclose)
{
	closeKlp(klp_glwhq); // Отключение клапана отбора голов/хвостов
	startKlpPwm(klp_sr, topen, tclose);
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
	ESP_ERROR_CHECK(uart_driver_install(CONFIG_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
	// Tell VFS to use UART driver
	esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);
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

	nvs_flash_init();
        ESP_LOGI(TAG, "RAM left %d", esp_get_free_heap_size());

	/* Настройка SPI */
	spi_setup();

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

	xTaskCreate(&console_task, "console_task", 8192, NULL, 1, NULL);

	/* Получаем конфигурацию сетевого доступа */
	get_network_config();

	/* Настройка wifi */
	wifi_setup();

	/* Чтение настроек */
	param_setup();

	/* Поиск и настройка датиков темпервтуры и периодический опрос их */
	xTaskCreate(&ds_task, "ds_task", 4096, NULL, 1, NULL);

	/* Настройка gpio детектора нуля сетевого напряжения */
	gpio_set_direction(GPIO_DETECT_ZERO, GPIO_MODE_INPUT);
	gpio_set_intr_type(GPIO_DETECT_ZERO, GPIO_INTR_NEGEDGE);
	gpio_set_pull_mode(GPIO_DETECT_ZERO, GPIO_PULLUP_ONLY);
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
	xTaskCreate(&display_task, "display_task", 2048, NULL, 1, NULL);

	/* Настройка PZEM */
	xTaskCreate(&pzem_task, "pzem_task", 2048, NULL, 1, NULL);

	ledc_timer_config_t ledc_timer = {
		.bit_num = LEDC_TIMER_10_BIT,
		.freq_hz = HZ*2,
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
		ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
		LEDC.channel_group[0].channel[i+1].conf0.sig_out_en = 0;
	}
	ledc_fade_func_install(0);


	ESP_ERROR_CHECK(gpio_intr_enable(GPIO_DETECT_ZERO));
	ESP_LOGI(TAG, "Enabled zero crossing interrupt.\n");
	if (beepChangeState) myBeep(false);


	while (true) {
		cJSON *ja = getInformation();
     		char *r=cJSON_Print(ja);
		cgiWebsockBroadcast("/ws", r, strlen(r), WEBSOCK_FLAG_NONE);
		cJSON_Delete(ja);
		if (r) free(r);
		if (MODE_RECTIFICATION == MainMode) Rectification();
		else if (MODE_DISTIL == MainMode) Distillation();
		vTaskDelay(5000/portTICK_PERIOD_MS);
	}
}

