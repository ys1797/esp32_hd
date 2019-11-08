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

extern char I2C_detect[128];		/* Список обнаруженных устройств на шине i2c */

// Режимы работы
typedef enum {
        MODE_IDLE=0,		// Режим мониторинга
        MODE_POWEERREG=1,	// Режим регулятора мощности
	MODE_DISTIL=2,		// Режим дистилляции
	MODE_RECTIFICATION=3,	// Режим ректификации
	MODE_TESTKLP=10,	// Режим тестирования клапанов
} main_mode;

// Аварийные состояния
typedef enum {
	NO_ALARM    =0x0,		// Нет аварии
	ALARM_TEMP  =0x2,		// Авария по превышению температуры
	ALARM_WATER =0x4,		// Авария: отсутствие воды охлаждения
	ALARM_FREQ  =0x8, 		// Авария: отсутствие напряжения сети
	ALARM_NOLOAD=0x10,		// Авария: отсутствие подключения нагрузки
} alarm_mode;

#define START_WAIT	0	// Ожидание запуска процесса
#define PROC_START	1	// Начало процесса
#define PROC_RAZGON	2	// Разгон до рабочей температуры
#define PROC_STAB	3	// Стабилизация температуры
#define PROC_GLV	4	// Отбор головных фракций
#define PROC_T_WAIT	5       // Ожидание стабилизации температуры
#define PROC_SR		6	// Отбор СР
#define PROC_DISTILL	PROC_SR	// Дистилляция
#define PROC_HV		7	// Отбор хвостовых фракций
#define PROC_WAITEND	8	// Отключение нагрева, подача воды для охлаждения
#define PROC_END	100	// Окончание работы


extern char *Hostname;		// Имя хоста
extern char *httpUser;		// Имя пользователя для http
extern char *httpPassword;	// Пароль для http
extern int httpSecure;		// Спрашивать пароль
extern int wsPeriod;		// Период обновления данных через websocket

extern main_mode MainMode;	// Текущий режим работы
extern alarm_mode AlarmMode;	// Состояние аварии
extern int16_t MainStatus;	// Текущее состояние (в зависимости от режима)
extern int16_t CurPower;	// Текущая измерянная мощность
extern int16_t SetPower;	// Установленная мощность
extern int16_t CurVolts;	// Текущее измеренное напряжение
extern volatile int16_t CurFreq;// Измерянное число периодов в секунду питающего напряжения
extern int16_t WaterOn;		// Флаг включения контура охлаждения
extern float TempWaterIn;	// Температура воды на входе в контур
extern float TempWaterOut;	// Температура воды на выходе из контура
extern int16_t WaterFlow;	// Значения датчика потока воды.







#define TICK_RATE_HZ 100
#define TICK_PERIOD_MS (1000 / TICK_RATE_HZ)
#define LED_HZ 50
// TRIAC is kept high for TRIAC_GATE_IMPULSE_CYCLES PWM counts before setting low.
#define TRIAC_GATE_IMPULSE_CYCLES 10
// TRIAC is always set low at least TRIAC_GATE_QUIESCE_CYCLES PWM counts 
// before the next zero crossing.
#define TRIAC_GATE_QUIESCE_CYCLES 10
#define TRIAC_GATE_MAX_CYCLES 55

#define HMAX (1<<LEDC_TIMER_10_BIT)-1


// Определение для работы с клапанами
typedef struct  {
	bool is_pwm;			// Флаг, означающий, что клапан  в режиме шим.
	float open_time;		// Время для открытия таймера в секундах (в режиме шим)
	float close_time;		// Время для закрытого состояния клапана в секундах (в режиме шим)
	float timer_sec;		// Отсчет секунд для текущего состояния таймера. (в режиме шим)
	bool is_open;			// Флаг, что клапан в открытом состоянии
	int channel;			// Канал ledc
} klp_list;

extern klp_list Klp[MAX_KLP];		// Список клапанов.
#define klp_water 0			// Клапан подачи воды.
#define klp_glwhq 1			// Клапан отбора голов и хвостов.
#define klp_sr    2			// Клапан отбора товарного спирта


void myBeep(bool lng);		// Включаем бипер


void PZEM_init(void);
bool PZEMv30_updateValues(void);
float PZEM_voltage(void);
float PZEM_current(void);
float PZEM_power(void);
float PZEM_energy(void);

const char *getMainModeStr(void);
const char *getAlarmModeStr(void);
const char *getMainStatusStr(void);
cJSON* getInformation(void);

void sendSMS(char *text);	// Отправка SMS
void Rectification(void);	// Обработка состояний в режиме ректификации
void setPower(int16_t pw);	// Установка рабочей мощности
void setMainMode(int new_mode);	// Установка нового режима работы
void setStatus(int next);	// Ручная установка состояния конечного автомата
void closeAllKlp(void);		// Закрытие всех клапанов.
void openKlp(int i);		// Открытие клапана воды
void closeKlp(int i);		// Закрытие определенного клапана
void startKlpPwm(int i, float topen, float tclose); // Запуск шима клапана
void startGlvKlp(float topen, float tclose); // Запуск шима отбора голов
void startSrKlp(float topen, float tclose);	// Запуск шима отбора товарного продукта


int hd_httpd_init(void);	// Запуск http сервера