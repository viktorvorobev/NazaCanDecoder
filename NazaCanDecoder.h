#ifndef NAZACANDECODER_H_
#define NAZACANDECODER_H_

#include <python3.5/Python.h>   // для работы с Python3
#include <linux/can.h>		// для работы с can
#include <linux/can/raw.h>
#include <sys/socket.h>		// для работы с сокетами
#include <sys/types.h>
#include <sys/ioctl.h>		// низкоуровневая работа с интерфейсами
#include <sys/un.h>
#include <netinet/in.h>
#include <net/if.h>
#include <ifaddrs.h>    // проверить существование сети

#include <iostream>
#include <cstdio>
#include <string>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>

#include <thread>
#include <chrono>
#include <cmath>
#include <queue>

// Получение сообщений от контроллера заряда батарей
//#define GET_SMART_BATTERY_DATA

// ID сообщений
#define NAZA_MESSAGE_COUNT		3
#define NAZA_MESSAGE_NONE		0x0000
#define NAZA_MESSAGE_MSG1002	0x1002
#define NAZA_MESSAGE_MSG1003	0x1003
#define NAZA_MESSAGE_MSG1009	0x1009
#ifdef GET_SMART_BATTERY_DATA
#define NAZA_MESSAGE_MSG0926	0x0926
#endif

//class NazaCanDecoder
//{
//public:
typedef enum {NO_FIX = 0, FIX_2D = 2, FIX_3D = 3, FIX_DGPS = 4} fixType_t;	// тип фиксации GPS
typedef enum {MANUAL = 0, GPS = 1, FAILSAFE = 2, ATTI = 3} flyMode_t;			// режим полета
typedef enum {MOTOR_M1 = 0, MOTOR_M2 = 1, MOTOR_M3 = 2, MOTOR_M4 = 3,
                MOTOR_M5 = 4, MOTOR_M6 = 5, MOTOR_F1 = 6, MOTOR_F2 = 7} motorOut_t;	// индексы выходных сигналов моторов
typedef enum {RC_UNUSED_1 = 0, RC_A = 1, RC_E = 2,
                RC_R = 3, RC_U = 4, RC_T = 5, RC_UNUSED_2 = 6,
                RC_X1 = 7, RC_X2 = 8, RC_UNUSED_3 = 9} rcInChan_t;			// индексы каналов управления
//	void NazaCanDecoder(const char* canBus);	// конструктор
int Begin(const char* canBus);		// запуск класса
int InitCanSocket(int *socket, const char* interface);	// создание CAN сокета
void ThreadDebug();
void ThreadHeartbeat();	// периодически (раз в 2 секунды) шлет heartbeat сообщение контроллеру
void ThreadCanRead();
std::queue<can_frame> inputMsg;    // очередь входящих сообщений
uint8_t debugCounter;
void ThreadParser();    // тред в котором будем парсить входящие сообщения
unsigned long GetDebugCounter();
int Stop();        // функция, останавливающая треды
uint16_t Decode();	// декодировать входящее CAN сообщение, если есть (должно вызываться в цикле)
bool stop = true;	// флаг, по которому будем завершать треды

// функции возвращающие значения
double GetLatitude();	// возвращает широту в градусах
double GetLongitude();	// возвращает долготу в градусах
double GetAltitude();	// возвращает высоту в метрах (от барометра)
double GetGpsAltitude();	// возвращает высоту в метрах (от GPS)
double GetSpeed();	// возвращает скорость в м/с
fixType_t GetFixType();	// возвращает тип фиксации
uint8_t GetNumSat();	// возвращает кол-во найденных спутников
double GetHeading();	// возвращает курс в градусах (c компенсацией наклона)
double GetHeadingNc();	// возвращает курс в градусах (без компенсации наклона)
double GetCog();		// возвращает курс над землей в градусах (относительно сторон света)
double GetVsi();		// возвращает скорость набора высоты в м/с (от барометра)
double GetVsiGps();	// возвращает скорость набора высоты в м/с (от GPS)
double GetHdop();	// возвращает горизонтальный DOP (dilution of precision)
double GetVdop();	// возрващает вертикальный DOP (dilution of precision)
int8_t GetPitch();	// возвращает угол рысканья в градусах
int16_t GetRoll();	// возвращает угол крена в градусах
uint8_t GetYear();	// возвращает год от GPS (последние 2 цифры)
uint8_t GetMonth();	// возвращает месяц от GPS
uint8_t GetDay();		// возвращает число от GPS
uint8_t GetHour();	// возвращает часы от GPS (для времени от 16:00 до 23:59 GPS возвращает 0:00 - 7:59)
uint8_t GetMinute();	// возвращает минуты от GPS
uint8_t GetSecond();	// возвращает секунды от GPS
uint16_t GetBattery();	// возвращает напряжение батарейки в мВ
uint16_t GetMotorOutput(motorOut_t mot);	// возвращает значение подаваемое на мотор (0 - не используется, иначе 16920~35000, 16920 = мотор выкл)
int16_t GetRcIn(rcInChan_t chan);		// возвращает значение получаемое от джойстика для каждого канала (-1000~1000)
flyMode_t GetMode();	// возвращает текущий режим работы

#ifdef GET_SMART_BATTERY_DATA
typedef enum {CELL_1 = 0, CELL_2 = 1, CELL_3 = 2, } smartBatteryCell_t;	// индексы банок контроллера заряда
uint8_t GetBatteryPercent();	// возвращает заряд батареи в процентах
uint16_t GetBatteryCell(smartBatteryCell_t cell);	// возвращает напряжение банки в мВ
#endif

//private:
// заголовок общий для всех сообщений
typedef struct __attribute__((packed))
{
    uint16_t id;
    uint16_t len;
} naza_msg_header_t;

// структуры сообщений
typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    double longitude;	// долгота (радианы)
    double latitude;	// широта (радианы)
    float altGps;		// высота по GPS (метры)
    float accX;			// показания акселерометра по оси X (неизв. формат)
    float accY;			// показания акселерометра по оси Y (неизв. формат)
    float accZ;			// показания акселерометра по оси Z (неизв. формат)
    float gyrX;			// показания гироскопа по оси X (неизв. формат)
    float gyrY;			// показания гироскопа по оси Y (неизв. формат)
    float gyrZ;			// показания гироскопа по оси Z (неизв. формат)
    float altBaro;		// высота по барометру (метры)
    float quaternion[4];	// кватернион ориентации в пространстве
    float unk1[3];			// неизвестные данные
    float northVelocity;	// средняя скорость движения на север (м/с), 0 если меньше 5 спутников
    float eastVelocity;		// средняя скорость движения на восток (м/с), 0 если меньше 5 спутников
    float downVelocity;		// средняя скорость движения вниз/вверх (м/с), по барометру
    float unk2[3];		// неизвестные данные
    int16_t magCalX;	// показания магнетометра по оси X
    int16_t magCalY;	// показания магнетометра по оси Y
    int16_t magCalZ;	// показания магнетометра по оси Z
    uint8_t unk3[10];	// неизвестные данные
    uint8_t numSat;		// кол-во подключенных спутников
    uint8_t unk4;		// неизвестные данные
    uint16_t seqNum;	// номер сообщения (увеличивается с каждым сообщением)
} naza_msg1002_t;

typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    uint32_t dateTime;		// дата и время
    uint32_t longitude;		// долгота*10^7 (градусы)
    uint32_t latitude;		// широта*10^7 (градусы)
    uint32_t altGps;		// высота по GPS (миллиметры)
    uint32_t hae;			// оценка горизонтальной точности (миллиметры) ???
    uint32_t vat;			// оценка вертикальной точности (миллиметры) ???
    uint8_t unk0[4];		// неизвестные данные
    int32_t northVelocity;	// скорость движения на север (см/с)
    int32_t eastVelocity;	// скорость движения на восток (см/с)
    int32_t downVelocity;	// скорость движения вниз (см/с)
    uint16_t pdop;		// position DOP (x100) *я не знаю как это переводить, извините :( что-то связанное с позиционированием группы объектов*
    uint16_t vdop;		// vertical DOP (подробнее см. uBlox NAV-DOP messages)
    uint16_t ndop;		// northing DOP (подробнее см. uBlox NAV-DOP messages)
    uint16_t edop;		// easting DOP (подробнее см. uBlox NAV-DOP messages)
    uint8_t numSat;		// кол-во подключенных спутников
    uint8_t unk1;		// неизвестные данные
    uint8_t fixType;	// тип фиксации (0 - без фиксации, 2 - 2D, 3 - 3D, скорее всего других вариантов нет,подробнее uBlox NAV-SOL messages)
    uint8_t unk2;		// неизвестные данные
    uint8_t fixStatus;	// статус фиксации (подробнее см. uBlox NAV-SOL messages)
    uint8_t unk[3];		// неизвестные данные
    uint16_t seqNum;	// номер сообщения (увеличивается с каждым сообщением)
} naza_msg1003_t;

typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    uint8_t unk1[4];		// неизвестные данные
    uint16_t motorOut[8];	// выходной сигнал для моторов (M1/M2/M3/M4/M5/M6/F1/F2)
    uint8_t unk2[4];		// неизвестные данные
    int16_t rcIn[10];		// входной сигнал пульта управления (порядок: неисп./A/E/R/U/T/неисп./X1/X2/неисп.)
    uint8_t unk3[11];		// неизвестные данные
    uint8_t flightMode;		// режим полета (0 - ручное, 1 - GPS, 2 - failsafe (отказоустойчивый?), 3 - atti (удержание высоты?))
    uint8_t unk4[8];		// неизвестные данные
    double homeLat;			// "домашняя" широта (радианы)
    double homeLon;			// "домашняя" долгота (радианы)
    float homeAltBaro;		// "домашняя" высота от барометра + 20 м (метры)
    uint16_t seqNum;		// номер сообщения
    uint8_t unk5[2];		// неизвестные данные
    float stabRollIn;		// входной сигнал стабилизатора положения по оси крена (-1000~1000)
    float stabPitchIn;		// входной сигнал стабилизатора положения по оси тангажа (-1000~1000)
    float stabThroIn;		// входной сигнал стабилизатора положения по тяге ? (-1000~1000)
    uint8_t unk6[4];		// неизвестные данные
    float actAileIn;		// actual aileron input, mode and arm state dependent (-1000~1000)
    float actElevIn;		// actual elevator input, mode and arm state dependent (-1000~1000)
    float actThroIn;		// actual throttle input, mode and arm state dependent (-1000~1000)
    uint16_t batVolt;		// напряжение главного (?) аккумулятора (милливольты)
    uint16_t becVolt;		// напряжение платы раздачи/преобразователя питания (?) (милливольты)
    uint8_t unk7[4];		// неизвестные данные
    uint8_t controlMode;	// режим управления (0 - GPS/failsafe, 1 - waypoint mode (?), 3 - ручной (manual), 6 - atti (удержание высоты?))
    uint8_t unk8[5];		// неизвестные данные
    int16_t gyrScalX;		// gyroscope scale X ???
    int16_t gyrScalY;		// gyroscope scale Y ???
    int16_t gyrScalZ;		// gyroscope scale Z ???
    uint8_t unk9[32];		// неизвестные данные
    float downVelocity;		// скорость набора высоты/спуска (м/с)
    float altBaro;			// высота по барометру (метры)
    float roll;				// угол крена (радианы)
    float pitch;			// угол тангажа (радианы)
} naza_msg1009_t;

#ifdef GET_SMART_BATTERY_DATA
typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    uint16_t designCapacity;	// расчетная емкость аккумулятора? (мАч)
    uint16_t fullCapacity;		// максимальная емкость аккумулятора (мАч)
    uint16_t currentCapacity;	// текущая емкость аккумулятора (мАч)
    uint16_t voltage;			// напряжение (мВ)
    int16_t current;			// ток (мА)
    uint8_t lifePercent;		// остаток заряда (%)
    uint8_t chargePercent;		// % до зарядки ??
    int16_t temperature;		// температура (град цельсия/10)
    uint16_t dischargeCount;	// кол-во разрядов аккумулятора ?
    uint16_t serialNumber;		// серийный номер
    uint16_t cellVoltage[3];	// напряжение каждой ячейки (банки) аккумулятора (мВ)
    uint8_t unk1[11];			// неизвестные данные
} naza_msg0926_t;
#endif

// структура содержащая все сообщения (с ее помощью будем парсить)
typedef union
{
    uint8_t 			bytes[256];	// максимальная длина посылки (184) + начальная последовательность (4) + завершающая последовательность (4)
    naza_msg_header_t 	header;
    naza_msg1002_t		msg1002;
    naza_msg1003_t		msg1003;
    naza_msg1009_t		msg1009;
//#ifdef GET_SMART_BATTERY_DATA
    naza_msg0926_t		msg0926;
#endif
} naza_msg_t;

int canSocket;
uint32_t heartBeatTime = 0;				// счетчик по которому будем посылать heartBeat
struct can_frame HEARTBEAT_1;		// сообщения ThreadHeartbeat
struct can_frame HEARTBEAT_2;
//	struct can_filter FILTER_MASK;	// маски для фильтрации входящих сообщений
//	struct can_filter FILTER_090;
//	struct can_filter FILTER_108;
//	struct can_filter FILTER_7F8;
uint8_t canMsgIdIdx = 0;	// индекс, которым будем разделять прилетающие сообщения
uint8_t canMsgByte = 0;	// номер байта в сообщении

naza_msg_t msgBuf[NAZA_MESSAGE_COUNT];	// буффер сообщений (сюда собираем сообщение перед тем как парсить)
uint16_t msgLen[NAZA_MESSAGE_COUNT];	// ожидаемая длина сообщения
uint16_t msgIdx[NAZA_MESSAGE_COUNT];	// текущий номер байта в сообщении
uint8_t header[NAZA_MESSAGE_COUNT];		// флаги, по которым следим за начальной послдеовательностью
uint8_t footer[NAZA_MESSAGE_COUNT];		// флаги, по которым следим за завершающей последовательностью
uint8_t collectData[NAZA_MESSAGE_COUNT];	// флаги, по которым следим надо ли собирать сообщение в буффер

// возвращаемые переменные
double longitude = 0;	// долгота (град)
double latitude = 0;	// широта (град)
double altitude = 0;	// высота по барометру (м)
double gpsAltitude = 0;	// высота по GPS (м)
double speed = 0;		// скорость (м/с)
fixType_t fix = NO_FIX;		// тип фиксации
uint8_t satellite = 0;	// кол-во спутников
double heading = 0;		// курс в градусах (с компенсацией наклона)
double headingNc = 0;	// курс в градусах (без компенсации наклона)
double cog = 0;			// "курс над землей" относительно сторон света
double vsi = 0;			// скорость набора высоты по барометру (м/с)
double hdop = 0;		// горизонтальный DOP
double vdop = 0;		// вертикальный DOP
double gpsVsi = 0;		// скорость набора высоты по GPS (м/с)
float pitchRad = 0;		// угол рысканья (радианы)
float rollRad = 0;		// угол крена (радианы)
int8_t pitch = 0;		// угол рысканья (градусы)
int16_t roll = 0;		// угол крена (радианы)
uint8_t year = 0;		// год (минус 2000)
uint8_t month = 0;		// месяц
uint8_t day = 0;		// число
uint8_t hour = 0;		// часы
uint8_t minute = 0;		// минуты
uint8_t second = 0;		// секунды
uint16_t battery = 0;	// напряжение аккумулятора (мВ)
uint16_t motorOut[8];	// выходное значение для моторов (0 - не используется, иначе 16920~35000, 16920 = мотор выкл)
int16_t rcIn[10];	// входное значение с пульта управления (-1000~1000)
flyMode_t mode = FAILSAFE;		// режим полета
#ifdef GET_SMART_BATTERY_DATA
uint8_t batteryPercent = 0;		// заряд аккумулятора (%)
uint16_t batteryCell[3];	// напряжение каждой ячейки (мВ)
#endif
//};

#endif /* NAZACANDECODER_H_ */
