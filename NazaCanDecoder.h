/*
 * NazaCanDecoder.h
 *
 *  Created on: 6 февр. 2018 г.
 *      Author: vikey
 */

#ifndef NAZACANDECODER_H_
#define NAZACANDECODER_H_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

// Получение сообщений от контроллера заряда батарей
#define GET_SMART_BATTERY_DATA

// ID сообщений
#define NAZA_MESSAGE_COUNT		3
#define NAZA_MESSAGE_NONE		0x0000
#define NAZA_MESSAGE_MSG1002	0x1002
#define NAZA_MESSAGE_MSG1003	0x1003
#define NAZA_MESSAGE_MSG1009	0x1009
#ifdef GET_SMART_BATTERY_DATA
#define NAZA_MESSAGE_MSG0926	0x0926
#endif

class NazaCanDecoder
{
public:
	typedef enum {NO_FIX = 0, FIX_2D = 2, FIX_3D = 3, FIX_DGPS = 4} fixType_t;	// тип фиксации GPS
	typedef enum {MANUAL = 0, GPS = 1, FAILSAFE = 2, ATTI = 3} mode_t;			// режим полета
	typedef enum {MOTOR_M1 = 0, MOTOR_M2 = 1, MOTOR_M3 = 2, MOTOR_M4 = 3,
				  MOTOR_M5 = 4, MOTOR_M6 = 5, MOTOR_F1 = 6, MOTOR_F2 = 7} motorOut_t;	// индексы выходных сигналов моторов
	typedef enum {RC_UNUSED_1 = 0, RC_A = 1, RC_E = 2,
				  RC_R = 3, RC_U = 4, RC_T = 5, RC_UNUSED_2 = 6,
				  RC_X1 = 7, RC_X2 = 8, RC_UNUSED_3 = 9} rcInChan_t;			// индексы каналов управления
	NazaCanDecoder();	// конструктор
	void Begin();		// запуск класса
	uint16_t Decode();	// декодировать входящее CAN сообщение, если есть (должно вызываться в цикле)
	void Heartbeat();	// периодически (раз в 2 секунды) шлет heartbeat сообщение контроллеру
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
	uint8_tGetMinute();	// возвращает минуты от GPS
	uint8_t GetSecond();	// возвращает секунды от GPS
	uint16_t GetBattery();	// возвращает напряжение батарейки в мВ
	uint16_t GetMotorOutput(motorOut_t mot);	// возвращает значение подаваемое на мотор (0 - не используется, иначе 16920~35000, 16920 = мотор выкл)
	int16_t GetRcIn(rcInChan_t chan);		// возвращает значение получаемое от джойстика для каждого канала (-1000~1000)
	mode_t GetMode();	// возвращает текущий режим работы

#ifdef GET_SMART_BATTERY_DATA
	typedef enum {CELL_1 = 0, CELL_2 = 1, CELL_3 = 2, } smartBatteryCell_t;	// индексы банок контроллера заряда
	uint8_t GetBatteryPercent();	// возвращает заряд батареи в процентах
	uint16_t GetBatteryCell(smartBatteryCell_t cell);	// возвращает напряжение банки в мВ
#endif

private:
	typedef struct __attribute__((packed))
	{
		uint16_t id;
		uint16_t len;
	} naza_msg_header_t;

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
		uint16_t currentCapacity;
		uint16_t voltage;
		int16_t current;
		uint8_t lifePercent;
		uint8_t chargePercent;
		int16_t temperature;
		uint16_t dischargeCount;
		uint16_t serialNumber;
		uint16_t cellVoltage[3];
		uint8_t unk1[11];
	} naza_msg0926_t;
#endif
};


#endif /* NAZACANDECODER_H_ */
