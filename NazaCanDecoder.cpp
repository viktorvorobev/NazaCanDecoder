#include "NazaCanDecoder.h"

NazaCanDecoder::NazaCanDecoder(const char* canBus)	// конструктор класса
{
	// создаем HeartBeat сообщения
	HEARTBEAT_1.can_id = 0x108;
	HEARTBEAT_1.can_dlc = 8;
	HEARTBEAT_1.data[8] = {0x55, 0xAA, 0x55, 0xAA, 0x07, 0x10, 0x00, 0x00};
	HEARTBEAT_2.can_id = 0x108;
	HEARTBEAT_2.can_dlc = 4;
	HEARTBEAT_2.data[4] = {0x66, 0xCC, 0x66, 0xCC};
	// создаем маски
//	FILTER_MASK.can_id = 0x7FF;
//	FILTER_090.can_id = 0x090;
//	FILTER_108.can_id = 0x108;
//	FILTER_7F8.can_id = 0x7F8;

	InitCanSocket(&canSocket, canBus);	// инициализируем CAN
}

int NazaCanDecoder::InitCanSocket(int *sock, const char* interface)	// инициализация CAN
{
	int s;
	struct sockaddr_can addr;
	struct ifreq ifr;
	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(s < 0)
	{
		perror("Error while opening socket");
		return -1;
	}
	strcpy(ifr.ifr_name, interface);
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		perror("Error in socket bind");
		return -1;
	}
	(*sock) = s;
	return 0;
}

// функции, возвращающие значения
double NazaCanDecoder::GetLatitude() {return latitude;}	// возвращает широту в градусах
double NazaCanDecoder::GetLongitude() {return longitude;}	// возвращает долготу в градусах
double NazaCanDecoder::GetAltitude() {return altitude;}	// возвращает высоту в метрах (от барометра)
double NazaCanDecoder::GetGpsAltitude() {return gpsAltitude;}	// возвращает высоту в метрах (от GPS)
double NazaCanDecoder::GetSpeed() {return speed;}	// возвращает скорость в м/с
NazaCanDecoder::fixType_t NazaCanDecoder::GetFixType() {return fix;}	// возвращает тип фиксации
uint8_t NazaCanDecoder::GetNumSat() {return satellite;}	// возвращает кол-во найденных спутников
double NazaCanDecoder::GetHeading() {return heading;}	// возвращает курс в градусах (c компенсацией наклона)
double NazaCanDecoder::GetHeadingNc() {return headingNc;}	// возвращает курс в градусах (без компенсации наклона)
double NazaCanDecoder::GetCog() {return cog;}		// возвращает курс над землей в градусах (относительно сторон света)
double NazaCanDecoder::GetVsi() {return vsi;}		// возвращает скорость набора высоты в м/с (от барометра)
double NazaCanDecoder::GetVsiGps() {return gpsVsi;}	// возвращает скорость набора высоты в м/с (от GPS)
double NazaCanDecoder::GetHdop() {return hdop;}	// возвращает горизонтальный DOP (dilution of precision)
double NazaCanDecoder::GetVdop() {return vdop;}	// возрващает вертикальный DOP (dilution of precision)
int8_t NazaCanDecoder::GetPitch() {return pitch;}	// возвращает угол рысканья в градусах
int16_t NazaCanDecoder::GetRoll() {return roll;}	// возвращает угол крена в градусах
uint8_t NazaCanDecoder::GetYear() {return year;}	// возвращает год от GPS (последние 2 цифры)
uint8_t NazaCanDecoder::GetMonth() {return month;}	// возвращает месяц от GPS
uint8_t NazaCanDecoder::GetDay() {return day;}		// возвращает число от GPS
uint8_t NazaCanDecoder::GetHour() {return hour;}	// возвращает часы от GPS (для времени от 16:00 до 23:59 GPS возвращает 0:00 - 7:59)
uint8_t NazaCanDecoder::GetMinute() {return minute;}	// возвращает минуты от GPS
uint8_t NazaCanDecoder::GetSecond() {return second;}	// возвращает секунды от GPS
uint16_t NazaCanDecoder::GetBattery() {return battery;}	// возвращает напряжение батарейки в мВ
uint16_t NazaCanDecoder::GetMotorOutput(motorOut_t mot) {return motorOut[mot];}	// возвращает значение подаваемое на мотор (0 - не используется, иначе 16920~35000, 16920 = мотор выкл)
int16_t NazaCanDecoder::GetRcIn(rcInChan_t chan) {return rcIn[chan];}		// возвращает значение получаемое от джойстика для каждого канала (-1000~1000)
NazaCanDecoder::mode_t NazaCanDecoder::GetMode() {return mode;}	// возвращает текущий режим работы
#ifdef GET_SMART_BATTERY_DATA
uint8_t NazaCanDecoder::GetBatteryPercent(){return batteryPercent;}	// возвращает заряд батареи в процентах
uint16_t NazaCanDecoder::GetBatteryCell(smartBatteryCell_t cell) {return batteryCell[cell];}	// возвращает напряжение банки в мВ
#endif
