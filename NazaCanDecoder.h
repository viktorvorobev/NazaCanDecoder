#ifndef NAZACANDECODER_H_
#define NAZACANDECODER_H_

#include <python3.6/Python.h>   // this one must be on top, because the latter one can reorder something
#include "python3.6/structmember.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/un.h>
#include <sys/uio.h>
#include <netinet/in.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <mutex>

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

// Flag to get data from battery controller
//#define GET_SMART_BATTERY_DATA

// message IDs
#define NAZA_MESSAGE_COUNT		3
#define NAZA_MESSAGE_NONE		0x0000
#define NAZA_MESSAGE_MSG1002	0x1002
#define NAZA_MESSAGE_MSG1003	0x1003
#define NAZA_MESSAGE_MSG1009	0x1009
#ifdef GET_SMART_BATTERY_DATA
#define NAZA_MESSAGE_MSG0926	0x0926
#endif

typedef enum {NO_FIX = 0, FIX_2D = 2, FIX_3D = 3, FIX_DGPS = 4} fixType_t;	// GPS fix types
typedef enum {MANUAL = 0, GPS = 1, FAILSAFE = 2, ATTI = 3} flyMode_t;   // flight modes
typedef enum {MOTOR_M1 = 0, MOTOR_M2 = 1, MOTOR_M3 = 2, MOTOR_M4 = 3,
                MOTOR_M5 = 4, MOTOR_M6 = 5, MOTOR_F1 = 6, MOTOR_F2 = 7} motorOut_t;	// motor input numbers
typedef enum {RC_UNUSED_1 = 0, RC_A = 1, RC_E = 2,
                RC_R = 3, RC_U = 4, RC_T = 5, RC_UNUSED_2 = 6,
                RC_X1 = 7, RC_X2 = 8, RC_UNUSED_3 = 9} rcInChan_t;			// rc channel numbers
// A - aileron E - elevator, R - rudder, T - throttle, U - , X1 - , X2 -

int Begin(const char* canBus);		// launch all threads
int InitCanSocket(int *socket, const char* interface);
void SendHeartbeat();	// send init message
void ThreadCanRead();
std::queue<can_frame> inputMsg;
void Parser(struct can_frame frame);
int Stop();
uint16_t Decode();
static volatile int stop = true;

// getters
double GetLatitude();	// degree
double GetLongitude();	// degree
double GetAltitude();	// barometric height (meters)
double GetGpsAltitude();	// gps height (meters)
double GetSpeed();	// meters per second
fixType_t GetFixType();
uint8_t GetNumSat();	// number of visible satellites
double GetHeading();	// heading with tilt compensation
double GetHeadingNc();	// heading with out tilt compensation
double GetCog();		// course over ground
double GetVsi();		// barometric vertical speed (meters per second)
double GetVsiGps();	// gps vertical speed (meters per second)
double GetHdop();	// horizontal dilution of precision
double GetVdop();	// vertical dilution of precision
int8_t GetPitch();	// degree
int16_t GetRoll();	// degree
uint8_t GetYear();	// last 2 numbers of it
uint8_t GetMonth();
uint8_t GetDay();
uint8_t GetHour();	// be aware of time zones
uint8_t GetMinute();
uint8_t GetSecond();
uint16_t GetBattery();	// millivolts
uint16_t GetMotorOutput(int mot);	// 0 - motor unused, otherwise 16920 (off) ~ 35000 (full speed)
int16_t GetRcIn(int chan);		// -1000 ~ 1000
flyMode_t GetMode();

#ifdef GET_SMART_BATTERY_DATA
typedef enum {CELL_1 = 0, CELL_2 = 1, CELL_3 = 2, } smartBatteryCell_t;
uint8_t GetBatteryPercent();
uint16_t GetBatteryCell(int cell);	// millivolts
#endif

// заголовок общий для всех сообщений
typedef struct __attribute__((packed))
{
    uint16_t id;
    uint16_t len;
} naza_msg_header_t;


typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    double longitude;	// radians
    double latitude;	// radians
    float altGps;		// meters
    float accX;			// acceleration X (unknown format)
    float accY;			// acceleration Y (unknown format)
    float accZ;			// acceleration Z (unknown format)
    float gyrX;			// gyroscope X (unknown format)
    float gyrY;			// gyroscope Y (unknown format)
    float gyrZ;			// gyroscope Z (unknown format)
    float altBaro;		// barometric altitude
    float quaternion[4];
    float unk1[3];			// unknown data
    float northVelocity;	// northing velocity (m/s), 0 if less than 5 satellites seen
    float eastVelocity;		// easting velocity (m/s), 0 if less than 5 satellites seen
    float downVelocity;		// barometric vertical velocity (m/s)
    float unk2[3];		// unknown data
    int16_t magCalX;	// magnetometer X
    int16_t magCalY;	// magnetometer Y
    int16_t magCalZ;	// magnetometer Z
    uint8_t unk3[10];	// unknown data
    uint8_t numSat;		// visible satellites
    uint8_t unk4;		// unknown data
    uint16_t seqNum;	// message count (increased with every new message)
} naza_msg1002_t;

typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    uint32_t dateTime;
    uint32_t longitude;		// *10^7 (degree)
    uint32_t latitude;		// *10^7 (degree)
    uint32_t altGps;		// millimeters
    uint32_t hae;			// horizontal accuracy estimate (millimeters, see uBlox NAV-POSLLH message for details)
    uint32_t vat;			// vertical accuracy estimate (millimeters, see uBlox NAV-POSLLH message for details)
    uint8_t unk0[4];		// unknown data
    int32_t northVelocity;	// northing velocity (cm/s)
    int32_t eastVelocity;	// easting velocity (cm/s)
    int32_t downVelocity;	// vertical velocity (cm/s)
    uint16_t pdop;		// position DOP (x100)
    uint16_t vdop;		// vertical DOP (see uBlox NAV-DOP message for details)
    uint16_t ndop;		// northing DOP (see uBlox NAV-DOP message for details)
    uint16_t edop;		// easting DOP (see uBlox NAV-DOP message for details)
    uint8_t numSat;		// number of visible satellites
    uint8_t unk1;		// unknown data
    uint8_t fixType;	// (0 - no lock, 2 - 2D lock, 3 - 3D lock, not sure if other values can be expected - see uBlox NAV-SOL message for details)
    uint8_t unk2;		// unknown data
    uint8_t fixStatus;	// fix status flags (see uBlox NAV-SOL message for details)
    uint8_t unk[3];		// unknown data
    uint16_t seqNum;	// sequence number (not XORed), once there is a lock - increases with every message. When the lock is lost later LSB and MSB are swapped with every message.
} naza_msg1003_t;

typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    uint8_t unk1[4];		// unknown data
    uint16_t motorOut[8];	// motor output (M1/M2/M3/M4/M5/M6/F1/F2)
    uint8_t unk2[4];		// unknown data
    int16_t rcIn[10];		// RC controller input (unused/A/E/R/U/T/unused/X1/X2/unused)
    uint8_t unk3[11];		// unknown data
    uint8_t flightMode;		// 0 - manual, 1 - GPS, 2 - failsafe, 3 - atti (maintain altitude))
    uint8_t unk4[8];		// unknown data
    double homeLat;			// home latitude (radians)
    double homeLon;			// home longitude (radians)
    float homeAltBaro;		// home barometric altitude + 20 m (meters)
    uint16_t seqNum;		// sequence number, increases with every message
    uint8_t unk5[2];		// unknown data
    float stabRollIn;		// attitude stabilizer roll input to the system, merged with aileron input, flight mode dependent (float, -1000 ~ 1000 range)
    float stabPitchIn;		// attitude stabilizer pitch input the system, merged elevator input, flight mode dependent (float, -1000~1000 range)
    float stabThroIn;		// altitude stabilizer throttle input to the system based on flight mode and arm state (float, about -1000~1000 range)
    uint8_t unk6[4];		// unknown data
    float actAileIn;		// actual aileron feed to the system based on mode and arm state (float, about -1000~1000 range)
    float actElevIn;		// actual elevator feed to the system based on flight mode and arm state (float, about -1000~1000 range)
    float actThroIn;		// actual throttle feed to the system based on flight mode (float, about -1000~1000 range, changes to -200 in failsafe)
    uint16_t batVolt;		// battery voltage (millivolts)
    uint16_t becVolt;		// BEC voltage (millivolts)
    uint8_t unk7[4];		// unknown data
    uint8_t controlMode;	// control mode (0 - GPS/failsafe, 1 - waypoint mode?, 3 - manual, 6 - atti)
    uint8_t unk8[5];		// unknown data
    int16_t gyrScalX;		// gyroscope scale X ???
    int16_t gyrScalY;		// gyroscope scale Y ???
    int16_t gyrScalZ;		// gyroscope scale Z ???
    uint8_t unk9[32];		// неизвестные данные
    float downVelocity;		// downward velocity? (m/s)
    float altBaro;			// altitude from the barometric sensor (meters)
    float roll;				// radians
    float pitch;			// radians
} naza_msg1009_t;

#ifdef GET_SMART_BATTERY_DATA
typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    uint16_t designCapacity;	// mAh
    uint16_t fullCapacity;		// mAh
    uint16_t currentCapacity;	// mAh
    uint16_t voltage;			// mV
    int16_t current;			// mA
    uint8_t lifePercent;		// percentage of life
    uint8_t chargePercent;		// percentage of charge
    int16_t temperature;		// temperature (degrees Celsius/10)
    uint16_t dischargeCount;	// discharging times
    uint16_t serialNumber;		// serial number
    uint16_t cellVoltage[3];	// mV
    uint8_t unk1[11];			// unknown data
} naza_msg0926_t;
#endif

// this will help us parse everything
typedef union
{
    uint8_t 			bytes[256];	// max length (184) + header (4) + footer (4)
    naza_msg_header_t 	header;
    naza_msg1002_t		msg1002;
    naza_msg1003_t		msg1003;
    naza_msg1009_t		msg1009;
#ifdef GET_SMART_BATTERY_DATA
    naza_msg0926_t		msg0926;
#endif
} naza_msg_t;

int canSocket;

uint32_t heartBeatTime = 0;
struct can_frame HEARTBEAT_1;
struct can_frame HEARTBEAT_2;
//	struct can_filter FILTER_MASK;
//	struct can_filter FILTER_090;
//	struct can_filter FILTER_108;
//	struct can_filter FILTER_7F8;
uint8_t canMsgIdIdx = 0;
uint8_t canMsgByte = 0;

naza_msg_t msgBuf[NAZA_MESSAGE_COUNT];	// buffer for unparsed messages
uint16_t msgLen[NAZA_MESSAGE_COUNT];	// expected msg len
uint16_t msgIdx[NAZA_MESSAGE_COUNT];	// current index of byte in message
uint8_t header[NAZA_MESSAGE_COUNT];		// header flags
uint8_t footer[NAZA_MESSAGE_COUNT];		// footer flags
uint8_t collectData[NAZA_MESSAGE_COUNT];	// collect flags

// return values
double longitude = 0;	// degree
double latitude = 0;	// degree
double altitude = 0;	// barometric (meters)
double gpsAltitude = 0;	// gps (meters)
double speed = 0;		// m/s
fixType_t fix = NO_FIX;		// fix type
uint8_t satellite = 0;	// visible satellites
double heading = 0;		// heading with tilt compensation
double headingNc = 0;	// heading without tilt compensation
double cog = 0;			// course over ground
double vsi = 0;			// barometric vertical speed (m/s)
double hdop = 0;		// horizontal DOP
double vdop = 0;		// vertical DOP
double gpsVsi = 0;		// GPS vertical speed (m/s)
float pitchRad = 0;		// pitch (radians)
float rollRad = 0;		// roll (radians)
int8_t pitch = 0;		// pitch (degree)
int16_t roll = 0;		// roll (degree)
uint8_t year = 0;		// last 2 numbers
uint8_t month = 0;
uint8_t day = 0;
uint8_t hour = 0;
uint8_t minute = 0;
uint8_t second = 0;
uint16_t battery = 0;	// mV
uint16_t motorOut[8];	// motor outputs (0 - unused, 16920 ~ 35000)
int16_t rcIn[10];	// RC inputs (-1000~1000)
flyMode_t mode = FAILSAFE;
#ifdef GET_SMART_BATTERY_DATA
uint8_t batteryPercent = 0;
uint16_t batteryCell[3]; // mV
#endif

#endif /* NAZACANDECODER_H_ */
