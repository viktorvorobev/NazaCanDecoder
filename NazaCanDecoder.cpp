#include "NazaCanDecoder.h"

/*
 * Работа модуля.
 * Модуль работает в нескольких потоках (тредах):
 * 1) Отправка HeartBeat в CAN сеть.
 * 2) Получение сообщений из CAN сети, их парсинг.
 */

int InitCanSocket(int *sock, const char* interface) {   // инициализация CAN
    int s, n;
    struct sockaddr_can addr{};
    struct ifreq ifr{};
    bool noSuchInterface = true;    // флаг, что интерфейс вообще есть
    struct ifaddrs *ifaddr, *ifa;

    if(getifaddrs(&ifaddr) == -1) {    // получаем список всех интерфейсов
        perror("ERROR: Failed to get ifaddrs.");
        return -1;
    }
    for(ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++)    // ищем нужный интерфейс
        // если нашли нужный интерфейс, и он "поднят", меняем флаг - разрешаем дальнейшую работу
        if((!strcmp(interface, ifa->ifa_name)) && ifa->ifa_flags & IFF_UP)   // (strcmp: полное совпадение == 0)
            noSuchInterface = false;

    if(noSuchInterface) {  // Ошибка что интерфейс не найден
        perror("ERROR: Interface not found or down.");
        return -2;
    }

    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {   // открываем порт
        perror("ERROR: Failed to open port.");
        return -3;
    }
    strcpy(ifr.ifr_name, interface);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        perror("ERROR: Failed to bind socket.");
        return -4;
    }

    (*sock) = s;
    printf("Socket inited\n");
    return 0;
}

void ThreadHeartbeat() {    // тред в котором отправляем HeartBeat метку в сеть
    printf("Heartbeat started\n");
    while(!stop) {
        struct can_frame frame{};
        frame = HEARTBEAT_1;
        write(canSocket, &frame, sizeof(struct can_frame));
        frame = HEARTBEAT_2;
        write(canSocket, &frame, sizeof(struct can_frame));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    printf("Heartbeat stopped\n");
}

void ThreadCanRead() {  // тред в котором ловим посылки из CAN сети и помещаем их в FIFO буффер
    printf("Can reading thread started\n");
    while(!stop)
    {
        struct can_frame frame{};
        read(canSocket, &frame, sizeof(struct can_frame));  // читаем
        inputMsg.push(frame);   // помещаем в очередь
    }
    printf("Can reading stopped\n");
}

void ThreadParser() {   // тред в котором собираем сообщение в буффер и парсим его
    printf("Parser started \n");
    while(!stop) {
        uint16_t msgId = NAZA_MESSAGE_NONE;
        if(inputMsg.empty()) continue; // если очередь пустая - пропускаем цикл
        struct can_frame canMsg = inputMsg.front();  // забираем первый элемент
        inputMsg.pop(); // удаляем его из очереди

        if(canMsg.can_id == 0x090) canMsgIdIdx = 0;  // смотрим от кого пришло сообщение
        else if(canMsg.can_id == 0x108) canMsgIdIdx = 1;
        else if(canMsg.can_id == 0x7F8) canMsgIdIdx = 2;
        else continue;  // если сообщение прилетело от кого-то еще - пропускаем его

        for(uint8_t i = 0; i < canMsg.can_dlc; i++) {   // смотрим на каждый байт сообщения
            canMsgByte = canMsg.data[i];
            if(collectData[canMsgIdIdx] == 1) {
                msgBuf[canMsgIdIdx].bytes[msgIdx[canMsgIdIdx]] = canMsgByte;  // записываем сообщение в буффер
                if(msgIdx[canMsgIdIdx] == 3)    // если получили 4 байта - получаем длину сообщения
                    msgLen[canMsgIdIdx] = msgBuf[canMsgIdIdx].header.len;
                msgIdx[canMsgIdIdx] += 1;   // увеличиваем счетчик (на каком байте мы сейчас)
                // если собрали все байты сообщения - прекращаем собирать
                // (+8 тк: 2 - Id сообщения, 2 - Длина сообщения, 4 - завершающая последовательность)
                if((msgIdx[canMsgIdIdx] > (msgLen[canMsgIdIdx] + 8)) || (msgIdx[canMsgIdIdx] > 256))
                    collectData[canMsgIdIdx] = 0;
            }

            // ищем начальную последовательность (0x55 0xAA 0x55 0xAA)
            if(canMsgByte == 0x55) {
                if(header[canMsgIdIdx] == 0)    // пользуемся счетчиком, чтобы понять какой это байт последовательности
                    header[canMsgIdIdx] = 1;
                else if(header[canMsgIdIdx] == 2)
                    header[canMsgIdIdx] = 3;
                else header[canMsgIdIdx] = 0;   // если упустили последовательность - начинаем заново
            }
            else if(canMsgByte == 0xAA) {
                if(header[canMsgIdIdx] == 1)
                    header[canMsgIdIdx] = 2;
                else if(header[canMsgIdIdx] == 3) { // как только последовательность найдена
                    header[canMsgIdIdx] = 0;    // сбрасываем счетчик
                    collectData[canMsgIdIdx] = 1;   // поднимаем флаг, чтобы набирать буффер
                    msgIdx[canMsgIdIdx] = 0;    // обнуляем счетчик байт в сообщении
                }
                else header[canMsgIdIdx] = 0;   // если упустили последовательность - начинаем заново
            }
            else header[canMsgIdIdx] = 0;

            // ищем завершающую последовательность (0x66 0xCC 0x66 0xCC)
            if(canMsgByte == 0x66) {    // аналогично начальной
                if(footer[canMsgIdIdx] == 0)
                    footer[canMsgIdIdx] = 1;
                else if(footer[canMsgIdIdx] == 2)
                    footer[canMsgIdIdx] = 3;
                else footer[canMsgIdIdx] = 0;
            }
            else if(canMsgByte == 0xCC) {
                if(footer[canMsgIdIdx] == 1)
                    footer[canMsgIdIdx] = 2;
                else if(footer[canMsgIdIdx] == 3) {
                    footer[canMsgIdIdx] = 0;
                    if(collectData[canMsgIdIdx] != 0)   // как только нашли - прекращаем собирать буффер сообщений
                        collectData[canMsgIdIdx] = 2;   // парсим сообщение
                }
                else footer[canMsgIdIdx] = 0;
            }
            else footer[canMsgIdIdx] = 0;

            // парсинг сообщений
            if(collectData[canMsgIdIdx] == 2) {
                if(msgIdx[canMsgIdIdx] == (msgLen[canMsgIdIdx] + 8)) {  // проверяем, что сообщение пришло целиком
                    if(msgBuf[canMsgIdIdx].header.id == NAZA_MESSAGE_MSG1002) { // парсим в зависимости от ID
                        float magCalX = msgBuf[canMsgIdIdx].msg1002.magCalX;
                        float magCalY = msgBuf[canMsgIdIdx].msg1002.magCalY;
                        headingNc = -atan2(magCalY, magCalX)/M_PI * 180.0;
                        if(headingNc < 0) headingNc += 360;
                        float q0 = msgBuf[canMsgIdIdx].msg1002.quaternion[0];
                        float q1 = msgBuf[canMsgIdIdx].msg1002.quaternion[1];
                        float q2 = msgBuf[canMsgIdIdx].msg1002.quaternion[2];
                        float q3 = msgBuf[canMsgIdIdx].msg1002.quaternion[3];
                        heading = atan2(2.0*(q3*q0 + q1*q2), -1.0 + 2.0*(q0*q0 + q1*q1)) / M_PI*180.0;
                        if(heading < 0) heading += 360.0;
                        satellite = msgBuf[canMsgIdIdx].msg1002.numSat;
                        gpsAltitude = msgBuf[canMsgIdIdx].msg1002.altGps;
                        latitude = msgBuf[canMsgIdIdx].msg1002.latitude / M_PI*180.0;
                        longitude = msgBuf[canMsgIdIdx].msg1002.longitude / M_PI*180.0;
                        altitude = msgBuf[canMsgIdIdx].msg1002.altBaro;
                        float nVel = msgBuf[canMsgIdIdx].msg1002.northVelocity;
                        float eVel = msgBuf[canMsgIdIdx].msg1002.eastVelocity;
                        speed = sqrt(nVel*nVel + eVel*eVel);
                        cog = atan2(eVel, nVel) / M_PI*180.0;
                        if(cog < 0) cog += 360;
                        vsi = -msgBuf[canMsgIdIdx].msg1002.downVelocity;
                        msgId = NAZA_MESSAGE_MSG1002;
                    }
                    else if(msgBuf[canMsgIdIdx].header.id == NAZA_MESSAGE_MSG1003) {
                        uint32_t dateTime = msgBuf[canMsgIdIdx].msg1003.dateTime;
                        second = dateTime & 0b00111111; dateTime >>= 6;
                        minute = dateTime & 0b00111111; dateTime >>= 6;
                        hour = dateTime & 0b00001111; dateTime >>= 4;
                        day = dateTime & 0b00011111; dateTime >>= 5;
                        if(hour > 7) day++; //TODO: разобраться с часовыми поясами
                        month = dateTime & 0b00001111; dateTime >>= 4;
                        year = dateTime & 0b01111111;
                        gpsVsi = -msgBuf[canMsgIdIdx].msg1003.downVelocity;
                        vdop = (double)msgBuf[canMsgIdIdx].msg1003.vdop / 100;
                        double ndop = (double)msgBuf[canMsgIdIdx].msg1003.ndop / 100;
                        double edop = (double)msgBuf[canMsgIdIdx].msg1003.edop / 100;
                        hdop = sqrt(ndop*ndop + edop*edop);
                        uint8_t fixType = msgBuf[canMsgIdIdx].msg1003.fixType;
                        uint8_t fixFlags = msgBuf[canMsgIdIdx].msg1003.fixStatus;
                        switch(fixType) {
                            case 2 : fix = FIX_2D; break;
                            case 3 : fix = FIX_3D; break;
                            default: fix = NO_FIX; break;
                        }
                        if((fix != NO_FIX) && (fixFlags & 0x02)) fix = FIX_DGPS;
                        msgId = NAZA_MESSAGE_MSG1003;
                    }
                    else if(msgBuf[canMsgIdIdx].header.id == NAZA_MESSAGE_MSG1009) {
                        for(uint8_t j = 0; j < 8; j++)
                            motorOut[j] = msgBuf[canMsgIdIdx].msg1009.motorOut[j];
                        for(uint8_t j = 0; j < 10; j++)
                            rcIn[j] = msgBuf[canMsgIdIdx].msg1009.rcIn[j];
#ifndef GET_SMART_BATTERY_DATA
                        battery = msgBuf[canMsgIdIdx].msg1009.batVolt;
#endif
                        rollRad = msgBuf[canMsgIdIdx].msg1009.roll;
                        pitchRad = msgBuf[canMsgIdIdx].msg1009.pitch;
                        roll = (int16_t)(rollRad*180.0 / M_PI);
                        pitch = (int16_t)(pitchRad*180.0 / M_PI);
                        mode = (flyMode_t)msgBuf[canMsgIdIdx].msg1009.flightMode;
                        msgId = NAZA_MESSAGE_MSG1009;
                    }
#ifdef GET_SMART_BATTERY_DATA
                    else if(msgBuf[canMsgIdIdx].header.id == NAZA_MESSAGE_MSG0926){
                        battery = msgBuf[canMsgIdIdx].msg0926.voltage;
                        batteryPercent = msgBuf[canMsgIdIdx].msg0926.chargePercent;
                        for(uint8_t j = 0; j < 3; j++)
                            batteryCell[j] = msgBuf[canMsgIdIdx].msg0926.cellVoltage[j];
                        msgId = NAZA_MESSAGE_MSG0926;
                    }
#endif
                }
                collectData[canMsgIdIdx] = 0;
            }
        }
    }
    printf("Parser stopped\n");
}

int Begin(const char* canBus) {
    // создаем HeartBeat сообщения
    HEARTBEAT_1.can_id = 0x108;
    HEARTBEAT_1.data[0] = 0x55;
    HEARTBEAT_1.data[1] = 0xAA;
    HEARTBEAT_1.data[2] = 0x55;
    HEARTBEAT_1.data[3] = 0xAA;
    HEARTBEAT_1.data[4] = 0x07;
    HEARTBEAT_1.data[5] = 0x10;
    HEARTBEAT_1.data[6] = 0x00;
    HEARTBEAT_1.data[7] = 0x00;
    HEARTBEAT_1.can_dlc = 8;

    HEARTBEAT_2.can_id = 0x108;
    HEARTBEAT_2.data[0] = 0x66;
    HEARTBEAT_2.data[1] = 0xCC;
    HEARTBEAT_2.data[2] = 0x66;
    HEARTBEAT_2.data[3] = 0xCC;
    HEARTBEAT_2.can_dlc = 4;

    // создаем маски
//    FILTER_MASK.can_id = 0x7FF;
//    FILTER_090.can_id = 0x090;
//    FILTER_108.can_id = 0x108;
//    FILTER_7F8.can_id = 0x7F8;
    printf("Initialising CAN socket: \"%s\"...\n", canBus);
    if(InitCanSocket(&canSocket, canBus) != 0) {    // инициализируем CAN
        perror("ERROR: Failed to initialize can bus");
        return -1;
    }
    stop = false;
    printf("Starting threads...\n");
    std::thread thr1(ThreadHeartbeat);
    std::thread thr2(ThreadCanRead);
    std::thread thr3(ThreadParser);
    thr1.detach();
    thr2.detach();
    thr3.detach();
    return 0;
}

int Stop() {    // функция останавливающая все треды
    printf("Stopping threads...\n");
    stop = true;
    return 0;
}

void ThreadDebug() {
    printf("Debug counter started\n");
    while(!stop) {
        debugCounter += 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    printf("Debug counter stopped\n");
}

// функции, возвращающие значения
unsigned long GetDebugCounter() {return inputMsg.size();}
double GetLatitude() {return latitude;}	// возвращает широту в градусах
double GetLongitude() {return longitude;}	// возвращает долготу в градусах
double GetAltitude() {return altitude;}	// возвращает высоту в метрах (от барометра)
double GetGpsAltitude() {return gpsAltitude;}	// возвращает высоту в метрах (от GPS)
double GetSpeed() {return speed;}	// возвращает скорость в м/с
fixType_t GetFixType() {return fix;}	// возвращает тип фиксации
uint8_t GetNumSat() {return satellite;}	// возвращает кол-во найденных спутников
double GetHeading() {return heading;}	// возвращает курс в градусах (c компенсацией наклона)
double GetHeadingNc() {return headingNc;}	// возвращает курс в градусах (без компенсации наклона)
double GetCog() {return cog;}		// возвращает курс над землей в градусах (относительно сторон света)
double GetVsi() {return vsi;}		// возвращает скорость набора высоты в м/с (от барометра)
double GetVsiGps() {return gpsVsi;}	// возвращает скорость набора высоты в м/с (от GPS)
double GetHdop() {return hdop;}	// возвращает горизонтальный DOP (dilution of precision)
double GetVdop() {return vdop;}	// возрващает вертикальный DOP (dilution of precision)
int8_t GetPitch() {return pitch;}	// возвращает угол рысканья в градусах
int16_t GetRoll() {return roll;}	// возвращает угол крена в градусах
uint8_t GetYear() {return year;}	// возвращает год от GPS (последние 2 цифры)
uint8_t GetMonth() {return month;}	// возвращает месяц от GPS
uint8_t GetDay() {return day;}		// возвращает число от GPS
uint8_t GetHour() {return hour;}	// возвращает часы от GPS (для времени от 16:00 до 23:59 GPS возвращает 0:00 - 7:59)
uint8_t GetMinute() {return minute;}	// возвращает минуты от GPS
uint8_t GetSecond() {return second;}	// возвращает секунды от GPS
uint16_t GetBattery() {return battery;}	// возвращает напряжение батарейки в мВ
uint16_t GetMotorOutput(int mot) {return motorOut[mot];}	// возвращает значение подаваемое на мотор (0 - не используется, иначе 16920~35000, 16920 = мотор выкл)
int16_t GetRcIn(int chan) {return rcIn[chan];}		// возвращает значение получаемое от джойстика для каждого канала (-1000~1000)
flyMode_t GetMode() {return mode;}	// возвращает текущий режим работы
#ifdef GET_SMART_BATTERY_DATA
uint8_t GetBatteryPercent(){return batteryPercent;}	// возвращает заряд батареи в процентах
uint16_t GetBatteryCell(smartBatteryCell_t cell) {return batteryCell[cell];}	// возвращает напряжение банки в мВ
#endif

/*
 * Дальше все что нужно для питона
 */

static PyObject *NCDError;  //Naza Can Decoder Error
//PyTypeObject Decoder_Type;    // тип данных нового класса ?

typedef struct {    // структура объекта, не должна быть пустой, скорее всего потом я что-нибудь отсюда уберу
    PyObject_HEAD
    double longitude;	// долгота (град)
    double latitude;	// широта (град)
    double altitude;	// высота по барометру (м)
    double gpsAltitude;	// высота по GPS (м)
    double speed;		// скорость (м/с)
    uint8_t fix;		// тип фиксации
    uint8_t satellite;	// кол-во спутников
    double heading;		// курс в градусах (с компенсацией наклона)
    double headingNc;	// курс в градусах (без компенсации наклона)
    double cog;			// "курс над землей" относительно сторон света
    double vsi;			// скорость набора высоты по барометру (м/с)
    double gpsVsi;		// скорость набора высоты по GPS (м/с)
    double hdop;		// горизонтальный DOP
    double vdop;		// вертикальный DOP
    int8_t pitch;		// угол рысканья (градусы)
    int16_t roll;		// угол крена (радианы)
    uint8_t year;		// год (минус 2000)
    uint8_t month;		// месяц
    uint8_t day;		// число
    uint8_t hour;		// часы
    uint8_t minute;		// минуты
    uint8_t second;		// секунды
    uint16_t battery;	// напряжение аккумулятора (мВ)
    uint8_t mode;		// режим полета
#ifdef GET_SMART_BATTERY_DATA
    uint8_t batteryPercent;		// заряд аккумулятора (%)
#endif
} NazaCanDecObject;

static void NazaCanDecoder_dealloc(NazaCanDecObject* self){     // деструктор, очищает память при удалении объекта
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static int NazaCanDecoder_init(NazaCanDecObject* self, PyObject *args){ // инициализация класса
    return 0;
}

static PyObject* NazaCanDecoder_new(PyTypeObject *type, PyObject *args, PyObject *kwds){    // конструктор класса
    NazaCanDecObject* self; // объявляем что self'ом теперь будет объявленная ранее структура
    self = (NazaCanDecObject *)type->tp_alloc(type, 0);
    if(self != NULL)    // обнуление переменных
    {
        self -> longitude = 0;
        self -> latitude = 0;
        self -> altitude = 0;
        self -> gpsAltitude = 0;
        self -> speed = 0;
        self -> fix = 0;
        self -> satellite = 0;
        self -> heading = 0;
        self -> headingNc = 0;
        self -> cog = 0;
        self -> vsi = 0;
        self -> hdop = 0;
        self -> vdop = 0;
        self -> gpsVsi = 0;
        self -> pitch = 0;
        self -> roll = 0;
        self -> year = 0;
        self -> month = 0;
        self -> day = 0;
        self -> hour = 0;
        self -> minute = 0;
        self -> second = 0;
        self -> battery = 0;
        self -> mode = 0;
#ifdef GET_SMART_BATTERY_DATA
        uint8_t batteryPercent = 0;
#endif
    }
    return (PyObject*)self;
}


// метод Begin, запускает треды, прослушивание CAN, парсинг сообщений, онлайн метки, аргумент - имя шины CAN
static PyObject * NazaCanDecoder_Begin(PyObject *self, PyObject *args) {    // п
    const char *canBus;
    int ret;    // результат выполнения функции Begin
    if(!PyArg_ParseTuple(args, "s", &canBus)) return NULL;  // пробуем парсить строку
    ret = Begin(canBus);
    if(ret != 0) {  // смотрим на результат выполнения команды
        PyErr_SetString(NCDError, "Begin failed");
        return NULL;
    }
    return PyLong_FromLong(ret);
}

// метод Stop - останавливает все треды
static PyObject * NazaCanDecoder_Stop(PyObject *self, PyObject *args) {
    int ret = Stop();
    PyLong_FromLong(ret);
}

// методы возвращающие посчитанные значения
static PyObject * NazaCanDecoder_GetDebugCounter(PyObject *self, PyObject *args) {
    unsigned long ret;
    ret = GetDebugCounter();
    PyLong_FromUnsignedLong(ret);
}
static PyObject * NazaCanDecoder_GetLatitude(PyObject *self, PyObject *args) {  // широта
    double ret = GetLatitude();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetLongitude(PyObject *self, PyObject *args) {  // долгода
    double ret = GetLongitude();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetAltitude(PyObject *self, PyObject *args) {  // высота по барометру
    double ret = GetAltitude();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetGpsAltitude(PyObject *self, PyObject *args) {  // высота по GPS
    double ret = GetGpsAltitude();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetSpeed(PyObject *self, PyObject *args) {  // скорость
    double ret = GetSpeed();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetFixType(PyObject *self, PyObject *args) {  // тип фиксации
    uint8_t ret = GetFixType();
    PyLong_FromSize_t(ret);
}
static PyObject * NazaCanDecoder_GetNumSat(PyObject *self, PyObject *args) {  // кол-во спутников
    uint8_t ret = GetNumSat();
    PyLong_FromSize_t(ret);
}
static PyObject * NazaCanDecoder_GetHeading(PyObject *self, PyObject *args) {  // курс
    double ret = GetHeading();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetHeadingNc(PyObject *self, PyObject *args) {  // курс без компенсации наклона
    double ret = GetHeadingNc();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetCog(PyObject *self, PyObject *args) {  // курс над землей
    double ret = GetCog();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetVsi(PyObject *self, PyObject *args) {  // скорость набора высоты (барометр)
    double ret = GetVsi();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetVsiGps(PyObject *self, PyObject *args) {  // скорость набора высоты (GPS)
    double ret = GetVsiGps();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetHdop(PyObject *self, PyObject *args) {  // горизонтальный DOP
    double ret = GetHdop();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetVdop(PyObject *self, PyObject *args) {  // вертикальный DOP
    double ret = GetVdop();
    PyFloat_FromDouble(ret);
}
static PyObject * NazaCanDecoder_GetPitch(PyObject *self, PyObject *args) {  // угол рысканья, град
    int8_t ret = GetPitch();
    PyLong_FromSsize_t(ret);
}
static PyObject * NazaCanDecoder_GetRoll(PyObject *self, PyObject *args) {  // угол крена, град
    int16_t ret = GetRoll();
    PyLong_FromSsize_t(ret);
}
static PyObject * NazaCanDecoder_GetYear(PyObject *self, PyObject *args) {  // год
    uint8_t ret = GetYear();
    PyLong_FromSize_t(ret);
}
static PyObject * NazaCanDecoder_GetMonth(PyObject *self, PyObject *args) {  // месяц
    uint8_t ret = GetMonth();
    PyLong_FromSize_t(ret);
}
static PyObject * NazaCanDecoder_GetDay(PyObject *self, PyObject *args) {  // день
    uint8_t ret = GetDay();
    PyLong_FromSize_t(ret);
}
static PyObject * NazaCanDecoder_GetHour(PyObject *self, PyObject *args) {  // час
    uint8_t ret = GetHour();
    PyLong_FromSize_t(ret);
}
static PyObject * NazaCanDecoder_GetMinute(PyObject *self, PyObject *args) {  // минута
    uint8_t ret = GetMinute();
    PyLong_FromSize_t(ret);
}
static PyObject * NazaCanDecoder_GetSecond(PyObject *self, PyObject *args) {  // секунда
    uint8_t ret = GetSecond();
    PyLong_FromSize_t(ret);
}
static PyObject * NazaCanDecoder_GetBattery(PyObject *self, PyObject *args) {  // заряд аккумулятора
    uint16_t ret = GetBattery();
    PyLong_FromSize_t(ret);
}
static PyObject * NazaCanDecoder_GetMotorOut(PyObject *self, PyObject *args) {// значение подаваемое на мотор, по номеру
    int motor;
    if(!PyArg_ParseTuple(args, "i", &motor)) return NULL;
//    uint16_t ret = GetMotorOutput(static_cast<motorOut_t>(motor));
    uint16_t ret = GetMotorOutput(motor);
    PyLong_FromSize_t(ret);
}
static PyObject * NazaCanDecoder_GetRcIn(PyObject *self, PyObject *args) {  // управляющее воздействие, по каналу
    int channel;
    if(!PyArg_ParseTuple(args, "i", &channel)) return NULL;
//    int16_t ret = GetRcIn(static_cast<rcInChan_t>(channel));
    int16_t ret = GetRcIn(channel);
    PyLong_FromSsize_t(ret);
}
static PyObject * NazaCanDecoder_GetMode(PyObject *self, PyObject *args) {  // режим полета
    uint8_t ret = GetMode();
    PyLong_FromSize_t(ret);
}
#ifdef GET_SMART_BATTERY_DATA
static PyObject * NazaCanDecoder_GetBatteryPercent(PyObject *self, PyObject *args){ // заряд батареи
    uint8_t ret = GetBatteryPercent();
    PyLong_FromSize_t(ret);
}
static PyObject * NazaCanDecoder_GetBatteryCell(PyObject *self, PyObject *args){ // заряд банки
    int num;
    if(!PyArg_ParseTuple(args, "h", &num)) return NULL;
    uint16_t ret = GetBatteryCell(static_cast<smartBatteryCell_t>(num));
    PyLong_FromSize_t(ret);
}
#endif

static PyMethodDef NazaCanDecoderMethods[] = {  // методы модуля
        {"Begin",       NazaCanDecoder_Begin, METH_VARARGS, "Starting Naza-Can Decoder threads"},
        {"Stop",        NazaCanDecoder_Stop, METH_NOARGS, "Stoping Naza Can Decoder threads"},
        {"GetLatitude",     NazaCanDecoder_GetLatitude, METH_NOARGS, "Returns latitude, degrees"},
        {"GetLongitude",    NazaCanDecoder_GetLongitude, METH_NOARGS, "Returns longitude, degrees"},
        {"GetAltitude",     NazaCanDecoder_GetAltitude, METH_NOARGS, "Returns altitude (barometric), m"},
        {"GetAltitudeGps",  NazaCanDecoder_GetGpsAltitude, METH_NOARGS, "Returns altitude (GPS), m"},
        {"GetSpeed",    NazaCanDecoder_GetSpeed, METH_NOARGS, "Returns speed (GPS), m/s"},
        {"GetFixType",  NazaCanDecoder_GetFixType, METH_NOARGS, "Returns fix type (0 - no fix, 2 - 2D, 3 - 3D, 4 - DGPS)"},
        {"GetNumSat",   NazaCanDecoder_GetNumSat, METH_NOARGS, "Returns number of found satellites"},
        {"GetHeading",  NazaCanDecoder_GetHeading, METH_NOARGS, "Returns heading (compensated), degrees"},
        {"GetHeadingNc",    NazaCanDecoder_GetHeadingNc, METH_NOARGS, "Returns heading (not compensated), degrees"},
        {"GetCog",      NazaCanDecoder_GetCog, METH_NOARGS, "Returns course over ground, degrees"},
        {"GetVsi",      NazaCanDecoder_GetVsi, METH_NOARGS, "Returns vertical speed (barometric), m/s"},
        {"GetVsiGps",   NazaCanDecoder_GetVsiGps, METH_NOARGS, "Returns vertical speed (GPS), m/s"},
        {"GetHdop",     NazaCanDecoder_GetHdop, METH_NOARGS, "Returns horizontal Dilution of Precision"},
        {"GetVdop",     NazaCanDecoder_GetVdop, METH_NOARGS, "Returns vertical Dilution of Precision"},
        {"GetPitch",    NazaCanDecoder_GetPitch, METH_NOARGS, "Returns pitch, degrees"},
        {"GetRoll",     NazaCanDecoder_GetRoll, METH_NOARGS, "Returns roll, degrees"},
        {"GetYear",     NazaCanDecoder_GetYear, METH_NOARGS, "Returns year (GPS), minus 2000"},
        {"GetMonth",    NazaCanDecoder_GetMonth, METH_NOARGS, "Returns month (GPS)"},
        {"GetDay",      NazaCanDecoder_GetDay, METH_NOARGS, "Returns day (GPS)"},
        {"GetHour",     NazaCanDecoder_GetHour, METH_NOARGS, "Returns hour (GPS)"},
        {"GetMinute",   NazaCanDecoder_GetMinute, METH_NOARGS, "Returns minute (GPS)"},
        {"GetSecond",   NazaCanDecoder_GetSecond, METH_NOARGS, "Returns second (GPS)"},
        {"GetBattery",  NazaCanDecoder_GetBattery, METH_NOARGS, "Returns battery voltage, mV"},
        {"GetMotorOut", NazaCanDecoder_GetMotorOut, METH_VARARGS, "Returns motor output (8 motors total, 0 - unused, otherwise 16920~35000, 16920 = motor off)"},
        {"GetRcIn",     NazaCanDecoder_GetRcIn, METH_VARARGS, "Returns RC stick input (10 channels total, -1000~1000)"},
        {"GetMode",     NazaCanDecoder_GetMode, METH_NOARGS, "Returns flight mode (0 - Manual, 1 - GPS, 2 - Failsafe, 3 - Atti)"},
#ifdef GET_SMART_BATTERY_DATA
{"GetBatteryPercent",   NazaCanDecoder_GetBatteryPercent, METH_VARARGS, "Returns battery charge percentage (0-100%)"},
        {"GetBatteryCell",      NazaCanDecoder_GetBatteryCell, METH_VARARGS, "Returns battery cell voltage in mV"},
#endif
        {NULL, NULL, 0, NULL}   // обязательный член, отмечает конец списка методов
};

static PyMemberDef NazaCanDec_members[] = {     // переменные модуля
        {"longitude", T_FLOAT, offsetof(NazaCanDecObject, longitude), READONLY, "Longitude, degrees"},
        {"latitude", T_FLOAT, offsetof(NazaCanDecObject, latitude), READONLY, "Latitude, degrees"},
        {"altitude", T_FLOAT, offsetof(NazaCanDecObject, altitude), READONLY, "Altitude (barometric), m"},
        {"gpsAltitude", T_FLOAT, offsetof(NazaCanDecObject, gpsAltitude), READONLY, "Altitude (GPS), m"},
        {"speed", T_FLOAT, offsetof(NazaCanDecObject, speed), READONLY, "Speed (GPS), m/s"},
        {"fix", T_INT, offsetof(NazaCanDecObject, fix), READONLY, "Fix type (0 - no fix, 2 - 2D, 3 - 3D, 4 - DGPS)"},
        {"satellite", T_INT, offsetof(NazaCanDecObject, satellite), READONLY, "Number of found satellites"},
        {"heading", T_FLOAT, offsetof(NazaCanDecObject, heading), READONLY, "Heading (compensated), degrees"},
        {"headingNc", T_FLOAT, offsetof(NazaCanDecObject, headingNc), READONLY, "Heading (not compensated), degrees"},
        {"cog", T_FLOAT, offsetof(NazaCanDecObject, cog), READONLY, "Course over ground, degrees"},
        {"vsi", T_FLOAT, offsetof(NazaCanDecObject, vsi), READONLY, "Vertical speed (GPS), m/s"},
        {"vsiGps", T_FLOAT, offsetof(NazaCanDecObject, gpsVsi), READONLY, "vertical speed (barometric), m/s"},
        {"hdop", T_FLOAT, offsetof(NazaCanDecObject, hdop), READONLY, " Horizontal Dilution of Precision"},
        {"vdop", T_FLOAT, offsetof(NazaCanDecObject, vdop), READONLY, " Vertical Dilution of Precision"},
        {"pitch", T_INT, offsetof(NazaCanDecObject, pitch), READONLY, "Pitch, degrees"},
        {"roll", T_INT, offsetof(NazaCanDecObject, roll), READONLY, "Roll, degrees"},
        {"year", T_INT, offsetof(NazaCanDecObject, year), READONLY, "Year (GPS), minus 2000"},
        {"month", T_INT, offsetof(NazaCanDecObject, month), READONLY, "Month (GPS)"},
        {"day", T_INT, offsetof(NazaCanDecObject, day), READONLY, "Day (GPS)"},
        {"hour", T_INT, offsetof(NazaCanDecObject, hour), READONLY, "Hour (GPS)"},
        {"minute", T_INT, offsetof(NazaCanDecObject, minute), READONLY, "Minute (GPS)"},
        {"second", T_INT, offsetof(NazaCanDecObject, second), READONLY, "Second (GPS)"},
        {"battery", T_INT, offsetof(NazaCanDecObject, battery), READONLY, "Battery voltage, mV"},
        {"mode", T_INT, offsetof(NazaCanDecObject, mode), READONLY, "Flight mode (0 - Manual, 1 - GPS, 2 - Failsafe, 3 - Atti)"},
#ifdef GET_SMART_BATTERY_DATA
        {"batteryPercent", T_INT, offsetof(NazaCanDecObject, batteryPercent), READONLY, "Battery charge percentage (0-100%)"},
#endif
        {NULL}  // завершение списка
};

static struct PyModuleDef NazaCanDecoderModule = {  // описание модуля
        PyModuleDef_HEAD_INIT,
        "NazaCanDecoder",
        NULL,
        -1,
        NazaCanDecoderMethods
};

static PyTypeObject Decoder_Type = { // описание нового объекта Python
        PyVarObject_HEAD_INIT(NULL, 0)
        "NazaCanDecoder.Decoder",       // tp_name
        sizeof(NazaCanDecObject),       // basic size
        0,                              // tp_itemsize
        (destructor)NazaCanDecoder_dealloc, // tp_dealloc
        0,                                  // tp_print
        0,                              // tp_getattr
        0,                              // tp_setattr
        0,                              // tp_reserved
        0,                              // tp_repr
        0,                              // tp_as_number
        0,                              // tp_as_sequence
        0,                              // tp_as_mapping
        0,                              // tp_hash
        0,                              // tp_call
        0,                              // tp_str
        0,                              // tp_getattro
        0,                              // tp_setattro
        0,                              // tp_as_buffer
        Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,   // tp_flags
        "Decoder object",               // tp_doc
        0,                              // tp_traverse
        0,                              // tp_clear
        0,                              // tp_richcompare
        0,                              // tp_weaklistoffset
        0,                              // tp_iter
        0,                              // tp_iternext
        NazaCanDecoderMethods,          // tp_methods
        0,                              // tp_members
        0,                              // tp_getset
        0,                              // tp_base
        0,                              // tp_dict
        0,                              // tp_descr_get
        0,                              // tp_descr_set
        0,                              // tp_dictoffset
        (initproc)NazaCanDecoder_init,  // tp_init
        0,                              // tp_alloc
        NazaCanDecoder_new,             // tp_new
};
#define Decoder_Check(x) ((x) -> ob_type == &Decoder_Type)        // ф-ия проверки


PyMODINIT_FUNC
PyInit_NazaCanDecoder() {   // инициализация модуля
    PyObject *m;
    if(PyType_Ready(&Decoder_Type) < 0) return NULL;
    m = PyModule_Create(&NazaCanDecoderModule); // создаем модуль
    if(m == NULL) return NULL;
    NCDError = PyErr_NewException("pymod.error", NULL, NULL);  // создаем исключение
    Py_INCREF(NCDError);
    PyModule_AddObject(m, "error", NCDError);

    Py_INCREF(&Decoder_Type);
    PyModule_AddObject(m, "Decoder", (PyObject *)&Decoder_Type);

    return m;
}

int main(int argc, char *argv[]) {  // запуск библиотеки
    wchar_t *program = Py_DecodeLocale(argv[0], NULL);  // ловим параметры при инициализации
    if(program == NULL) {   // ошибки определения параметров
        fprintf(stderr, "Fatal error: cannot decode argv[0]\n");
        exit(1);
    }
    // говорим какая функция отвечает за инициализацию модуля
    PyImport_AppendInittab("NazaCanDecoder",PyInit_NazaCanDecoder);
    // передаем argv[0] интерпретатору питона
    Py_SetProgramName(program);
    // инициализируем питон
    Py_Initialize();
    // импортируем модуль
    PyImport_ImportModule("NazaCanDecoder");
    PyMem_RawFree(program);
    return 0;
}
