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

    if (getifaddrs(&ifaddr) == -1) {    // получаем список всех интерфейсов
        perror("ERROR: Failed to get ifaddrs.");
        return -1;
    }
    for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++)    // ищем нужный интерфейс
        // если нашли нужный интерфейс, и он "поднят", меняем флаг - разрешаем дальнейшую работу
        if ((!strcmp(interface, ifa->ifa_name)) && ifa->ifa_flags & IFF_UP)   // (strcmp: полное совпадение == 0)
            noSuchInterface = false;

    if (noSuchInterface) {  // Ошибка что интерфейс не найден
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
    while (!stop) {
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
    while (!stop)
    {
        struct can_frame frame{};
        read(canSocket, &frame, sizeof(struct can_frame));  // читаем
        inputMsg.push(frame);   // помещаем в очередь
    }
    printf("Can reading stopped\n");
}

void ThreadParser() {
    while (!stop) {
        uint16_t msgId = NAZA_MESSAGE_NONE;
        if (inputMsg.size() != 0) {   // если в очереди что-то есть
            struct can_frame frame = inputMsg.front();  // забираем первый элемент
            inputMsg.pop(); // удаляем его из очереди
        // TODO: дописать парсер
        }
    }
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
    thr1.detach();
    thr2.detach();
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
uint16_t GetMotorOutput(motorOut_t mot) {return motorOut[mot];}	// возвращает значение подаваемое на мотор (0 - не используется, иначе 16920~35000, 16920 = мотор выкл)
int16_t GetRcIn(rcInChan_t chan) {return rcIn[chan];}		// возвращает значение получаемое от джойстика для каждого канала (-1000~1000)
flyMode_t GetMode() {return mode;}	// возвращает текущий режим работы
#ifdef GET_SMART_BATTERY_DATA
uint8_t GetBatteryPercent(){return batteryPercent;}	// возвращает заряд батареи в процентах
uint16_t GetBatteryCell(smartBatteryCell_t cell) {return batteryCell[cell];}	// возвращает напряжение банки в мВ
#endif

static PyObject *NCDError;  //Naza Can Decoder Error

static PyObject * NazaCanDecoder_Begin(PyObject *self, PyObject *args) {    // метод Begin, аргумент - имя шины CAN
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

static PyObject * NazaCanDecoder_GetDebugCounter(PyObject *self, PyObject *args) {
    int8_t ret;
    ret = GetDebugCounter();
    PyLong_FromUnsignedLong(ret);
}

static PyObject * NazaCanDecoder_Stop(PyObject *self, PyObject *args) {
    int ret = Stop();
    PyLong_FromLong(ret);
}

static PyMethodDef NazaCanDecoderMethods[] = {  // методы модуля
        {"Begin", NazaCanDecoder_Begin, METH_VARARGS, "Starting Naza-Can Decoder threads."},
        {"GetDebugCounter", NazaCanDecoder_GetDebugCounter, METH_VARARGS, "Get Debug counter value"},
        {"Stop", NazaCanDecoder_Stop, METH_VARARGS, "Stoping Naza Can Decoder threads."},
        {NULL, NULL, 0, NULL}
    };

static struct PyModuleDef NazaCanDecoderModule = {  // описание модуля
        PyModuleDef_HEAD_INIT,
        "NazaCanDecoder",
        NULL,
        -1,
        NazaCanDecoderMethods
    };

PyMODINIT_FUNC
PyInit_NazaCanDecoder() {   // инициализация модуля
    PyObject *m;
    m = PyModule_Create(&NazaCanDecoderModule); // создаем модуль
    if (m == NULL) return NULL;
    NCDError = PyErr_NewException("pymod.error", NULL, NULL);  // создаем исключение
    Py_INCREF(NCDError);
    PyModule_AddObject(m, "error", NCDError);
    return m;
}

int main(int argc, char *argv[]) {  // запуск библиотеки
    wchar_t *program = Py_DecodeLocale(argv[0], NULL);  // ловим параметры при инициализации
    if (program == NULL) {  // ошибки определения параметров
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
