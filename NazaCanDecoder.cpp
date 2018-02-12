#include "NazaCanDecoder.h"

/*
 * Работа модуля.
 * Модуль работает в нескольких потоках (тредах):
 * 1) Отправка HeartBeat в CAN сеть.
 * 2) Получение сообщений из CAN сети, их парсинг.
 */

int InitCanSocket(int *sock, const char* interface)	// инициализация CAN
{
	int s, n;
	struct sockaddr_can addr{};
	struct ifreq ifr{};
    bool noSuchInterface = true;    // флаг, что интерфейс вообще есть
    struct ifaddrs *ifaddr, *ifa;

    if (getifaddrs(&ifaddr) == -1)  // получаем список всех интерфейсов
    {
        perror("ERROR: Failed to get ifaddrs.");
        return -1;
    }
    for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++)    // ищем нужный интерфейс
    {
        //TODO: правильно описать сравнение строк
        if (strcmp(interface, ifa->ifa_name))   //если нашли - меняем флаг
            noSuchInterface = false;
        else continue;
    }

    if (noSuchInterface)    // Ошибка что интерфейс не найден
    {
        printf("ERROR: No interface with name %s found.", interface);
        perror("ERROR: No interface with that name found.");
        return -2;
    }

	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) // открываем порт
	{
		perror("ERROR: Failed to open port.");
		return -3;
	}
	strcpy(ifr.ifr_name, interface);
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
    int b = bind(s, (struct sockaddr *)&addr, sizeof(addr));    // занимаем порт
	if(b != 0)
	{
//        std::cout << "Error binding socket\n";
		perror("ERROR: Failed to bind socket.");
		return -4;
	}


	(*sock) = s;
    std::cout << "Socket inited\n";
	return 0;
}

void Heartbeat()
{
    std::cout << "Heartbeat started\n";
	while (!stop)
	{
        struct can_frame frame{};
		frame = HEARTBEAT_1;
		write(canSocket, &frame, sizeof(struct can_frame));
		frame = HEARTBEAT_2;
		write(canSocket, &frame, sizeof(struct can_frame));
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
    std::cout << "Heartbeat stopped\n";
}

int Begin(const char* canBus)
{
	// создаем HeartBeat сообщения
	HEARTBEAT_1.can_id = 0x108;
//	int data1[8] = {0x55, 0xAA, 0x55, 0xAA, 0x07, 0x10, 0x00, 0x00};
    HEARTBEAT_1.data[0] = 0x55;
    HEARTBEAT_1.data[1] = 0xAA;
    HEARTBEAT_1.data[2] = 0x55;
    HEARTBEAT_1.data[3] = 0xAA;
    HEARTBEAT_1.data[4] = 0x07;
    HEARTBEAT_1.data[5] = 0x10;
    HEARTBEAT_1.data[6] = 0x00;
    HEARTBEAT_1.data[7] = 0x00;
//	memcpy(HEARTBEAT_1.data, data1, sizeof(data1));
	HEARTBEAT_1.can_dlc = 8;

	HEARTBEAT_2.can_id = 0x108;
//	int data2[4] = {0x66, 0xCC, 0x66, 0xCC};
//	memcpy(HEARTBEAT_2.data, data2, sizeof(data2));
    HEARTBEAT_2.data[0] = 0x66;
    HEARTBEAT_2.data[1] = 0xCC;
    HEARTBEAT_2.data[2] = 0x66;
    HEARTBEAT_2.data[3] = 0xCC;
	HEARTBEAT_2.can_dlc = 4;

	// создаем маски
//	FILTER_MASK.can_id = 0x7FF;
//	FILTER_090.can_id = 0x090;
//	FILTER_108.can_id = 0x108;
//	FILTER_7F8.can_id = 0x7F8;
    std::cout << "Initing CAN socket\n";
	if(InitCanSocket(&canSocket, canBus) != 0)	// инициализируем CAN
    {
        std::cout << "Error initing CAN bus \n";
        perror("Error initialization CAN bus");
        return -1;
    }
	stop = false;
    std::cout << "Starting threads\n";
	std::thread thr1(Heartbeat);
    std::thread thr2(DebugThread);
	thr1.detach();
    thr2.detach();
    return 0;
}

int Stop()
{
    std::cout << "Stopping threads\n";
    stop = true;
    return 0;
}

void DebugThread()
{
    std::cout << "Debug counter started\n";
    while(!stop)
    {
        debugCounter += 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    std::cout << "Debug counter stopped\n";
}

// функции, возвращающие значения
uint8_t GetDebugCounter() {return debugCounter;}
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

static PyObject * NazaCanDecoder_Begin(PyObject *self, PyObject *args)  // метод Begin, аргумент - имя шины CAN
{
    const char *canBus;
    int ret;    // результат выполнения функции Begin
    if(!PyArg_ParseTuple(args, "s", &canBus)) return NULL;  // пробуем парсить строку
    ret = Begin(canBus);
    if(ret != 0) // смотрим на результат выполнения команды
    {
        PyErr_SetString(NCDError, "Begin failed");
        return NULL;
    }
    return PyLong_FromLong(ret);
}

static PyObject * NazaCanDecoder_GetDebugCounter(PyObject *self, PyObject *args)
{
    int8_t ret;
    ret = GetDebugCounter();
    PyLong_FromLong(ret);
}

static PyObject * NazaCanDecoder_Stop(PyObject *self, PyObject *args)
{
    int ret = Stop();
    PyLong_FromLong(ret);
}

static PyMethodDef NazaCanDecoderMethods[] =    // методы модуля
    {
        {"Begin", NazaCanDecoder_Begin, METH_VARARGS, "Starting Naza-Can Decoder threads."},
        {"GetDebugCounter", NazaCanDecoder_GetDebugCounter, METH_VARARGS, "Get Debug counter value"},
        {"Stop", NazaCanDecoder_Stop, METH_VARARGS, "Stoping Naza Can Decoder threads."},
        {NULL, NULL, 0, NULL}
    };

static struct PyModuleDef NazaCanDecoderModule =    // описание модуля
    {
        PyModuleDef_HEAD_INIT,
        "NazaCanDecoder",
        NULL,
        -1,
        NazaCanDecoderMethods
    };

PyMODINIT_FUNC
PyInit_NazaCanDecoder() // инициализация модуля
{
    PyObject *m;
    m = PyModule_Create(&NazaCanDecoderModule); // создаем модуль
    if (m == NULL) return NULL;
    NCDError = PyErr_NewException("pymod.error", NULL, NULL);  // создаем исключение
    Py_INCREF(NCDError);
    PyModule_AddObject(m, "error", NCDError);
    return m;
}

int main(int argc, char *argv[])    // запуск библиотеки
{
    wchar_t *program = Py_DecodeLocale(argv[0], NULL);  // ловим параметры при инициализации
    if (program == NULL)    // ошибки определения параметров
    {
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
