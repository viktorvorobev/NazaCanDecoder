/*
 * wrapper.cpp
 *
 *  Created on: 7 февр. 2018 г.
 *      Author: vikey
 */
#include <python3.5/Python.h>
#include "NazaCanDecoder.h"

static PyObject *NCDError;  //Naza Can Decoder Error

static PyObject * NazaCanDecoder_Begin(PyObject *self, PyObject *args)  // метод Begin, аргумент - имя шины CAN
{
    const char *canBus;
    int ret;    // результат выполнения функции Begin
    if(!PyArg_ParseTuple(args, "s", &canBus)) return NULL;  // пробуем парсить строку
    ret = Begin(canBus);
    if(ret < 0) // смотрим на результат выполнения команды
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

static PyMethodDef NazaCanDecoderMethods[] =    // методы модуля
{
    {"Begin", NazaCanDecoder_Begin, METH_VARARGS, "Starting Naza-Can Decoder threads."},
    {"GetDebugCounter", NazaCanDecoder_GetDebugCounter, METH_VARARGS, "Get Debug counter value"},
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

