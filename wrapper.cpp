/*
 * wrapper.cpp
 *
 *  Created on: 7 февр. 2018 г.
 *      Author: vikey
 */
#include <python3.5/Python.h>
#include "NazaCanDecoder.h"


static PyObject * NazaCanDecoder_Begin(PyObject *self, PyObject *args)
{

}

static PyMethodDef NazaCanDecoderMethods[] =
{
		{"Begin", NazaCanDecoder_Begin, METH_VARARGS, "Starting Naza-Can Decoder threads."},
		{NULL, NULL, 0, NULL}
};

static struct PyModuleDef NazaCanDecoderModule =
{
		PyModuleDef_HEAD_INIT,
		"NazaCanDecoder",
		NULL,
		-1,
		NazaCanDecoderMethods
};

PyMODINIT_FUNC
PyInit_NazaCanDecoder()
{
	PyObject *m;
	m = PyModule_Create(&NazaCanDecoderModule);
}


