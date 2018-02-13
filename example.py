#!/usr/bin/env python3
import NazaCanDecoder   # после установки модуль подключается глобально
import time

NazaCanDecoder.Begin("can0")    # запуск модуля с привязкой к конкретной CAN шине
time.sleep(2)
NazaCanDecoder.Stop()   # завершение работы модуля
