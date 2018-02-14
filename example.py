#!/usr/bin/env python3
import NazaCanDecoder   # после установки модуль подключается глобально
import time

NazaCanDecoder.Begin("can0")    # запуск модуля с привязкой к конкретной CAN шине
for i in range(1000):
    print(NazaCanDecoder.GetBattery())
    time.sleep(0.001)
NazaCanDecoder.Stop()   # завершение работы модуля
