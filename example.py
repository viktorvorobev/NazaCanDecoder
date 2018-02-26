#!/usr/bin/env python3
import NazaCanDecoder   # после установки модуль подключается глобально
import time

naza = NazaCanDecoder.Decoder()    # запуск модуля с привязкой к конкретной CAN шине
naza.Begin("can0")
for i in range(1000):
    print(naza.GetBattery())
    time.sleep(0.001)
naza.Stop()   # завершение работы модуля
