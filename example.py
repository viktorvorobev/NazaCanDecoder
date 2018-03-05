#!/usr/bin/env python3
import NazaCanDecoder   # после установки модуль подключается глобально
import time

naza = NazaCanDecoder.Decoder()    # запуск модуля с привязкой к конкретной CAN шине
naza.Begin("can0")
motor = [0]*8
rcIn = [0]*10
for i in range(10000):
    # не требуют GPS
    altitude = naza.GetAltitude()
    vsi = naza.GetVsi()     #   vertical speed (barometric)
    pitch = naza.GetPitch()
    roll = naza.GetRoll()
    battery = naza.GetBattery()/1000
    mode = naza.GetMode()
    heading = naza.GetHeading()         # курс от кватерниона (видимо еще и от гироскопа)
    headingNc = naza.GetHeadingNc()     # курс от магнетометра
    if mode == 0:
        modeStr = 'manual'
    elif mode == 1:
        modeStr = 'gps'
    elif mode == 2:
        modeStr = 'failsafe'
    elif mode == 3:
        modeStr = 'atti'
    else:
        modeStr = 'unknown'
    msgNoGps = 'mode: %s\tbat: %.2f\talt: %.2f\tvsi: %.2f\tpitch: %d\troll: %d\theading: %.2f\theadingNc: %.2f' \
               % (modeStr, battery, altitude, vsi, pitch, roll, heading, headingNc)

    for i in range(8):    # значения посылаемые на моторы
        motor[i] = naza.GetMotorOut(i)
    msgMotors = 'M1: %d\tM2: %d\tM3: %d\tM4: %d\tM5: %d\tM6: %d\tF1: %d\tF2: %d\t' \
                % (motor[0], motor[1], motor[2], motor[3], motor[4], motor[5], motor[6], motor[7])

    for i in range(10):     # значения получаемые с каналов пульта
        rcIn[i] = naza.GetRcIn(i)
    msgRcIn = 'un1: %d\tRcA: %d\tRcE: %d\tRcR: %d\tRcU: %d\t' \
              'RcT: %d\tun2: %d\tRcX1: %d\tRcX2: %d\tun3: %d' \
              % (rcIn[0], rcIn[1], rcIn[2], rcIn[3], rcIn[4], rcIn[5], rcIn[6], rcIn[7], rcIn[8], rcIn[9])

    # требуют GPS
    altitudeGps = naza.GetAltitudeGps()
    latitude = naza.GetLatitude()
    longitude = naza.GetLongitude()
    speed = naza.GetSpeed()
    fixType = naza.GetFixType()
    numSat = naza.GetNumSat()
    cog = naza.GetCog()     # Course over ground
    vsiGps = naza.GetVsiGps()   # vertical speed (GPS)
    hdop = naza.GetHdop()   # horizontal Dilution of Precision
    vdop = naza.GetVdop()   # vertical Dilution of Precision
    year = naza.GetYear()
    month = naza.GetMonth()
    day = naza.GetDay()
    hour = naza.GetHour()
    min = naza.GetMinute()
    sec = naza.GetSecond()
    if fixType == 0:
        fixTypeStr = 'NoFix'
    elif fixType == 2:
        fixTypeStr = '2D'
    elif fixType == 3:
        fixTypeStr = '3D'
    elif fixType == 4:
        fixTypeStr = 'DGPS'
    else:
        fixTypeStr = 'Unknown'
    msgGps1 = 'fix: %s\tnumSat: %d\talt: %.2f\tlat: %.2f\tlon:%.2f\tspeed: %.2f' % (fixTypeStr, numSat, altitudeGps, latitude, longitude, speed)
    msgGps2 = 'cog: %.2f\tvsi: %.2f\thdop: %.2f\tvdop: %.2f\ttime: %.2d:%.2d:%.2d\t date: %.2d.%.2d.%.2d' \
              % (cog, vsiGps, hdop, vdop, hour, min, sec, day, month, year)

    # вывод сообщений
    # print(msgNoGps)     # телеметрия которой не нужен GPS
    print(msgMotors)    # значения, посылаемые на моторы
    # print(msgRcIn)      # значения, получаемые с пульта
    # print(msgGps1)      # телеметрия которой нужен GPS
    # print(msgGps2)

    time.sleep(0.01)
naza.Stop()   # завершение работы модуля








