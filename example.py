#!/usr/bin/env python3
import NazaCanDecoder
import time

naza = NazaCanDecoder.Decoder()  # запуск модуля с привязкой к конкретной CAN шине
try:
    naza.Begin("can0")
except NazaCanDecoder.error:
    print("Failed to start reading")
    exit(1)
motor = [0] * 8
rcIn = [0] * 10

for i in range(1000):
    # These values are not require GPS module to be connected
    altitude = naza.GetAltitude()
    vertical_speed = naza.GetVsi()  # vertical speed (barometric)
    pitch = naza.GetPitch()
    roll = naza.GetRoll()
    battery = naza.GetBattery() / 1000
    mode = naza.GetMode()
    heading = naza.GetHeading()
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
    msg_no_gps = 'mode: %s\tbat: %.2f\talt: %.2f\tvsi: %.2f\tpitch: %d\troll: %d\theading: %.2f' \
                 % (modeStr, battery, altitude, vertical_speed, pitch, roll, heading)

    for i in range(8):  # these are values that are sent to motors
        motor[i] = naza.GetMotorOut(i)
    msg_motors = 'M1: %d\tM2: %d\tM3: %d\tM4: %d\tM5: %d\tM6: %d\tF1: %d\tF2: %d\t' \
                 % (motor[0], motor[1], motor[2], motor[3], motor[4], motor[5], motor[6], motor[7])

    for i in range(10):  # these are readings from te remote
        rcIn[i] = naza.GetRcIn(i)
    msg_rc_in = 'un1: %d\tRcA: %d\tRcE: %d\tRcR: %d\tRcU: %d\t' \
                'RcT: %d\tun2: %d\tRcX1: %d\tRcX2: %d\tun3: %d' \
                % (rcIn[0], rcIn[1], rcIn[2], rcIn[3], rcIn[4], rcIn[5], rcIn[6], rcIn[7], rcIn[8], rcIn[9])

    # these values are require GPS module to be connected
    altitude_gps = naza.GetAltitudeGps()
    latitude = naza.GetLatitude()
    longitude = naza.GetLongitude()
    speed = naza.GetSpeed()
    fix_type = naza.GetFixType()
    num_satellites = naza.GetNumSat()
    course_over_ground = naza.GetCog()
    vertical_speed_gps = naza.GetVsiGps()
    hdop = naza.GetHdop()  # horizontal Dilution of Precision
    vdop = naza.GetVdop()  # vertical Dilution of Precision
    heading_magnetometer = naza.GetHeadingNc()
    year = naza.GetYear()
    month = naza.GetMonth()
    day = naza.GetDay()
    hour = naza.GetHour()
    min = naza.GetMinute()
    sec = naza.GetSecond()
    if fix_type == 0:
        fixTypeStr = 'NoFix'
    elif fix_type == 2:
        fixTypeStr = '2D'
    elif fix_type == 3:
        fixTypeStr = '3D'
    elif fix_type == 4:
        fixTypeStr = 'DGPS'
    else:
        fixTypeStr = 'Unknown'
    msg_gps_1 = 'fix: %s\tnumSat: %d\talt: %.2f\tlat: %.2f\tlon:%.2f\tspeed: %.2f' % (
        fixTypeStr, num_satellites, altitude_gps, latitude, longitude, speed)
    msg_gps_2 = 'headingNc: %.2f\tcog: %.2f\tvsi: %.2f\thdop: %.2f\tvdop: %.2f\ttime: %.2d:%.2d:%.2d\t date: %.2d.%.2d.%.2d' \
                % (heading_magnetometer, course_over_ground, vertical_speed_gps, hdop, vdop, hour, min, sec, day, month,
                   year)

    print(msg_no_gps)
    # print(msg_motors)
    # print(msg_rc_in)
    # print(msg_gps_1)
    # print(msg_gps_2)
    time.sleep(0.01)
naza.Stop()
