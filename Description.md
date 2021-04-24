## Module description
This is a Python3 port of [NazaCanDecoder Arduino Library](https://www.rcgroups.com/forums/showthread.php?2071772-DJI-NAZA-Phantom-A2-CAN-bus-communication-protocol-NazaCanDecoder-Arduino-library).
This module is designed to decode CAN messages from Naza Mv2 flight controller.
It can **only read** data.

## Beginning
- First you need to compile module:  
  `sudo python3 setup.py build`  
- Then you need to install module (you should be able to install it to `venv` but I didn't check that)  
  `sudo python3 setup.py install`  


## Description
*You can just check the example, it's pretty much straight forward*.  
`NazaCanDecoder.Decoder()` - cass that you're looking for.
  - `Begin("can0")` - That's how you start listening. Argument - name of the CAN bus.
    Now you can use getter methods to read every value you like.
  - `Stop` - that's how you stop everything  

### Getter methods

**Methods that don't require GPS module**
- `GetAltitude` - barometric altitude (meters)
- `GetVsi` - barometric vertical speed (m/s)
- `GetPitch` - pitch (degrees)
- `GetRoll` - pitch (degrees)
- `GetBattery` - battery voltage (mV)
- `GetMode` - flight mode (0 - Manual, 1 - GPS, 2 - Failsafe, 3 - Atti)
- `GetHeading` - heading (degrees)
- `GetMotorOut(number)` - value sent to the motor (0 - unused, otherwise 16920 ~ 35000)   
  Argument - integer motor number:
  - 0 - M1 
  - 1 - M2
  - 2 - M3
  - 3 - M4
  - 4 - M5
  - 5 - M6
  - 6 - F1
  - 7 - F2
- `GetRcIn(number)` - RC controller values (-1000~1000)
  Argument - integer channel number:
  - 0 - unused
  - 1 - Rc_A (aileron)
  - 2 - Rc_E (elevator)
  - 3 - Rc_R (rudder)
  - 4 - Rc_U
  - 5 - Rc_T (throttle)
  - 6 - unused,
  - 7 - Rc_X1
  - 8 - Rc_X2
  - 9 - unused

**Methods that require GPS module**
- `GetAltitudeGps` - altitude from GPS (m)
- `GetLatitude` - latitude (degrees)
- `GetLongitude` - longitude (degrees)
- `GetHeadingNc` - not compensated heading (degrees)
- `GetSpeed` - speed (m/s)
- `GetFixType` - 0 - no fix, 2 - 2D, 3 - 3D, 4 - DPGS
- `GetNumSat` - number of visible satellites
- `GetCog` - course over ground (degrees)
- `GetVsiGps` - vertical speed from GPS (m/s)
- `GetHdop` - horizontal DOP (dilution of precision, see uBlox NAV-DOP message for details)
- `GetVdop` - vertical DOP
- `GetYear` - last two numbers of the year
- `GetMonth` - month
- `GetDay` - day of month
- `GetHour` - hours (for the time between 16:00 and 23:59 the hour will be returned as 0-7 and there seems to be no way to differentiate between 00:00 - 07:59 and 16:00 - 23:59.)
- `GetMinute` - minutes
- `GetSecond` - seconds
**Note:** GPS returns UTC time.
