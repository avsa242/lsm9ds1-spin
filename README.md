# lsm9ds1-spin
## Parallax's LSM9DS1 Simple C library ported to SPIN
---

This repository houses a driver/object for the LSM9DS1 IMU module by Parallax, as well as a demo program, and the original test harness for the LSM9DS1 IMU module by Parallax (ported from C). Big thanks to Dave Hein for [cspin](http://forums.parallax.com/discussion/119342/cspin-a-c-to-spin-converter/p1) - converting this from C to SPIN would've taken a *great* deal longer without it.

### What works/what is implemented:
I believe the Accelerometer and Gyroscope data are good, as these are reasonably easy to test.
I am less confident in the magnetometer output, only because I don't know of a bona-fide way of testing it.
The integrated temperature sensor reads within a few degrees of a multimeter I have with a thermocouple, so I believe it is ok.
The Demo top-level object provides a serial terminal-based interactive test of the module. A background cog monitors for keypresses and changes the demo state corresponding to the hotkey pressed (e.g., for monitoring the current Accelerometer values, raw or calculated, or continually printing out all register values)
The dependency on floating point math objects has been removed - now all math is fixed-point. The methods that return calculated values from the sensor now return scaled up values (e.g., calculated magnetometer readings are now returned in milli-gauss, rather than gauss)

### What doesn't work/what isn't implemented:
As the header text in the lsm9ds1-test-methods.spin source mentions, the Test Harness is more-or-less a 1:1 port of the
original C version, so only the functions used in that are implemented. There may be settings, such as
alternate data refresh rates that the IMU is capable of that aren't implemented here.

### Improvements Needed/For the future:
The calculation for temperature is very imprecise because of the fixed-point math. For example, Celsius is returned just as a whole degree (iow, there is no 68.67degC would be returned as 68 instead).

I'm unsure of how accurate the magnetometer reading is now. The original code used constants that weren't whole numbers as the resolution for a given Scale setting (see the setMagScale method). Unless I write the object to only use one specific level of precision (right now it's settable using SetPrecision(digits)), I'm not sure how else to handle these values except to chop off the decimal point and leave them whole numbers.


Productive suggestions are welcome!
