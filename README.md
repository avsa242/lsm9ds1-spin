# lsm9ds1-spin
Parallax's LSM9DS1 Simple C library ported to SPIN

This repository includes the test harness for the LSM9DS1 IMU module by Parallax, as well as code that I used to test
each method as I was porting it from C (big thanks to Dave Hein for cspin, also on github - converting this from C to SPIN
would've taken a _great_ deal longer without it).

To validate methods as I was converting them, I compared their output to the output from the same C functions from Parallax's
Simple Library version. To do this, I added a loop right after the initialization of the IMU in the Test Harness code that
simply spat out the raw or calculated values from the IMU. Once I was comfortable the values I was getting from my converted
code matched the original C version, I marked the end of the method with a comment "'WORKS"

What works/what is implemented:
I believe the Accelerometer and Gyroscope data are good, as these are reasonably easy to test.
I am less confident in the magnetometer output, only because I don't know of a bona-fide way of testing it.
The integrated temperature sensor reads within a few degrees of a multimeter I have with a thermistor, so I believe it is ok.

What doesn't work/what isn't implemented:
As the header text in the lsm9ds1-test-methods.spin source mentions, the Test Harness is more-or-less a 1:1 port of the
original C version, so only the functions used in that are implemented. There may be settings, such as
alternate data refresh rates that the IMU is capable of that aren't implemented here.

For the future:
I _would_ like to rewrite the test method source to operate more interactively. Right now, changing test
modes is a matter of changing a variable ("test_mode"). While this isn't terribly difficult, it means changing the source,
reprogramming the Propeller, testing the output, and redoing this every time a different test mode is desired, which is a
minor hassle.
