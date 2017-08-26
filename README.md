# lsm9ds1-spin
## Parallax's LSM9DS1 Simple C library ported to SPIN
---

This repository includes the test harness for the LSM9DS1 IMU module by Parallax, as well as code that I used to test
each method as I was porting it from C (big thanks to Dave Hein for [cspin](http://forums.parallax.com/discussion/119342/cspin-a-c-to-spin-converter/p1) - converting this from C to SPIN would've taken a *great* deal longer without it).

To validate methods as I was converting them, I compared their output to the output from the same C functions from Parallax's
Simple Library version. To do this, I added a loop right after the initialization of the IMU in the Test Harness code that
simply spat out the raw or calculated values from the IMU. Once I was comfortable the values I was getting from my converted
code matched the original C version, I marked the end of the method with a comment `'WORKS`

### What works/what is implemented:
I believe the Accelerometer and Gyroscope data are good, as these are reasonably easy to test.
I am less confident in the magnetometer output, only because I don't know of a bona-fide way of testing it.
The integrated temperature sensor reads within a few degrees of a multimeter I have with a thermistor, so I believe it is ok.

### What doesn't work/what isn't implemented:
As the header text in the lsm9ds1-test-methods.spin source mentions, the Test Harness is more-or-less a 1:1 port of the
original C version, so only the functions used in that are implemented. There may be settings, such as
alternate data refresh rates that the IMU is capable of that aren't implemented here.

### Improvements Needed/For the future:
Like many MCUs, the Propeller lacks hardware floating-point support, and unlike C, so does the SPIN language, really.
As such, some floating-point objects were used to facilitate representing these non-whole numbers, and some of the lines used
to calculate them are quite verbose looking (i.e., difficult to read). Even though an end-user using these SPIN methods
shouldn't generally need to dissect the operation of them, I would like to investigate simplifying them
(and FWIW, the way I've implemented it may not be the greatest to begin with. I was simply trying to achieve a match to
the output of the original code).

I _would_ like to rewrite the lsm9ds1-test-methods.spin test method source to operate more interactively.  Right now,
changing test modes is a matter of changing a variable ("test_mode"). While this isn't terribly difficult, it means changing
the source, reprogramming the Propeller, testing the output, and redoing this every time a different test mode is desired,
which is a minor hassle. This source started out as just a way to vet the converted source, but I think there is value in
having a tool that simply displays live data from the IMU.

Some may notice the use of pointers for returning data from the IMU through the same (VAR) variables used to call the methods with. During the initial conversion from C, I struggled a bit with how best to return data from the methods, as I didn't think SPIN supported pointers, at least not in the same way C does, and though I understood the concept of pointers, it didn't really _click_ with me. After seeing it actually work, however, I decided I liked having it the way it is now.

Lastly, for those wondering why such a simple bit of code was placed in a git repository, especially online on github: I initialized a git repo on my computer, locally, somewhere mid-way through this whole process because I found on more than one occasion, I tried a change in the code that may've worked for what I was working on specifically at the time, but later seemed to break something, and I wasn't always sure what the cause was. Git makes it easy to "roll back" to previous versions to find out where a problem was introduced. As for putting it on github, it's to have an online backup for the code as well as an unencumbered, pretty universal way to share the code.
