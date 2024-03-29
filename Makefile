# lsm9ds1-spin Makefile - requires GNU Make, or compatible
# Variables below can be overridden on the command line
#	e.g. make IFACE=LSM9DS1_SPI

# P1, P2 device nodes and baudrates
#P1DEV=
P1BAUD=115200
#P2DEV=
P2BAUD=2000000

# P1, P2 compilers
P1BUILD=flexspin --interp=rom
P2BUILD=flexspin -2

# LSM9DS1 interface: I2C or SPI
IFACE=LSM9DS1_I2C
#IFACE=LSM9DS1_SPI

# Paths to spin-standard-library, and p2-spin-standard-library,
#  if not specified externally
SPIN1_LIB_PATH=-L ../spin-standard-library/library
SPIN2_LIB_PATH=-L ../p2-spin-standard-library/library


# -- Internal --
SPIN1_DRIVER_FN=sensor.imu.9dof.lsm9ds1.spin
SPIN2_DRIVER_FN=sensor.imu.9dof.lsm9ds1.spin2
CORE_FN=core.con.lsm9ds1.spin
# --

# Build all targets (build only)
all: LSM9DS1-Demo.binary LSM9DS1-Demo.bin2

# Load P1 or P2 target (will build first, if necessary)
p1demo: loadp1demo
p2demo: loadp2demo

# Build binaries
LSM9DS1-Demo.binary: LSM9DS1-Demo.spin $(SPIN1_DRIVER_FN) $(CORE_FN)
	$(P1BUILD) $(SPIN1_LIB_PATH) -b -D $(IFACE) LSM9DS1-Demo.spin

LSM9DS1-Demo.bin2: LSM9DS1-Demo.spin2 $(SPIN2_DRIVER_FN) $(CORE_FN)
	$(P2BUILD) $(SPIN2_LIB_PATH) -b -2 -D $(IFACE) -o LSM9DS1-Demo.bin2 LSM9DS1-Demo.spin2

# Load binaries to RAM (will build first, if necessary)
loadp1demo: LSM9DS1-Demo.binary
	proploader -t -p $(P1DEV) -Dbaudrate=$(P1BAUD) LSM9DS1-Demo.binary

loadp2demo: LSM9DS1-Demo.bin2
	loadp2 -SINGLE -p $(P2DEV) -v -b$(P2BAUD) -l$(P2BAUD) LSM9DS1-Demo.bin2 -t

# Remove built binaries and assembler outputs
clean:
	rm -fv *.binary *.bin2 *.pasm *.p2asm

