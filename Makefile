CC=gcc
CXX=g++

EDCFLAGS= -I./ -O2 -Wall -std=gnu11 $(CFLAGS)
EDCXXFLAGS= -I./ -O2 -Wall -Wno-narrowing -std=gnu++11 $(CXXFLAGS)

EDLDFLAGS= -lm -lpthread $(LDFLAGS)

CPPOBJS=adafruit/Adafruit_MotorShield.o
MOTORSHIELDTEST=examples/motorshield.o

COBJS=i2cbus/i2cbus.o \
		gpiodev/gpiodev.o

motor: $(COBJS) $(CPPOBJS) $(MOTORSHIELDTEST)
	$(CXX) -o $@.out $(COBJS) $(CPPOBJS) $(MOTORSHIELDTEST) $(EDLDFLAGS)

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<

%.o: %.cpp
	$(CXX) $(EDCXXFLAGS) -o $@ -c $<

.PHONY: clean

clean:
	rm -vf $(COBJS) $(CPPOBJS) $(MOTORSHIELDTEST)
	rm -vf *.out