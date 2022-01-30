CC=gcc
CXX=g++

EDCFLAGS= -I./ -O2 -Wall -std=gnu11 $(CFLAGS)
EDCXXFLAGS= -I./ -O2 -Wall -Wno-narrowing -std=gnu++11 $(CXXFLAGS)

EDLDFLAGS= -lm -lpthread $(LDFLAGS)

MAINOBJS=main.o

CPPOBJS=adafruit/Adafruit_MotorShield.o

COBJS=i2cbus/i2cbus.o \
		gpiodev/gpiodev.o

all: $(COBJS) $(CPPOBJS) $(MAINOBJS)
	$(CXX) -o main.out $(COBJS) $(CPPOBJS) $(MAINOBJS) $(EDLDFLAGS)

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<

%.o: %.cpp
	$(CXX) $(EDCXXFLAGS) -o $@ -c $<

.PHONY: clean

clean:
	rm -vf $(COBJS) $(CPPOBJS) $(MAINOBJS)