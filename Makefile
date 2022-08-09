CC=gcc
CXX=g++
PWD=$(shell pwd)
EDCFLAGS= -I./ -O2 -Wall -std=gnu11 $(CFLAGS)
EDCXXFLAGS= -I./ -I clkgen/include -O2 -Wall -Wno-narrowing -std=gnu++11 $(CXXFLAGS)

EDLDFLAGS= -lm -lpthread $(LDFLAGS)

CPPOBJS=Adafruit/MotorShield.o
EXAMPLESRCS=$(wildcard examples/*.cpp)
EXAMPLEOBJS=$(EXAMPLESRCS:.cpp=.o)

LIBCLKGEN = clkgen/libclkgen.a

COBJS=i2cbus/i2cbus.o

all: $(COBJS) $(CPPOBJS) $(EXAMPLEOBJS) $(LIBCLKGEN)
	for obj in $(EXAMPLEOBJS); do \
		bin=`echo $$obj | sed 's/\.o/\.out/' | sed 's/examples//'`; \
		$(CXX) -o $(PWD)/$$bin $(COBJS) $(CPPOBJS) $$obj $(LIBCLKGEN) $(EDLDFLAGS); \
	done

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<

%.o: %.cpp
	$(CXX) $(EDCXXFLAGS) -o $@ -c $<

$(LIBCLKGEN):
	cd clkgen && make && cd ..

.PHONY: clean doc

doc:
	doxygen .doxyconfig

clean:
	rm -vf $(COBJS) $(CPPOBJS) $(EXAMPLEOBJS)
	rm -vf *.out
	cd clkgen && make clean && cd ..

spotless: clean
	rm -vrf doc