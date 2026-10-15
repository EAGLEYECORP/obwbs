CC      ?= cc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion -Werror
CPPFLAGS = -Iinclude -Isim
LDLIBS   = -lm

CORE = src/wb_core.c src/wb_config.c src/wb_file.c
SIM  = sim/sim.c

all: build/wbsim build/test_wb

build:
	mkdir -p build

build/wbsim: $(CORE) $(SIM) sim/wbsim.c include/wb.h sim/sim.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(CORE) $(SIM) sim/wbsim.c $(LDLIBS)

build/test_wb: $(CORE) $(SIM) tests/test_wb.c include/wb.h sim/sim.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(CORE) $(SIM) tests/test_wb.c $(LDLIBS)

test: build/test_wb
	./build/test_wb

demo: build/wbsim
	./build/wbsim configs/types/NB180.acf configs/tails/DEMO-NB180.cal --zfw 59500 --zfw-cg 24 --fuel 9800 --trip 6400 --pitch 0.8 --loadsheet 59200 --loadsheet-cg 23.8

clean:
	rm -rf build

.PHONY: all test demo clean
