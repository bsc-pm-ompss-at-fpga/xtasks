.PHONY: clean all

CC  = gcc
CC_ = $(CROSS_COMPILE)$(CC)

all: power_monitor

power_monitor: power_monitor.c
	$(CC_) $^  -I../../common -I../../ -o $@

clean:
	rm power_monitor
