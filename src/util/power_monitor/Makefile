.PHONY: clean all

all: power_monitor

power_monitor: power_monitor.c
	$(CC) $^  -I../../common -I../../ -o $@

clean:
	rm power_monitor
