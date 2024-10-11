
xtasks_server: xtasks_server.c
	gcc $< -o $@ -Wall -O3 -I$(XTASKS_INSTALL)/include -L$(XTASKS_INSTALL)/lib -lxtasks -Wl,-rpath=$(XTASKS_INSTALL)/lib

install: xtasks_server
	cp xtasks_server $(PREFIX)/bin

