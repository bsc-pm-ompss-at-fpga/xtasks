CC  = gcc
CC_ = $(CROSS_COMPILE)$(CC)

CFLAGS_  = $(CFLAGS) -I../ -I../common/ -fPIC -g -Wall -O3
LDFLAGS_ = $(LDFLAGS)

## Check libxdma installation
#LIBXDMA_DIR      ?= /opt/install-arm/libxdma
LIBXDMA_INC_DIR  ?= $(LIBXDMA_DIR)/include
LIBXDMA_LIB_DIR  ?= $(LIBXDMA_DIR)/lib
LIBXDMA_INCS_     = -I$(LIBXDMA_INC_DIR)
LIBXDMA_LIBS_     = -L$(LIBXDMA_LIB_DIR) -lxdma
LIBXDMA_SUPPORT_ = $(if $(and $(wildcard $(LIBXDMA_INC_DIR)/libxdma.h ), \
	$(wildcard $(LIBXDMA_LIB_DIR)/libxdma.so )),YES,NO)

## Append needed things to CFLAGS, LDFLAGS and TARGETS
ifeq ($(LIBXDMA_SUPPORT_),YES)
	CFLAGS_  += $(LIBXDMA_INCS_)
	LDFLAGS_ += $(LIBXDMA_LIBS_)
else
    $(error No XDMA library detected)
endif

.PHONY: all
all: libxtasks-hwruntime.so read_bitinfo power_monitor

read_bitinfo: ../util/read_bitinfo/read_bitinfo_qdma.c ../util/read_bitinfo/read_bitinfo.c
	$(MAKE) -C ../util/read_bitinfo read_bitinfo_qdma
	mv ../util/read_bitinfo/read_bitinfo_qdma $@

power_monitor: ../util/power_monitor/power_monitor.c
	$(MAKE) -C ../util/power_monitor
	mv ../util/power_monitor/power_monitor $@

xtasks_server: ../util/xtasks_server/xtasks_server.c
	$(MAKE) -C ../util/xtasks_server XTASKS_INSTALL=$(PREFIX)
	mv ../util/xtasks_server/xtasks_server $@

libxtasks-hwruntime.o: libxtasks-hwruntime.c
	$(CC_) $(CFLAGS_) -c $^


libxtasks-hwruntime.so: libxtasks-hwruntime.o
	$(CC_) -shared -Wl,-rpath=$(LIBXDMA_LIB_DIR),-soname,libxtasks.so -o $@ $^ $(LDFLAGS_)

.PHONY: libxtasks_version.h
libxtasks_version.h: ../libxtasks_version_template.h
	@head -n 26 $^ >$@
ifeq (x$(shell git rev-parse --is-bare-repository 2>/dev/null), xfalse)
	@echo "/* Build commit" >>$@
	git show -s >>$@
	@echo "*/" >>$@
	@echo "#define LIBXTASKS_VERSION_COMMIT \\" >>$@
	@git show -s --format=%H >>$@
	@echo "" >>$@
	@echo "/* Build branch and status" >>$@
	git status -b -s >>$@
	@echo "*/" >>$@
else
	@echo "#define LIBXDMA_VERSION_COMMIT \\" >>$@
	@echo "unknown" >>$@
endif
	@tail -n 2 $^ >>$@

install-xtasks: libxtasks-hwruntime.so ../libxtasks.h libxtasks_version.h
	mkdir -p $(PREFIX)/lib
	cp libxtasks-*.so $(PREFIX)/lib
	ln -s $(PREFIX)/lib/libxtasks-hwruntime.so $(PREFIX)/lib/libxtasks.so || true
	mkdir -p $(PREFIX)/include
	cp ../libxtasks.h $(PREFIX)/include
	cp libxtasks_version.h $(PREFIX)/include

install-server: install-xtasks xtasks_server
	mkdir -p $(PREFIX)/bin
	cp xtasks_server $(PREFIX)/bin

install: install-xtasks install-server read_bitinfo power_monitor
	cp read_bitinfo $(PREFIX)/bin
	cp power_monitor $(PREFIX)/bin/fpga_power_monitor

.PHONY: clean
clean:
	rm -f *.o *.so libxtasks_version.h read_bitinfo power_monitor
