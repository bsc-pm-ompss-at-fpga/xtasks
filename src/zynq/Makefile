CC  = gcc
CC_ = $(CROSS_COMPILE)$(CC)

CFLAGS_  = $(CFLAGS) -O3 -std=gnu99 -Wall -g -fpic -Werror -Wno-unused-result
LDFLAGS_ = $(LDFLAGS)
TARGETS_ = read_bitinfo

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
	TARGETS_ += libxtasks-hwruntime.so
endif

.PHONY: all
all: $(TARGETS_)

read_bitinfo: ../util/read_bitinfo/read_bitinfo_zynq.c ../util/read_bitinfo/read_bitinfo.c
	$(MAKE) -C ../util/read_bitinfo/ read_bitinfo_zynq
	mv ../util/read_bitinfo/read_bitinfo_zynq $@

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

install: $(TARGETS_) ../libxtasks.h libxtasks_version.h
	mkdir -p $(PREFIX)/lib
	cp libxtasks-*.so $(PREFIX)/lib
	ln -s $(PREFIX)/lib/libxtasks-hwruntime.so $(PREFIX)/lib/libxtasks.so || true
	mkdir -p $(PREFIX)/include
	cp ../libxtasks.h $(PREFIX)/include
	cp libxtasks_version.h $(PREFIX)/include
	mkdir -p $(PREFIX)/bin
	cp read_bitinfo $(PREFIX)/bin

.PHONY: clean
clean:
	rm -f *.o *.so libxtasks_version.h read_bitinfo
