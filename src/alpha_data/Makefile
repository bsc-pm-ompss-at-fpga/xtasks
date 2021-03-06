CC    = gcc
CC_   = $(CC)

CFLAGS_      = $(CFLAGS) -I../ -I$(ALPHADATA_SDK)/include -std=c99 -Wall -g -fpic
LDFLAGS_     = $(LDFLAGS) -L$(ALPHADATA_SDK)/lib -ladmxrc3
TARGETS_     =

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
	mkdir -p $(PREFIX)/include
	ln -s $(PREFIX)/lib/libxtasks-hwruntime.so $(PREFIX)/lib/libxtasks.so || true
	cp ../libxtasks.h $(PREFIX)/include
	cp libxtasks_version.h $(PREFIX)/include

.PHONY: clean
clean:
	rm -f *.o *.so libxtasks_version.h
