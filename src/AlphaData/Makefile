CC    = gcc
CC_   = $(CC)

CFLAGS_      = $(CFLAGS) -I../ -I$(ALPHADATA_SDK)/include -O3 -std=c99 -Wall -g -fpic
LDFLAGS_     = $(LDFLAGS) -ladmxrc3
TARGETS_     =

## Check libxdma installation
#LIBXDMA_DIR      ?= /opt/install-arm/libxdma
LIBXDMA_INC_DIR  ?= $(LIBXDMA_DIR)/include
LIBXDMA_LIB_DIR  ?= $(LIBXDMA_DIR)/lib
LIBXDMA_INCS_     = -I$(LIBXDMA_INC_DIR)
LIBXDMA_LIBS_     = -L$(LIBXDMA_LIB_DIR) -lxdma
LIBXDMA_SUPPORT_ = $(if $(and $(wildcard $(LIBXDMA_INC_DIR)/libxdma.h ), \
	$(wildcard $(LIBXDMA_LIB_DIR)/libxdma.so )),YES,NO)

## Check libpicos installation
#LIBPICOS_DIR      ?= /opt/install-arm/libpicos
LIBPICOS_INC_DIR  ?= $(LIBPICOS_DIR)/include
LIBPICOS_LIB_DIR  ?= $(LIBPICOS_DIR)/lib
LIBPICOS_INCS_     = -I$(LIBPICOS_INC_DIR)
LIBPICOS_LIBS_     = -L$(LIBPICOS_LIB_DIR) -lpicos
LIBPICOS_SUPPORT_ = $(if $(and $(wildcard $(LIBPICOS_INC_DIR)/libpicos.h ), \
	$(wildcard $(LIBPICOS_LIB_DIR)/libpicos.so )),YES,NO)

## Append needed things to CFLAGS, LDFLAGS and TARGETS
ifeq ($(LIBXDMA_SUPPORT_),YES)
	CFLAGS_  += $(LIBXDMA_INCS_)
	LDFLAGS_ += $(LIBXDMA_LIBS_)
	TARGETS_ += libxtasks-taskmanager.so

	ifeq ($(LIBPICOS_SUPPORT_),YES)
		CFLAGS_  +=
		LDFLAGS_ +=
		TARGETS_ +=
	endif
endif

.PHONY: all
all: $(TARGETS_)

libxtasks-taskmanager.o: libxtasks-taskmanager.c
	$(CC_) $(CFLAGS_) -c $^

libxtasks-taskmanager.so: libxtasks-taskmanager.o
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
	cp ../libxtasks.h $(PREFIX)/include
	cp libxtasks_version.h $(PREFIX)/include
	@echo "========================= NOTE ========================="
	@echo "== Remember to create the symlink $(PREFIX)/lib/libxtasks.so to your desired backend!"
	@echo "== For example:"
	@echo "==   pushd $(PREFIX)/lib; ln -s libxtasks-taskmanager.so libxtasks.so; popd;"
	@echo "========================================================"

.PHONY: clean
clean:
	rm -f *.o *.so libxtasks_version.h
