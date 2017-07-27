CC    = gcc
CC_   = $(CROSS_COMPILE)$(CC)

CFLAGS_      = $(CFLAGS) -O3 -std=c99 -Wall -Werror -g -fpic
LDFLAGS_     = $(LDFLAGS)
TARGETS_     =

## Check libxdma installation
LIBXDMA_DIR      ?= /opt/install-arm/libxdma
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
	CFLAGS_  +=
	LDFLAGS_ +=
	TARGETS_ += libxtasks-stream.so libxtasks-taskmanager.so
endif
ifeq ($(LIBPICOS_SUPPORT_),YES)
	CFLAGS_  +=
	LDFLAGS_ +=
	TARGETS_ += libxtasks-picos.so
endif

.PHONY: all
all: $(TARGETS_)

libxtasks-taskmanager.o: ./src/libxtasks-taskmanager.c
	$(CC_) $(CFLAGS_) $(LIBXDMA_INCS_) -c $^

libxtasks-taskmanager.so: libxtasks-taskmanager.o
	$(CC_) -shared -Wl,-rpath=$(LIBXDMA_LIB_DIR),-soname,$@ -o $@ $^ $(LDFLAGS_) $(LIBXDMA_LIBS_)

libxtasks-stream.o: ./src/libxtasks-stream.c
	$(CC_) $(CFLAGS_) $(LIBXDMA_INCS_) -c $^

libxtasks-stream.so: libxtasks-stream.o
	$(CC_) -shared -Wl,-rpath=$(LIBXDMA_LIB_DIR),-soname,$@ -o $@ $^ $(LDFLAGS_) $(LIBXDMA_LIBS_)

libxtasks-picos.o: ./src/libxtasks-picos.c
	$(CC_) $(CFLAGS_) $(LIBPICOS_INCS_) -c $^

libxtasks-picos.so: libxtasks-picos.o
	$(CC_) -shared -Wl,-rpath=$(LIBPICOS_LIB_DIR),-soname,$@ -o $@ $^ $(LDFLAGS_) $(LIBPICOS_LIBS_)

.PHONY: libxtasks_version.h
libxtasks_version.h:
	@echo "#ifndef __LIBXTASKS_VERSION_H__" >libxtasks_version.h
	@echo "#define __LIBXTASKS_VERSION_H__" >>libxtasks_version.h
	@echo "" >>libxtasks_version.h
	@echo "/* Build commit" >>libxtasks_version.h
	git show -s >>libxtasks_version.h
	@echo "*/" >>libxtasks_version.h
	@echo "#define XTASKS_COMMIT_INFO \\" >>libxtasks_version.h
	@git show -s --format=%H >>libxtasks_version.h
	@echo "" >>libxtasks_version.h
	@echo "/* Build branch and status" >>libxtasks_version.h
	git status -b -s >>libxtasks_version.h
	@echo "*/" >>libxtasks_version.h
	@echo "" >>libxtasks_version.h
	@echo "#endif" >>libxtasks_version.h

install: $(TARGETS_) ./src/libxtasks.h libxtasks_version.h
	mkdir -p $(PREFIX)/lib
	cp libxtasks-*.so $(PREFIX)/lib
	mkdir -p $(PREFIX)/include
	cp ./src/libxtasks.h $(PREFIX)/include
	cp ./libxtasks_version.h $(PREFIX)/include
	@echo "====================================== NOTE ======================================"
	@echo "Remember to create the symlink $(PREFIX)/lib/libxtasks.so to your desired backend!"
	@echo "For example: pushd $(PREFIX)/lib; ln -s libxtasks-stream.so libxtasks.so; popd;"
	@echo "=================================================================================="

.PHONY: clean
clean:
	rm -f *.o *.so *.a libxtasks_version.h
