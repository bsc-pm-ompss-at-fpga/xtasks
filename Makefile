CC    = gcc
CC_   = $(CROSS_COMPILE)$(CC)

CFLAGS_      = $(CFLAGS) -O3 -std=c99 -Wall -Werror -g -fpic
LDFLAGS_     = $(LDFLAGS)

## Check libxdma installation
LIBXDMA_DIR      ?= /opt/install-arm/libxdma
LIBXDMA_INC_DIR  ?= $(LIBXDMA_DIR)/include
LIBXDMA_LIB_DIR  ?= $(LIBXDMA_DIR)/lib
LIBXDMA_SUPPORT_ = $(if $(and $(wildcard $(LIBXDMA_INC_DIR)/libxdma.h ), \
	$(wildcard $(LIBXDMA_LIB_DIR)/libxdma.so )),YES,NO)

## Append needed things to CFLAGS and LDFLAGS
ifeq ($(LIBXDMA_SUPPORT_),YES)
	CFLAGS_  += -I$(LIBXDMA_INC_DIR)
	LDFLAGS_ += -L$(LIBXDMA_LIB_DIR) -lxdma
endif

.PHONY: all
all: libxtasks.so

libxtasks.o: ./src/libxtasks.c
	$(CC_) $(CFLAGS_) -c $^

libxtasks.so: libxtasks.o
	$(CC_) -shared -Wl,-rpath=$(LIBXDMA_LIB_DIR),-soname,$@ -o $@ $^ $(LDFLAGS_)

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

install: libxtasks.so ./src/libxtasks.h libxtasks_version.h
	mkdir -p $(PREFIX)/lib
	cp libxtasks.so $(PREFIX)/lib
	mkdir -p $(PREFIX)/include
	cp ./src/libxtasks.h $(PREFIX)/include
	cp ./libxtasks_version.h $(PREFIX)/include

.PHONY: clean
clean:
	rm -f *.o *.so *.a
