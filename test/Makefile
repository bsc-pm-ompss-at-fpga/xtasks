CC    = gcc
CC_   = $(CROSS_COMPILE)$(CC)

CFLAGS_      = $(CFLAGS) -O3 -std=c99 -Wall -Werror -g -fpic
LDFLAGS_     = $(LDFLAGS)

## Check xtasks installation
LIBXTASKS_DIR      ?= /opt/install-arm/libxtasks
LIBXTASKS_INC_DIR  ?= $(LIBXTASKS_DIR)/include
LIBXTASKS_LIB_DIR  ?= $(LIBXTASKS_DIR)/lib
LIBXTASKS_SUPPORT_ = $(if $(and $(wildcard $(LIBXTASKS_INC_DIR)/libxtasks.h ), \
	$(wildcard $(LIBXTASKS_LIB_DIR)/libxtasks.so )),YES,NO)

## Append needed things to CFLAGS and LDFLAGS
ifeq ($(LIBXTASKS_SUPPORT_),YES)
	CFLAGS_  += -I$(LIBXTASKS_INC_DIR)
	LDFLAGS_ += -L$(LIBXTASKS_LIB_DIR) -lxtasks
endif

TARGETS = t01.test

all: $(TARGETS)

t%.test: t%.c
	$(CC_) $(CFLAGS_) $^ -o $@ -Wl,-rpath=$(LIBXTASKS_LIB_DIR) $(LDFLAGS_)

clean:
	rm -f *.o $(TARGETS)
