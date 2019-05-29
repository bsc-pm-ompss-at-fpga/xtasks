CC    = gcc
CC_   = $(CROSS_COMPILE)$(CC)

CFLAGS_      = $(CFLAGS) -O3 -std=c99 -Wall -Werror -g -fpic
LDFLAGS_     = $(LDFLAGS)
TARGETS_     = libxtasks-stub.so

.PHONY: all
all: $(TARGETS_)

libxtasks.o: libxtasks.c
	$(CC_) $(CFLAGS_) -c $^

libxtasks-stub.so: libxtasks.o
	$(CC_) -shared -Wl,-rpath=$(LIBXDMA_LIB_DIR),-soname,libxtasks.so -o $@ $^ $(LDFLAGS_)

.PHONY: libxtasks_version.h
libxtasks_version.h: libxtasks_version_template.h
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

install: $(TARGETS_) libxtasks.h libxtasks_version.h
	mkdir -p $(PREFIX)/lib
	cp libxtasks-*.so $(PREFIX)/lib
	mkdir -p $(PREFIX)/include
	cp libxtasks.h $(PREFIX)/include
	cp libxtasks_version.h $(PREFIX)/include

.PHONY: clean
clean:
	rm -f *.o *.so libxtasks_version.h
