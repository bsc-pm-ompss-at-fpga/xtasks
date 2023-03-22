.PHONY: clean all

all: read_bitinfo_qdma read_bitinfo_zynq read_bitinfo_coe

read_bitinfo_qdma: read_bitinfo_qdma.c read_bitinfo.c ../util/bitinfo.h
	gcc read_bitinfo_qdma.c read_bitinfo.c -o $@

read_bitinfo_zynq: read_bitinfo_zynq.c read_bitinfo.c ../util/bitinfo.h
	gcc read_bitinfo_zynq.c read_bitinfo.c -o $@

read_bitinfo_coe: read_bitinfo_coe.c read_bitinfo.c ../util/bitinfo.h
	gcc read_bitinfo_coe.c read_bitinfo.c -o $@

clean:
	rm read_bitinfo_qdma read_bitinfo_zynq read_bitinfo_coe
