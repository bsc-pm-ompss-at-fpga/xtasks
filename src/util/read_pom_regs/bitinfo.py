import sys

BITINFO_MIN_VER = 13

def check_compat(bitinfo):
	ver = int.from_bytes(bitinfo[0:4], byteorder='little')
	if ver < BITINFO_MIN_VER:
		print(f'Unsupported bitinfo version {ver}, min supported version is {BITINFO_MIN_VER}')
		sys.exit(1)

def get_naccs(bitinfo):
	return int.from_bytes(bitinfo[4:8], byteorder='little')

def get_pom_base_addr(bitinfo):
	return int.from_bytes(bitinfo[92:100], byteorder='little')

def get_acc_names(naccs, bitinfo):
	XTASKS_OFFSET_OFFSET = 29*4
	SLOTS_PER_ACC = 11
	ACC_NAME_LEN = 31
	xtasks_offset = int.from_bytes(bitinfo[XTASKS_OFFSET_OFFSET+2:XTASKS_OFFSET_OFFSET+4], byteorder='little')
	xtasks_size = int.from_bytes(bitinfo[XTASKS_OFFSET_OFFSET:XTASKS_OFFSET_OFFSET+2], byteorder='little')
	nacc_types = (xtasks_size//4)//SLOTS_PER_ACC
	names = []
	for i in range(nacc_types):
		name = bitinfo[(xtasks_offset+3)*4:(xtasks_offset+3)*4+ACC_NAME_LEN].decode('ascii').rstrip(" \0")
		num_inst = int.from_bytes(bitinfo[xtasks_offset*4:xtasks_offset*4 + 2], byteorder='little')
		xtasks_offset += SLOTS_PER_ACC
		for i in range(num_inst):
			names.append(f'{name}_{i}')
	if len(names) != naccs:
		print(f'Unexpected number of names {len(names)} expected {naccs}')
		sys.exit(1)
	return names

