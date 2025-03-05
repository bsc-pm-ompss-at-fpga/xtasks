import bitinfo

regs_pom = {
	'regs_info': [
		{'name': 'COPY_IN_OPT' , 'size': 4},
		{'name': 'COPY_OUT_OPT', 'size': 4},
		{'name': 'ACC_AVAIL'   , 'size': 8},
		{'name': 'QUEUE_NEMPTY', 'size': 8}
	 ]
}

mmap_fpgas = []

def complete_regs_pom(bitinfo_data):
	bitinfo.check_compat(bitinfo_data)
	naccs = bitinfo.get_naccs(bitinfo_data)
	names = bitinfo.get_acc_names(naccs, bitinfo_data)
	regs_pom['base_addr'] = bitinfo.get_pom_base_addr(bitinfo_data)

	for i, name in enumerate(names):
		regs_pom['regs_info'].append({'name': f'CMD_IN_N_CMDS_{name}', 'size': 4, 'addr': 0x800 + i*4})
	for i, name in enumerate(names):
		regs_pom['regs_info'].append({'name': f'CMD_OUT_N_CMDS_{name}', 'size': 4, 'addr': 0x900 + i*4})
	for i, name in enumerate(names):
		regs_pom['regs_info'].append({'name': f'AVAIL_COUNT_{name}', 'size': 8, 'addr': 0xA00 + i*8})

def read_regs(regs, vals):
	for mmap_fpga in mmap_fpgas:
		base_addr = regs['base_addr']
		axilite_addr = regs['base_addr']
		for reg in regs['regs_info']:
			if 'addr' in reg:
				axilite_addr = base_addr + reg['addr']
			vals.extend(mmap_fpga[axilite_addr:axilite_addr+reg['size']])
			axilite_addr += reg['size']

	return vals

def print_regs(regs, vals, n, csv):
	vals_idx = 0
	if csv:
		header = 'id'
		for reg in regs['regs_info']:
			header += ',{}'.format(reg['name'])
		print(header)

		for idx in range(n):
			row = str(idx)
			for reg in regs['regs_info']:
				row += ',{}'.format(int.from_bytes(vals[vals_idx:vals_idx+reg['size']], byteorder='little'))
				vals_idx += reg['size']
			print(row)
	else:
		for idx in range(n):
			print(f'-------- Idx {idx} --------')
			for reg in regs['regs_info']:
				val_int = int.from_bytes(vals[vals_idx:vals_idx+reg['size']], byteorder='little')
				vals_idx += reg['size']
				print(f"{reg['name']}: {val_int}")



