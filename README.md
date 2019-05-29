# xTasks Library

The xTasks Library provides a common interface to manage [OmpSs](https://pm.bsc.es/ompss) tasks with a FPGA target regardless the platform and the communication/back-end used.
Each platform has different supported back-ends that can be found inside the platform source folder (for example: `src/zynq`).
Please, refer to the platform README file for build instructions.

### Configuration File

xTasks Library reads the current FPGA configuration at each application launch from a formatted file.
The library looks for this file in the following locations with the same preference order:
 1. Path provided in the `XTASKS_CONFIG_FILE` environment variable.
 2. `/dev/ompss_fpga/bit_info/xtasks` (Only in Zynq boards).

##### File Format

The configuration file is expected to have the following format:
 - First line contains the headers info.
 - One line for each accelerator type with the following information separated by tabs.
   - Accelerator type identifier (interger number).
   - Number of instances of such type (positive integer number).
   - Description (string, max. 128 chars).
   - Working frequency in MHz (floating point number).
