# xTasks Library

The xTasks Library provides a common interface to manage [OmpSs](https://pm.bsc.es/ompss) tasks with a FPGA target regardless the communication/back-end used.
The current supported back-ends are:
 - `libxtasks-stream` implements a direct communication to the HW accelerators sending the task arguments by stream.
 - `libxtasks-taskmanager` implements an asynchronous communication where the tasks are pushed into a communication buffer. The Task Manager in the FPGA reads the information and communicates with the accelerators.
 - `libxtasks-picos` implements an asynchronous communication using the [Picos](https://doi.org/10.1109/IPDPS.2017.48) queue for direct ready tasks execution.

### Build

##### Requirements

Each back-end library has different requirements that are summarized in the following table.
Moreover, the installation path of required libraries must be provided during the build stage.

|     | stream | taskmanager | picos |
| --- | :----: | :---------: | :---: |
| [libxdma](https://pm.bsc.es/gitlab/ompss-at-fpga/xdma)          | X | X |   |
| [libpicos](https://pm.bsc.es/gitlab/picos/libpicos)             |   |   | X |

##### Instructions
1. Clone the repository or download the latest stable version.
```
git clone https://pm.bsc.es/gitlab/ompss-at-fpga/xtasks
cd xtasks
```

2. (Optional) Set the needed environment variables. For example, if you are cross-compiling, set `CROSS_COMPILE`:
```
export CROSS_COMPILE=arm-linux-gnueabihf-
```
The supported variables are:
 - `CROSS_COMPILE`
 - `CFLAGS`
 - `LDFLAGS`
 - `LIBXDMA_DIR`. Installation directory of libxdma library
    - `LIBXDMA_INC_DIR`. Installation directory of includes for libxdma (Default: `$LIBXDMA_DIR/include`)
    - `LIBXDMA_LIB_DIR`. Installation directory of OS libraries for libxdma (Default: `$LIBXDMA_DIR/lib`)
 - `LIBPICOS_DIR`. Installation directory of libpicos library
    - `LIBPICOS_INC_DIR`. Installation directory of includes for libpicos (Default: `$LIBPICOS_DIR/include`)
    - `LIBPICOS_LIB_DIR`. Installation directory of OS libraries for libpicos (Default: `$LIBPICOS_DIR/lib`)

4. Build.
```
make
```
Only the supported back-ends are compiled depending on their dependencies.

5. (Optional) Install the files in `PREFIX` folder. For example:
```
make PREFIX=/opt/install-arm/libxtasks install
```
Note that the install step does not create the `libxtasks.so` in the `$PREFIX/lib` directory. You have to create it using one of the compiled back-end libraries.

### Configuration File

xTasks Library reads the current FPGA configuration at each application launch from a formated file.
The library looks for this file in the following locatations with the same preference order:
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

### Additional info
##### Execution requirements

During the execution, libxtasks uses the device files inside `/dev/ompss_fpga` folder to communicate with different elements in the fpga bitstrem and check the available features.
Those device files are created by the [OmpSs@FPGA kernel module](https://pm.bsc.es/gitlab/ompss-at-fpga/ompss-at-fpga-kernel-module) which must be loaded before the execution and the device files must be available for opening by the user.
