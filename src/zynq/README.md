# xTasks back-ends for Zynq platforms

The current supported back-ends for Zynq platforms are:
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
cd xtasks/src/zynq
```

2. (Optional) Set the needed environment variables. For example, if you are cross-compiling, set `CROSS_COMPILE`:
```
export CROSS_COMPILE=arm-linux-gnueabihf-
```
The supported variables are:
 - `CROSS_COMPILE`
 - `CFLAGS`
    - Adding the definition of `XTASKS_DEBUG` processor variable puts the library in debug mode which includes sanity checks and additional debug methods.
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

### Execution

##### Requirements

During the execution, libxtasks uses the device files inside `/dev/ompss_fpga` folder to communicate with different elements in the fpga bitstream and check the available features.
Those device files are created by the [OmpSs@FPGA kernel module](https://pm.bsc.es/gitlab/ompss-at-fpga/ompss-at-fpga-kernel-module) which must be loaded before the execution and the device files must be available for opening by the user.

##### Environment variables

 - `XTASKS_CONFIG_FILE`. Defines the path of xtasks config file to use during library initialization. By default, the path is `/dev/ompss_fpga/bit_info/xtasks`.
 - `XTASKS_RESET_POLARITY` Defines the reset polarity of Task Manager (and accelerators) in the bitstream. By default, the polarity is `1`.
 - `XTASKS_FEATURES_CHECK` Defines whether the features checking in `/dev/ompss_fpga/bit_info/features/` must be done or not. By default, the checking is done.