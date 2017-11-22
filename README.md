# xTasks Library

The xTasks Library provides a common interface to manage [OmpSs](https://pm.bsc.es/ompss) tasks with a FPGA target regardless the communication/back-end used.
The current supported back-ends are:
 - `libxtasks-stream` implements a direct communication to the HW accelerators sending the task arguments by stream.
 - `libxtasks-taskmanager` implements an asynchronous communication where the tasks are pushed into a communication buffer. The Task Manager in the FPGA reads the information and communicates with the accelerators.
 - `libxtasks-picos` implements an asynchronous communication using the  [Picos](https://doi.org/10.1109/IPDPS.2017.48) queue for direct ready tasks execution.

### Build

##### Requirements

Each back-end library has different requirements that are summarized in the following table.
Moreover, the installation path of some required libraries must be provided during the build stage.

|     | stream | taskmanager | picos |
| --- | :----: | :---------: | :---: |
| [libxdma](https://pm.bsc.es/gitlab/ompss-at-fpga/xdma) (version >= 1.0) | X | X |   |
| [libpicos](https://pm.bsc.es/gitlab/jbosch/picos) (minimum commit [`3189090d`](https://pm.bsc.es/gitlab/jbosch/picos/commit/3189090d6513932fd530856ae4d3aac871a604f4)) |   |   | X |
| [autoVivado](https://pm.bsc.es/gitlab/ompss-at-fpga/autoVivado) (bitstream generated using minimum commit [`ea99f20e`](https://pm.bsc.es/gitlab/ompss-at-fpga/autoVivado/commit/ea99f20e112f1c6721ac6767823f84bc78bb66c8)) |   | X |   |

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
