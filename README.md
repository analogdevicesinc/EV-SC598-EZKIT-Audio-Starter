# EV-SC598-EZKIT

## Overview

This project contains a comprehensive BSP for the EV-SC598-EZKIT SoM plus
carrier board ez-kit.

For full details about this project, 
features and set up instructions, please refer to: https://wiki.analog.com/resources/tools-software/sharc-audio-module/advanced-audio-projects

## To build

### Install the tools

- Install CCES as-per Analog Devices instructions (3.0.2 CCES Version Minimum Required!)
- Install Git bash shell from here: https://git-scm.com/downloads
- Instead of Git bash, one can also install MinGW/MSYS2 from
  here: http://www.msys2.org/

### Source the environment

- Check your path first to see if it already includes the desired version of
  CCES.  In general Git bash inherits the path and MinGW/MSYS2 does not
  unless it's launched with '-use-full-path' option.

```
echo $PATH  # Check inherited path
```

- If necessary, modify the 'env.sh' shell to match your CCES installation
  directory and include the environment in your shell.

```
. ./env.sh # Note the space between the first two periods!
```

### Build the code

- Go into the build directory and type `make`

```
cd build
make      # Add -j4 for a faster build on multi-core machines
```

## Debugging the code

- Open CCES, create a new debug configuration
- Load `build/ezkitSC598W_preload_core0` into core0
- Also load the `EV-SC598-EZKIT-ARM.exe` executable into core0
- Load `EV-SC598-EZKIT-SHARC0.dxe` into core1
- Load `EV-SC598-EZKIT-SHARC1.dxe` into core2
- In the `Automatic Breakpoints` tab, uncheck `Enable Semihosting`
- Save and start debugging.

## Initial flashing of the code using the ADI's Serial Flash Programmer

- Download ADI Serial Flash Programmer: https://www.analog.com/en/resources/evaluation-hardware-and-software/embedded-development-software/crosscore-utilities.html.
- Plug in the USB-C from PC to the connector on the SOM.
- Select ADSP-SC598-SOM as target. 
- Set the blue rotary switch on the ez-kit to position 3 and reset the
  board
- Program the board using the serial flash programmer
- Set the blue rotary switch back to one and reset the board
- Additional details/instructions can be found in the Wiki

