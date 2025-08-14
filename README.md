<p align="center">
<img src="https://signalhound.com/sigdownloads/Other/SH-SOAPY.jpg" width="75%" />
</p>

# A [SoapySDR](https://github.com/pothosware/SoapySDR/wiki) driver for the [Signal Hound BB60D 6 GHz Real-Time Spectrum Analyzers](https://signalhound.com/products/bb60d-6-ghz-real-time-spectrum-analyzer/) and [Signal Hound BB60C 6 GHz Real-Time Spectrum Analyzers](https://signalhound.com/products/bb60c/)
## System Requirements

- 64-bit Linux operating system
    - Tested on DragonOS Noble 6.14.0-27-generic
- Native USB 3.0 support

## Dependancies

1. Libusb 1.0 
    ~~~
    $ sudo apt-get update
    $ sudo apt-get install libusb-1.0-0
    ~~~
    Note: libusb requires root by default to change this add the following to etc/udev/rules.d/sh.rules
    or download [Spike](https://signalhound.com/spike) and the setup will be done for you
    ~~~
    SUBSYSTEM=="usb", ATTR{idVendor}=="2817", MODE="0666", GROUP="plugdev"
    ~~~
2. FTDI D2XX library
- grab latest 64-bit Linux driver from [FTDI Chip](www.ftdichip.com/Drivers/D2XX.htm) 
- or find a copy of the library in the [Signal Hound SDK](https://signalhound.com/software/signal-hound-software-development-kit-sdk/) 
    - standard path device_apis/bb_series/lib/linux/Ubuntu 18.04/libftd2xx.so
        ~~~
        $ sudo cp libftd2xx* /usr/local/lib
        $ cd /usr/local/lib
        $ sudo chmod 0755 libftd2xx.so
        ~~~
3. BB60 API
- grab shared object file from [Signal Hound SDK](https://signalhound.com/software/signal-hound-software-development-kit-sdk/) 
    ~~~
    $ cd device_apis/bb_series/lib/linux/Ubuntu 18.04
    $ sudo cp libbb_api.* /usr/local/lib
    $ sudo ldconfig -v -n /usr/local/lib
    $ sudo ln -sf /usr/local/lib/libbb_api.so.5 /usr/local/lib/libbb_api.so
    ~~~
4. [SoapySDR](https://github.com/pothosware/PothosCore/wiki/Ubuntu).

## Installation

1. Clone this repository.
2. Run the following commands from the cloned repo:
    ~~~
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ sudo make install
    $ sudo ldconfig
    ~~~
Note: MATLAB runtime LD_LIBRARY_PATH environment variable may cause conflicts with cmake

Note: if a BB60D or BB60C device is plugged in, `SoapySDRUtil --find` will display its serial number.

## Usage

- `#include <SoapySDR/Device.hpp>` and use the functions in [Device.hpp](https://github.com/pothosware/SoapySDR/blob/master/include/SoapySDR/Device.hpp) to interface with the BB60D or BB60C.

- Use with [other platforms](https://github.com/pothosware/SoapySDR/wiki#platforms) that are compatible with SoapySDR such as [GNU Radio Companion](https://www.gnuradio.org/), [SDRangel](https://www.sdrangel.org/), and many others.
