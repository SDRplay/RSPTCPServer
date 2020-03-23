# rsp_tcp

This is a fork of the original rsp_tcp by F4FHH Nicolas with extensions to support all current RSP devices. Use the extended mode to access the full bit rate of the RSP and all of the RSP specific controls. By default the server supports the RTL TCP protocol.

February 2019: Added support for SDRplay API 3.07 (support all RSPs including RSPdx)

(c)2018 F4FHH Nicolas (f4fhh@ducor.fr). Licensed under the GNU GPL V3

## a rtl_tcp compatible IQ server for the RSP range of SDRPlay SDR

rsp_tcp is a direct port of [rtl_tcp](https://github.com/osmocom/rtl-sdr) for the RSP range of [SDRPlay SDR](https://www.sdrplay.com/).

As the rtl_tcp protocol is only 8 bits IQ, man will loose the major advantage of an RSP : its 14bits ADC, but :

1. It will work with any rtl_tcp capable frontend (probably), see usage below
2. As it's opensource, you could compile it on any Linux server

## OPTIONS
```
 -a listen address
 -p listen port (default: 1234)
 -d RSP device to use (default: 1, first found)
 -P Antenna Port select (0/1/2, default: 0, Port A)
 -T Bias-T enable (default: disabled)
 -N Broadcast Notch enable (default: disabled)
 -R Refclk output enable (default: disabled)
 -f frequency to tune to [Hz]
 -s samplerate in Hz (default: 2048000 Hz)
 -n max number of linked list buffers to keep (default: 500)
 -v Verbose output (debug) enable (default: disabled)
 -E extended mode full RSP bit rate and controls (default: RTL mode)
```
## USAGE
 - RTL Tuner AGC is mapped to RSP RF AGC
 - RTL AGC is mapped to LNAState (RTL AGC on = LNA enabled)
 - RTL RF gain is mapped to inverse gain reduction
 - RTL frequency correction is mapped to RSP setPPM
 - RTL sample rates >= 2Ms/s are mapped to the RSP sample rate, RTL sample rates < 2Ms/s use appropriate decimation

## BUILDING
```
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
```
## NOTES
 - a RSP API version >=2.13 must be installed on the linux server, see [sdrplay linux downloads](https://www.sdrplay.com/downloads/)
 - I try it with [SDR#](https://airspy.com/download/) frontend only. Other tests are welcome.
 - It should compile and run on Raspbian (raspberry pi) (not tested)
 - It should compile on windows as the initial code from rtl_tcp does

## TODO
 - Enhance the IF and RF gain management depending on bands
 - Enhance the re-quantization from 14/12/10 bits to 8 bits

## HISTORY
 - Version 0.1.0: Initial build

## CREDITS
 - [Open Source Mobile Communications (OSMOCOM)](https://github.com/osmocom/rtl-sdr.git) team for the original rtl_tcp code
 - [Thierry Leconte](https://github.com/TLeconte/airspy_tcp.git) for many ideas that I found in his Airspy port of rtl_tcp
 - [Tony Hoyle](https://github.com/TonyHoyle/sdrplay.git) for the initial idea
 - [Pothosware](https://github.com/pothosware) for the cmake build examples
