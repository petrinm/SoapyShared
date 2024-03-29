# SoapyShared

[![Build Status](https://api.travis-ci.com/petrinm/SoapyShared.svg?branch=master)](https://travis-ci.com/github/petrinm/SoapyShared)

SoapyShared consists a pair of virtual SoapySDR device drivers which can be used to share single physical device to multiple software processes using shared memory ring buffer.
One "master" process attaches to physical device via SoapySeeder-driver which forwards all the command to the actual driver but at the same time copies all received data to a shared ring buffer. After creation of the ring buffer virtual SDR devices using SoapySeeder appear and can be used to attach same RX-stream with minimal overhead and latency. Leecher device works like any SDR device except it cannot control the physical device. Streamed data from shared buffer can also transparently up/downconverted and decimated.
If your SDR supports full duplex leechers can even transmit over the shared memory.

![Soapy shared concept](https://docs.google.com/drawings/d/e/2PACX-1vSbQW4phBrNhbSARNJSPidplieuvZHWbtbqygT7g5WRM7pPFH-G8X65-gh8hFSdb-U2iZUYz3AGw1vG/pub?w=1296&amp;h=648)

So, how and when should I use it?

For example when you want to abstract the physical device SDR away from the signal processing software or when you want to use your SDR from multiple processes and the software itself doesn't support it. In this case you can open the device for example with GQRX via seeder driver, setup all gains etc once. GQRX will now show the whole band to you and bigger picture of what is happening on the frequency band. After this you can start developing a signal processing software using GNURadio tools and attach to the RX stream without loosing the waterfall on GQRX. Also because the SoapyShared has abstracted all physical stuff away, there is no need to set up and initializing the hardware when new software is started.


**This driver is still under development so random crashes are part of feature list!**


## Installation

Dependencies to be installed with your favorite package manager
- `libsoapysdr-dev`
- `libliquid-dev`
- `libboost-system-dev`
- `libboost-thread-dev`

Just like any other C++ software nowadays:
```
$ git clone git@github.com:petrinm/SoapyShared.git
$ cd SoapyShared
$ mkdir build && cd build
$ cmake ..
$ make
```

Installing to the system:
```
$ sudo make install
```
OR for developing purposes
```
$ sudo ln -s $(pwd)/libSoapyShared.so /usr/local/lib/SoapySDR/modules0.7/libSoapyShared.so
```


## Example:

Open master process which controls the physical device and streams to shared memory buffer
```
$ SoapySDRUtil --find
######################################################
## Soapy SDR -- the SDR abstraction library
######################################################

Found device 0
  device = HackRF One
  driver = hackrf

Found device 1
  driver = seeder
  seeder:device = HackRF One
  seeder:driver = hackrf

$ rx_sdr -d driver=seeder,seeder:driver=hackrf -f 100M -s 4M -b 16384 -F CF32 - > /dev/null
```

After the master process has been created leecher device can be found from the device list.

```
$ SoapySDRUtil --find
######################################################
## Soapy SDR -- the SDR abstraction library
######################################################

Found device 1
  driver = leecher
  shm = /soapy

$ rx_sdr -d device=leecher -f 100M -s 4M -b 16384 -F CF32 - > /dev/null
```

**Note:** rx_sdr is not the best tool for this because it supports only CS16 input format! Decimating works atm only with CF32!
So..., this is just for demonstration purposes :P
