# SoapyShared

SoapyShared consists a pair of virtual SoapySDR devices drivers which can be used to share single physical device to multiple software using POSIX shared memory ring buffer.
One "master" process attaches to physical device via SoapySeeder which forwards all the command to the actual driver but at the same time copies all received data to a shared ring buffer. After this the SoapyLeecher device can created and attached to the shared memory buffer. Leecher device works like any SDR device except it cannot. Streamed data from shared buffer can be up/downconverted and decimated on fly.

So, how should I use it? For example when you want to use your SDR from multiple processes and the software itself doesn't support it. In this case you can open the device for example with GQRX via seeder driver. GQRX will show the whole band to you and bigger picture of what is happening. After this you can start developing a modem software using GNURadio tools and attach to the RX stream without loosing the waterfall on GQRX. If your SDR supports full duplex you can even transmit using the leecher device (TODO).


**This driver is still under development so random crashes and memory leaks are part of feature list!**

**The code is not the most generic implementation and ATM created to fit my requirements**


## Installation

Just like any other C++ software nowadays:

```
$ git clone git@github.com:petrinm/SoapyShared.git
$ cd SoapyShared
$ mkdir build && cd build
$ make

$ sudo make install
or
$ ln -s libSoapyShared.so /usr/lib/x86_64-linux-gnu/SoapySDR/modules0.6
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

**Note:** rx_sdr is not the best tool for this because it doesn't support CS16 input format! Decimating work atm only with CF32!
So..., this is just for demonstration purposes :P

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
