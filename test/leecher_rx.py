#!/usr/bin/env python3

import SoapySDR
from SoapySDR import * # SOAPY_SDR_ constants
import numpy # use numpy for buffers
import sys


sdr = SoapySDR.Device({
    "driver": "leecher"
})


rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
sdr.activateStream(rxStream)
sdr.setSampleRate(SOAPY_SDR_RX, 0, 0.2e6)
sdr.setFrequency(SOAPY_SDR_RX, 0, 101.1e6) # Local FM radio station :D


# Create a re-usable buffer for rx samples
buff = numpy.array([0] * (16*1024), numpy.complex64)

try:
    print("Receiving...")

    # Receive some samples
    while True:
        sr = sdr.readStream(rxStream, [buff], len(buff), timeoutUs=200000)
        print(sr.ret, "samples") # Num samples or error code
        #print(sr.flags) # Flags set by receive operation
        #print(sr.timeNs) # Timestamp for receive buffer

except:
    # Shutdown the stream
    sdr.deactivateStream(rxStream)
    sdr.closeStream(rxStream)
    raise
