#!/usr/bin/env python3

import SoapySDR
from SoapySDR import * # SOAPY_SDR_ constants
import numpy # use numpy for buffers

sdr = SoapySDR.Device({
    "driver": "leecher"
})


txStream = sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32)
sdr.activateStream(txStream)
sdr.setSampleRate(SOAPY_SDR_TX, 0, 0.1e6)
sdr.setFrequency(SOAPY_SDR_TX, 0, 437e6)


# Create test signal
x = numpy.linspace(0, 2*3.14 * 100, 1024)
txsignal = numpy.sin(x).astype(numpy.complex64)

try:

    print("Transmitting...")

    # Transmit the test signal 10 times
    for i in range(10):
        sdr.writeStream(txStream, [txsignal], len(txsignal))

except:
    # Shutdown the stream
    sdr.deactivateStream(txStream)
    sdr.closeStream(txStream)
    raise
