#!/usr/bin/env python3

import SoapySDR
from SoapySDR import * # SOAPY_SDR_ constants
import numpy # use numpy for buffers

sdr = SoapySDR.Device({
    "driver": "seeder",
    "seeder:driver": "uhd",
    "tx": "true"
})
# Note: TX doesn't work directly like this with HackRF because it's half-duplex only device!


rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
sdr.setSampleRate(SOAPY_SDR_RX, 0, 4e6)
sdr.setFrequency(SOAPY_SDR_RX, 0, 100e6) # Let's listen some FM radio stations
sdr.setGain(SOAPY_SDR_RX, 0, 70)
sdr.activateStream(rxStream)

# Create a re-usable buffer for rx samples
buff = numpy.array([0]*(16 * 1024), numpy.complex64)

try:
    # Receive some samples
    while True:
        print( sdr.readStream(rxStream, [buff], len(buff)) )

finally:
    # Shutdown the stream
    sdr.deactivateStream(rxStream)
    sdr.closeStream(rxStream)
