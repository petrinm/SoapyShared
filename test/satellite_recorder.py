#!/usr/bin/env python3

import SoapySDR
from SoapySDR import * # SOAPY_SDR_ constants
import numpy # use numpy for buffers
import sys, socket
import argparser


parser = argparse.ArgumentParser()
parser.add_argument("--soapy", nargs="*"
    help="Soapy arguments")
parser.add_argument("output")
parser.parse_args()



sdr = SoapySDR.Device({
    "driver": "leecher"
})


rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
sdr.activateStream(rxStream)
sdr.setSampleRate(SOAPY_SDR_RX, 0, 200e3) # 200kHz
sdr.setFrequency(SOAPY_SDR_RX, 0, 437.126)


# Create a re-usable buffer for rx samples
buff = numpy.array([0] * (16*1024), numpy.complex64)

def ads():
    pass



def run_gpredict_listener(self, callback, gpredict="127.0.0.1:4532"):
    """
        Borrowed from: https://github.com/wnagele/gr-gpredict-doppler/blob/master/python/doppler.py
    """

    bind_to = gpredict.split(":")
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((bind_to[0], int(bind_to[1])))
    server.listen(0)

    time.sleep(0.5) # TODO: Find better way to know if init is all done

    while True:
        if self.verbose:
            print("Waiting for connection on: %s:%d" % bind_to)
        sock, addr = server.accept()
        if self.verbose:
            print("Connected from: %s:%d" % (addr[0], addr[1]))

        cur_freq = 0
        while True:
            data = sock.recv(1024)
            if not data:
                break

        if data.startswith('F'):
            freq = int(data[1:].strip())
            if cur_freq != freq:
                if self.verbose:
                    print("New frequency: %d" % freq)
                self.callback(freq)
                cur_freq = freq
            sock.sendall("RPRT 0\n")
        elif data.startswith('f'):
            sock.sendall("f: %d\n" % cur_freq)

      sock.close()
      if self.verbose: print "Disconnected from: %s:%d" % (addr[0], addr[1])


new_frequency = None
def new_frequency_callback(freq):
    new_frequency = freq

t = threading.Thread(run_gpredict_listener, new_frequency_callback)

output = open(args.output, "rb")
try:
    print("Receiving...")

    # Receive some samples
    while True:

        if new_frequency:
            sdr.setFrequency(new_frequency)
            new_frequency_callback = None

        sr = sdr.readStream(rxStream, [buff], len(buff), timeoutUs=200000)
        print(sr.ret, "samples") # Num samples or error code

        #output.write(sr)

        #print(sr.flags) # Flags set by receive operation
        #print(sr.timeNs) # Timestamp for receive buffer

except:
    # Shutdown the stream
    sdr.deactivateStream(rxStream)
    sdr.closeStream(rxStream)
    raise

output.close()
