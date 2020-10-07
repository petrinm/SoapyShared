#!/usr/bin/env python3

import SoapySDR
from SoapySDR import * # SOAPY_SDR_ constants
import numpy as np # use numpy for buffers
import sys, socket, threading
import argparse


parser = argparse.ArgumentParser()
parser.add_argument("--device", "-D", help="Soapy arguments", default="")
parser.add_argument("--frequency", "-f", help="Initial center frequency", default="437M")
parser.add_argument("--sample_rate", "-s", help="Sample rate", default="60k")
parser.add_argument("output_file")
args = parser.parse_args()


def parse_postfix(val):
    if val.endswith("k"):
        return 1e3 * float(val[:-1])
    elif val.endswith("M"):
        return 1e6 * float(val[:-1])
    elif val.endswith("G"):
        return 1e9 * float(val[:-1])
    return float(val)

args.frequency = parse_postfix(args.frequency)
args.sample_rate = parse_postfix(args.sample_rate)

kwargs = []
for param in args.device.split(","):
    if param:
        kwargs.append(param.split("="))

sdr = SoapySDR.Device(dict(kwargs))
rxStream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
print(rxStream)

sdr.activateStream(rxStream)
sdr.setSampleRate(SOAPY_SDR_RX, 0, args.sample_rate)
sdr.setFrequency(SOAPY_SDR_RX, 0, args.frequency)


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
    if self.verbose:
        print("Disconnected from: %s:%d" % (addr[0], addr[1]))


new_frequency = None
def new_frequency_callback(freq):
    new_frequency = freq

t = threading.Thread(target=run_gpredict_listener, args=(new_frequency_callback, ))


# Create a re-usable buffer for rx samples
buff = np.array([0] * (4*1024), np.complex64)

total = 0
output = open(args.output_file, "wb")
try:
    print("Receiving...")

    # Receive some samples
    while True:

        if new_frequency:
            sdr.setFrequency(new_frequency)
            new_frequency = None

        sr = sdr.readStream(rxStream, [buff], len(buff), timeoutUs=2000000)
        #print(sr.ret, "samples") # Num samples or error code
        #print(sr.flags) # Flags set by receive operation
        #print(sr.timeNs) # Timestamp for receive buffer

        if sr.ret > 0:
            total += output.write(buff.data)
            print(f"Saved {total} bytes", end="\r")

except KeyboardInterrupt:
    # Shutdown the stream
    sdr.deactivateStream(rxStream)
    sdr.closeStream(rxStream)
    raise

output.close()
