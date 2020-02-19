#!/usr/bin/env python3
# Script to generate C++ code for methods which only call the corresponding
# methods of the slave device.
# The functions which do something else are listed in dont_wrap.

# Install the parser: pip3 install CppHeaderParser
import CppHeaderParser

dont_wrap = 'Device getDriverKey getNumChannels getFullDuplex getStreamFormats setupStream closeStream activateStream deactivateStream readStream writeStream readStreamStatus getNumDirectAccessBuffers acquireReadBuffer releaseReadBuffer acquireWriteBuffer releaseWriteBuffer setFrequency setSampleRate'.split(' ')

hp = CppHeaderParser.CppHeader("/usr/include/SoapySDR/Device.hpp")

def fix_type(t):
    if t in ['Kwargs', 'ArgInfoList', 'Stream', 'Range', 'RangeList', 'ArgInfoList', 'ArgInfo', 'ArgInfoList', 'Stream','Stream *']:
        return 'SoapySDR::'+t
    else:
        return t

for m in hp.classes['SOAPY_SDR_API']['methods']['public']:
    if m['virtual'] and m['name'] not in dont_wrap:
        print('\t%s %s(%s)%s {\n\t\treturn slave->%s(%s);\n\t}\n' % (
            fix_type(m['rtnType']),
            m['name'],
            ', '.join([(fix_type(p['type']) + ' ' + p['name']).strip()
                for p in m['parameters']]),
            ' const' if m['const'] else '',
            m['name'],
            ', '.join([p['name'] for p in m['parameters']]),
            ))
