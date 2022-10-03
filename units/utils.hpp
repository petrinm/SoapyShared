#ifndef __UTILS_H__
#define __UTILS_H__


#include <string>
#include <vector>
#include <complex>
#include <cmath>


//typedef float _Complex cf32;
typedef std::complex<float>  cf32;

std::vector<std::string> listSHMs();


#include <cppunit/CompilerOutputter.h>
#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>

#include "SoapySDR/Device.hpp"
#include "SoapySDR/Formats.hpp"

#endif /* __UTILS_H__ */
