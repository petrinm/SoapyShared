# CMake script to check if libSoapyShared is installed
# This script runs SoapySDRUtil --info and checks for libSoapyShared module

# Run SoapySDRUtil --info
execute_process(
    COMMAND SoapySDRUtil --info
    OUTPUT_VARIABLE SOAPY_OUTPUT
    ERROR_VARIABLE SOAPY_ERROR
    RESULT_VARIABLE SOAPY_RESULT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
)

# Check if command executed successfully
if(SOAPY_RESULT)
    message(FATAL_ERROR "Failed to run SoapySDRUtil --info. Error: ${SOAPY_ERROR}")
endif()

# Combine output and error streams (SoapySDRUtil may write to stderr)
set(SOAPY_FULL_OUTPUT "${SOAPY_OUTPUT}\n${SOAPY_ERROR}")

# Check if libSoapyShared is found in the output
# Look for pattern like "Module found: /usr/lib/SoapySDR/modules0.8/libSoapyShared.so (9d88866)"
string(REGEX MATCH "Module found:[^\n]*libSoapyShared\\.so[^\n]*" MODULE_LINE "${SOAPY_FULL_OUTPUT}")

if(NOT MODULE_LINE)
    message(FATAL_ERROR "libSoapyShared module not found in SoapySDR modules.\n"
                        "Expected pattern: 'Module found: .../libSoapyShared.so ...'\n"
                        "SoapySDRUtil --info output:\n${SOAPY_FULL_OUTPUT}\n"
                        "Please install libSoapyShared.so to the SoapySDR modules directory.")
else()
    message(STATUS "libSoapyShared module found in SoapySDR modules.")
    message(STATUS "Found: ${MODULE_LINE}")
endif()

