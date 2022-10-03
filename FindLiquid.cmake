# - Find LiquidDSP
# Find the native LiquidDSP includes and library
#
#  Liquid_INCLUDES    - where to find liquid.h
#  Liquid_LIBRARIES   - List of libraries when using Liquid.
#  Liquid_FOUND       - True if LiquidDSP found.

if (Liquid_INCLUDES)
  # Already in cache, be silent
  set (Liquid_FIND_QUIETLY TRUE)
endif (Liquid_INCLUDES)

find_path (Liquid_INCLUDES liquid/liquid.h)

find_library (Liquid_LIBRARIES NAMES liquid)

# handle the QUIETLY and REQUIRED arguments and set Liquid_FOUND to TRUE if
# all listed variables are TRUE
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (Liquid DEFAULT_MSG Liquid_LIBRARIES Liquid_INCLUDES)

#mark_as_advanced (Liquid_LIBRARIES Liquid_INCLUDES)
