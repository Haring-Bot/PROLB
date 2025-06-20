# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_PROLB_Haring_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED PROLB_Haring_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(PROLB_Haring_FOUND FALSE)
  elseif(NOT PROLB_Haring_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(PROLB_Haring_FOUND FALSE)
  endif()
  return()
endif()
set(_PROLB_Haring_CONFIG_INCLUDED TRUE)

# output package information
if(NOT PROLB_Haring_FIND_QUIETLY)
  message(STATUS "Found PROLB_Haring: 0.0.0 (${PROLB_Haring_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'PROLB_Haring' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT PROLB_Haring_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(PROLB_Haring_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${PROLB_Haring_DIR}/${_extra}")
endforeach()
