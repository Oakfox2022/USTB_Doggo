# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ustb_doggo_urdf_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ustb_doggo_urdf_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ustb_doggo_urdf_FOUND FALSE)
  elseif(NOT ustb_doggo_urdf_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ustb_doggo_urdf_FOUND FALSE)
  endif()
  return()
endif()
set(_ustb_doggo_urdf_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ustb_doggo_urdf_FIND_QUIETLY)
  message(STATUS "Found ustb_doggo_urdf: 0.0.1 (${ustb_doggo_urdf_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ustb_doggo_urdf' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ustb_doggo_urdf_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ustb_doggo_urdf_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ustb_doggo_urdf_DIR}/${_extra}")
endforeach()
