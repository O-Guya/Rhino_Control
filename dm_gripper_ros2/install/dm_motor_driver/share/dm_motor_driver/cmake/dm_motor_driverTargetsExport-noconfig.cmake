#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dm_motor_driver::dm_motor_driver" for configuration ""
set_property(TARGET dm_motor_driver::dm_motor_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(dm_motor_driver::dm_motor_driver PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdm_motor_driver.so"
  IMPORTED_SONAME_NOCONFIG "libdm_motor_driver.so"
  )

list(APPEND _cmake_import_check_targets dm_motor_driver::dm_motor_driver )
list(APPEND _cmake_import_check_files_for_dm_motor_driver::dm_motor_driver "${_IMPORT_PREFIX}/lib/libdm_motor_driver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
