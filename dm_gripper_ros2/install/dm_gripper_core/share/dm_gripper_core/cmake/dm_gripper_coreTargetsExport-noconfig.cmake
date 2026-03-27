#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dm_gripper_core::dm_gripper_core" for configuration ""
set_property(TARGET dm_gripper_core::dm_gripper_core APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(dm_gripper_core::dm_gripper_core PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdm_gripper_core.so"
  IMPORTED_SONAME_NOCONFIG "libdm_gripper_core.so"
  )

list(APPEND _cmake_import_check_targets dm_gripper_core::dm_gripper_core )
list(APPEND _cmake_import_check_files_for_dm_gripper_core::dm_gripper_core "${_IMPORT_PREFIX}/lib/libdm_gripper_core.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
