# Test15 - Data structure library generated by CANDataStructureGenerator.
###########################################################################
# Try to find Test15 library.
# The user can specify an additional search path using the CMake variable
# TEST15_DIR
# First, search at the specific user path setting.
IF(NOT ("${TEST15_DIR}" STREQUAL ""))
    FIND_PATH(TEST15_INCLUDE_DIR test15/GeneratedHeaders_Test15.h
                 NAMES test15
                 PATHS ${TEST15_DIR}/include
                 NO_DEFAULT_PATH)
    FIND_LIBRARY(TEST15_LIBRARY
                 NAMES test15 test15-static
                 PATHS ${TEST15_DIR}/lib
                 NO_DEFAULT_PATH)
ENDIF()
IF(   ("${TEST15_INCLUDE_DIR}" STREQUAL "TEST15_INCLUDE_DIR-NOTFOUND")
   OR ("${TEST15_DIR}" STREQUAL "") )
    MESSAGE(STATUS "Trying to find Test15 in default paths.")
    # If not found, use the system's search paths.
    FIND_PATH(TEST15_INCLUDE_DIR test15/GeneratedHeaders_Test15.h
                 NAMES test15
                 PATHS /usr/include
                       /usr/local/include)
    FIND_LIBRARY(TEST15_LIBRARY
                 NAMES test15 test15-static
                 PATHS /usr/lib
                       /usr/lib64
                       /usr/local/lib
                       /usr/local/lib64)
ENDIF()
IF("${TEST15_INCLUDE_DIR}" STREQUAL "")
    MESSAGE(FATAL_ERROR "Could not find Test15 library.")
ENDIF()
###########################################################################
# Set linking libraries.
SET(TEST15_LIBRARIES ${TEST15_LIBRARY})
SET(TEST15_INCLUDE_DIRS ${TEST15_INCLUDE_DIR})
###########################################################################
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LibTest15 DEFAULT_MSG
                                  TEST15_LIBRARY TEST15_INCLUDE_DIR)
MARK_AS_ADVANCED(TEST15_INCLUDE_DIR TEST15_LIBRARY)