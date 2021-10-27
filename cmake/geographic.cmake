add_subdirectory(${PROJECT_SOURCE_DIR}/thirdParty/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/thirdParty/GeographicLib/include/)
list(APPEND ALL_TARGET_LIBRARIES_CMAKE libGeographiccc)