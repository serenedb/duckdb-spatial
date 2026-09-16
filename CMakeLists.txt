cmake_minimum_required(VERSION 3.5)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set extension name here
set(TARGET_NAME spatial)
set(EXTENSION_NAME ${TARGET_NAME}_extension)
set(CMAKE_CXX_STANDARD
    17
    CACHE STRING "C++ standard")

project(${TARGET_NAME})

add_definitions(-DDUCKDB_MAJOR_VERSION=${DUCKDB_MAJOR_VERSION})
add_definitions(-DDUCKDB_MINOR_VERSION=${DUCKDB_MINOR_VERSION})
add_definitions(-DDUCKDB_PATCH_VERSION=${DUCKDB_PATCH_VERSION})
add_definitions(-DDUCKDB_SPATIAL_EXTENSION=1)

# Options

# Enable GEOS support
option(SPATIAL_USE_GEOS "Enable GEOS support" ON)

# Enable the Boost.Geometry backend (the permissively licensed replacement for the GEOS module). It registers the same
# predicate functions, so exactly one of the two backends may be on.
option(SPATIAL_USE_BOOST "Enable the Boost.Geometry backend" OFF)

if(SPATIAL_USE_BOOST AND SPATIAL_USE_GEOS)
  message(
    FATAL_ERROR
      "SPATIAL_USE_BOOST and SPATIAL_USE_GEOS both register ST_Intersects and friends; enable only one"
  )
endif()

if(SPATIAL_USE_BOOST)
  add_definitions(-DSPATIAL_USE_BOOST=1)
endif()

# Enable the persistent RTREE index (the index type, its CREATE INDEX plan, its
# scan/pushdown optimizer and the rtree_index_* pragmas). The in-memory FlatRTree
# that accelerates spatial joins is a separate, per-query structure and is always
# built.
option(SPATIAL_USE_RTREE "Enable the RTREE index" OFF)

if(SPATIAL_USE_RTREE)
  add_definitions(-DSPATIAL_USE_RTREE=1)
endif()

add_subdirectory(src/spatial)
add_subdirectory(src/sgl)
include_directories(src)

include_directories(src/third_party/yyjson/include)
add_subdirectory(src/third_party/yyjson)

include_directories(src/third_party/protozero/include)

include_directories(src/third_party/shapelib)
add_subdirectory(src/third_party/shapelib)

include_directories(src/third_party/geographiclib)
add_subdirectory(src/third_party/geographiclib)

add_library(${EXTENSION_NAME} STATIC ${EXTENSION_SOURCES})

set(ZLIB_USE_STATIC_LIBS ON)
find_package(ZLIB REQUIRED)

# Important: The link order matters, its the reverse order of dependency
set(EXTENSION_DEPENDENCIES "")

list(APPEND EXTENSION_DEPENDENCIES ZLIB::ZLIB)

if(SPATIAL_USE_GEOS)
  message(STATUS "Building with GEOS functionality")
  if(NOT TARGET GEOS::geos_c)
    find_package(GEOS REQUIRED)
  endif()
  set(EXTENSION_DEPENDENCIES ${EXTENSION_DEPENDENCIES} GEOS::geos_c)
  add_definitions(-DSPATIAL_USE_GEOS=1)
endif()

if(SPATIAL_USE_BOOST)
  message(STATUS "Building with Boost.Geometry functionality")
  if(NOT TARGET Boost::headers)
    find_package(Boost REQUIRED)
  endif()
  list(APPEND EXTENSION_DEPENDENCIES Boost::headers)
endif()

if(WIN32)
  list(APPEND EXTENSION_DEPENDENCIES wbemuuid.lib)
endif()

if((NOT EMSCRIPTEN) AND (NOT IOS))
  if(APPLE)
    find_library(CoreFoundation_Library CoreFoundation)
    find_library(SystemConfiguration_Library SystemConfiguration)
    list(APPEND EXTENSION_DEPENDENCIES ${CoreFoundation_Library}
         ${SystemConfiguration_Library})
  endif()
endif()

# Add dependencies to extension
target_link_libraries(${EXTENSION_NAME} ${EXTENSION_DEPENDENCIES})

# Build extensions
set(PARAMETERS "-warnings")
build_loadable_extension(${TARGET_NAME} ${PARAMETERS} ${EXTENSION_SOURCES})

target_link_libraries(${TARGET_NAME}_loadable_extension
                      ${EXTENSION_DEPENDENCIES})

install(
  TARGETS ${EXTENSION_NAME}
  EXPORT "${DUCKDB_EXPORT_SET}"
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}")
