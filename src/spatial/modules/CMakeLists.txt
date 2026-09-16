add_subdirectory(main)
add_subdirectory(geodesic)
if(SPATIAL_USE_GEOS)
  add_subdirectory(geos)
endif()
if(SPATIAL_USE_BOOST)
  add_subdirectory(boost)
endif()
add_subdirectory(osm)
add_subdirectory(shapefile)
add_subdirectory(mvt)
add_subdirectory(wkb)

set(EXTENSION_SOURCES
    ${EXTENSION_SOURCES}
    PARENT_SCOPE)
