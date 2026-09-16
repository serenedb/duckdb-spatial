if(SPATIAL_USE_RTREE)
    add_subdirectory(rtree)
endif()

set(EXTENSION_SOURCES
        ${EXTENSION_SOURCES}
        PARENT_SCOPE
)