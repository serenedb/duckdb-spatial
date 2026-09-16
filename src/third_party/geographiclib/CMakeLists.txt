add_library(geographiclib OBJECT
    ${CMAKE_CURRENT_SOURCE_DIR}/geodesic.c
    ${CMAKE_CURRENT_SOURCE_DIR}/geodesic.h
)

set(EXTENSION_SOURCES
        ${EXTENSION_SOURCES}
        $<TARGET_OBJECTS:geographiclib>
        PARENT_SCOPE
    )
