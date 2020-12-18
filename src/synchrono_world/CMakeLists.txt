

message(STATUS "====worl")



set(SYN_WORLD_INCLUDES "${SYN_WORLD_INCLUDES}" PARENT_SCOPE)

set(SYN_WORLD_SOURCES
	test_world.cpp
	World.cpp
)

set(SYN_WORLD_HEADERS
	test_world.h
	SynApiWorld.h
	World.h
)
source_group("world" FILES
    ${SYN_WORLD_SOURCES}
    ${SYN_WORLD_HEADERS}
)

add_library(Synchrono_World SHARED
    ${SYN_WORLD_SOURCES}
    ${SYN_WORLD_HEADERS}
)

set_target_properties(Synchrono_World PROPERTIES
                      COMPILE_FLAGS "${SYN_CXX_FLAGS}"
                      LINK_FLAGS "${SYN_LINKERFLAG_SHARED} ${CHRONO_LINKER_FLAGS}"
				  COMPILE_DEFINITIONS "SYN_API_COMPILE_WORLD")

target_link_libraries(Synchrono_World ${CHRONO_LIBRARIES})
