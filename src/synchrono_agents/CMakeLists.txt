

message(STATUS "==Agent Component ====")


set(SYN_AGENT_INCLUDES "${SYN_AGENT_INCLUDES}" PARENT_SCOPE)

#include_directories("${SYN_AGENT_INCLUDES}")

set(SYN_AGENT_SOURCES
	SynAgent.cpp
	SedanAgent.cpp
)

set(SYN_AGENT_HEADERS
	SynAgent.h
	SedanAgent.h
	SynApiAgent.h
)
source_group("AGENT" FILES
    ${SYN_AGENT_HEADERS}
    ${SYN_AGENT_SOURCES}
)

add_library(Synchrono_agent SHARED
    ${SYN_AGENT_HEADERS}
    ${SYN_AGENT_SOURCES}
)

set_target_properties(Synchrono_agent PROPERTIES
                      COMPILE_FLAGS "${SYN_CXX_FLAGS}"
                      LINK_FLAGS "${SYN_LINKERFLAG_SHARED} ${CHRONO_LINKER_FLAGS}"
					  # COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\";CHRONO_VEHICLE_DATA_DIR=\"${CHRONO_VEHICLE_DATA_DIR}\""
				  COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\";CHRONO_VEHICLE_DATA_DIR=\"${CHRONO_VEHICLE_DATA_DIR}\";SYN_API_COMPILE_AGENT")

target_link_libraries(Synchrono_agent ${CHRONO_LIBRARIES})
