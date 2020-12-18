

INCLUDE_DIRECTORIES(${SYN_INCLUDES} ${CH_INCLUDES})

message(STATUS "synchrono includes: ${SYN_INCLUDES}")
message(STATUS "chrono includes: ${CH_INCLUDES}")

set(SYN_LIBS Synchrono_MPI Synchrono_agent)

message(STATUS "synchrono libs: ${SYN_LIBS}")

SET(COMPILER_FLAGS "${SYN_CXX_FLAGS}")
SET(LINKER_FLAGS "${SYN_LINKERFLAG_EXE}")
LIST(APPEND LIBS "")


#--------------------------------------------------------------

MESSAGE(STATUS "Demo programs for testing if Synchrono builds... ${CHRONO_LINKER_FLAGS}")

add_executable(demo_MPI_sedans demo_MPI_sedans.cpp)

set_target_properties(demo_MPI_sedans PROPERTIES
	    COMPILE_FLAGS "${SYNCHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS} ${MPI_COMPILE_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\";CHRONO_VEHICLE_DATA_DIR=\"${CHRONO_VEHICLE_DATA_DIR}\""
			LINK_FLAGS "${SYN_LINKERFLAG_EXE} ${CHRONO_LINKER_FLAGS} ${MPI_LINK_FLAGS}")

target_link_libraries(demo_MPI_sedans ${CHRONO_LIBRARIES} ${SYN_LIBS} ${MPI_LIBRARIES})
