# ------------------------------------------------------------------------------
# List of demos according their depencies
# ------------------------------------------------------------------------------

# List FSI demos that do not have other dependencies

#--------------------------------------------------------------

INCLUDE_DIRECTORIES(${SYN_INCLUDES})

message(STATUS "synchrono includes: ${SYN_INCLUDES}")
message(STATUS "chrono includes: ${CH_INCLUDE_DIRS}")

SET(COMPILER_FLAGS "${SYN_CXX_FLAGS}")
SET(LINKER_FLAGS "${SYN_LINKERFLAG_EXE}")
LIST(APPEND LIBS "")


#--------------------------------------------------------------

MESSAGE(STATUS "Demo programs for testing if Synchrono builds...")

add_executable(demo_chrono demo_chrono.cpp)

set_target_properties(demo_chrono PROPERTIES
	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	    LINK_FLAGS "${CHRONO_LINKER_FLAGS} ${CHRONO_LINKER_FLAGS}")

target_link_libraries(demo_chrono ${CHRONO_LIBRARIES})
