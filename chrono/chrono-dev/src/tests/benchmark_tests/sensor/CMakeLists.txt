if(NOT ENABLE_MODULE_SENSOR)
    return()
endif()


SET(LIBRARIES ChronoEngine ChronoEngine_sensor ${SENSOR_LIBRARIES})
INCLUDE_DIRECTORIES( ${CH_INCLUDES} ${CH_SENSOR_INCLUDES} )


# message(STATUS "\n\nbenchmark include directories: ${CH_IRRLICHTINC}\n\n")
# ------------------------------------------------------------------------------

set(TESTS
    btest_SEN_meshes
    btest_SEN_reflections
    )

# ------------------------------------------------------------------------------

MESSAGE(STATUS "benchmark test programs for SENSOR module...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)
