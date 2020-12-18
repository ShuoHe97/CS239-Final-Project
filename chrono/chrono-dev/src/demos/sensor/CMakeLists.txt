#--------------------------------------------------------------
# List of all sensor demos

#List of demos that show the sensor framework
SET(DEMOS
  demo_SEN_buildtest
  demo_SEN_camera
  demo_SEN_lidar
  demo_SEN_GPSIMU
  demo_SEN_zed
  demo_SEN_vis_materials
)

#list of demos that combine sensor simulation with vehicle dynamics
IF(ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_VEHICLE)
    SET(DEMOS
      ${DEMOS}
      demo_SEN_sedan
      demo_SEN_HMMWV
    )
ENDIF()

#--------------------------------------------------------------
# Additional include paths

# INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )

#--------------------------------------------------------------
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

# Add executables

MESSAGE(STATUS "Demo programs for Sensor module...")

#FOR OPTIONAL VEHICLE WITH IRRLICHT DEMO
IF(ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_VEHICLE)
    INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
ENDIF()


FOREACH(PROGRAM ${DEMOS})
  MESSAGE(STATUS "...add ${PROGRAM}")

  ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

  # Visual Studio needs to have the debugger working directory set explicitly to the output directory
  # (specifically, so that the data directory is found when running demos in the debugger)
  IF(${CMAKE_CXX_COMPILER_ID} STREQUAL MSVC)
	set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$(OutputPath)")
  ENDIF()

    #FOR OPTIONAL VEHICLE WITH IRRLICHT DEMO
    IF(ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_VEHICLE)
        SET_TARGET_PROPERTIES(${PROGRAM}  PROPERTIES
          FOLDER demos
          COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_SENSOR_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
          LINK_FLAGS "${CH_LINKERFLAG_EXE}"
          )
        TARGET_LINK_LIBRARIES(${PROGRAM}
          ChronoEngine
          ChronoEngine_sensor
          ChronoEngine_irrlicht
          ChronoEngine_vehicle
          ChronoModels_vehicle
          )
        ADD_DEPENDENCIES (${PROGRAM}
            ChronoEngine
            ChronoEngine_sensor
            ChronoEngine_irrlicht
            ChronoEngine_vehicle
            ChronoModels_vehicle
        )
    ELSE(ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_VEHICLE)
        SET_TARGET_PROPERTIES(${PROGRAM}  PROPERTIES
          FOLDER demos
          COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_SENSOR_CXX_FLAGS}"
          LINK_FLAGS "${CH_LINKERFLAG_EXE}"
          )
        TARGET_LINK_LIBRARIES(${PROGRAM}
          ChronoEngine
          ChronoEngine_sensor
          )
        ADD_DEPENDENCIES (${PROGRAM} ChronoEngine ChronoEngine_sensor)
    ENDIF()




  INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()
