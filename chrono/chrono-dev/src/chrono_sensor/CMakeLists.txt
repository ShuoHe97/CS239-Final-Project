#=============================================================================
# CMake configuration file for Chrono Sensor module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_SENSOR "Enable the Chrono Sensor module" OFF)

IF(NOT ENABLE_MODULE_SENSOR)
  return()
ENDIF()

message(STATUS "==== Chrono Sensor module ====")

# Return now if CUDA is not available
if(NOT CUDA_FOUND)
  message("Chrono::Sensor requires CUDA")
  message(STATUS "Chrono::Sensor disabled")
  set(ENABLE_MODULE_SENSOR OFF CACHE BOOL "Enable the Chrono Sensor module" FORCE)
  return()
endif()

#Check for GLFW to use as window to display data for debug purposes
mark_as_advanced(CLEAR GLFW_INCLUDE_DIR)
mark_as_advanced(CLEAR GLFW_LIBRARY)

find_package(GLFW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

SET(CH_SENSOR_INCLUDES
    ${CH_SENSOR_INCLUDES}
    ${GLFW_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
)



# ------------------------------------------------------------------------------
# Find and set everything needed for OptiX
# ------------------------------------------------------------------------------
find_package(OptiX REQUIRED)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    add_compile_definitions(NOMINMAX)
	message(STATUS "NOMINMAX set for windows")
endif()

SET(CH_SENSOR_INCLUDES
    ${CH_SENSOR_INCLUDES}
    ${OptiX_INCLUDE}/optixu
    ${OptiX_INCLUDE}
    ${CUDA_INCLUDE_DIRS}
)

# add the necessary optix libraries to the list to link
# list(APPEND LIBRARIES optix)
list(APPEND LIBRARIES ${optix_LIBRARY})
list(APPEND LIBRARIES ${optix_rpath})
list(APPEND LIBRARIES ${CUDA_LIBRARIES})
list(APPEND LIBRARIES ${GLFW_LIBRARY})
list(APPEND LIBRARIES ${GLEW_LIBRARY})

message(STATUS "\n\n\n\n\n\n\n\n\n\nSensor Libraries: ${LIBRARIES} \n\n")


list(APPEND CUDA_NVCC_FLAGS "--use_fast_math")

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigSensor.h.in
		${PROJECT_BINARY_DIR}/chrono_sensor/ChConfigSensor.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_sensor/ChConfigSensor.h"
		DESTINATION include/chrono_sensor)

#-----------------------------------------------------------------------------
# LIST CUDA FILES USED FOR RT PROGRAMS - TO BE COMPILED TO PTX SHADERS
#-----------------------------------------------------------------------------
set(ChronoEngine_sensor_RT_SOURCES
    rtkernels/depth_trace.cu
    rtkernels/box.cu
    rtkernels/sphere.cu
    rtkernels/camera.cu
    rtkernels/depth_camera.cu
    rtkernels/lidar.cu
    rtkernels/miss.cu
    rtkernels/basic_shader.cu
    rtkernels/triangle_mesh.cu
)
set(ChronoEngine_sensor_RT_HEADERS
    rtkernels/ray_utils.h
)

source_group("RT Programs" FILES
    ${ChronoEngine_sensor_RT_SOURCES}
	${ChronoEngine_sensor_RT_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST CUDA FILES THAT ARE TO BE COMPILED AS SOURCE
#-----------------------------------------------------------------------------
set(ChronoEngine_sensor_CUDA_SOURCES
	cuda/grayscale.cu
    cuda/pointcloud.cu
)
set(ChronoEngine_sensor_CUDA_HEADERS
	cuda/grayscale.cuh
    cuda/grayscale.cuh
)

source_group("Cuda" FILES
    ${ChronoEngine_sensor_CUDA_SOURCES}
	${ChronoEngine_sensor_CUDA_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE CORE SENSOR LIBRARY
#-----------------------------------------------------------------------------
set(ChronoEngine_sensor_SOURCES
    ChSensor.cpp
    ChCameraSensor.cpp
    ChDepthCameraSensor.cpp
    ChLidarSensor.cpp
    ChIMUSensor.cpp
    ChGPSSensor.cpp
    ChSensorManager.cpp
    ../chrono_thirdparty/nothings/stb_image.cpp
    ../chrono_thirdparty/nothings/stb_image_write.cpp
)
set(ChronoEngine_sensor_HEADERS
	ChApiSensor.h
    ChSensor.h
    ChDepthCameraSensor.h
    ChLidarSensor.h
    ChIMUSensor.h
    ChGPSSensor.h
    ChSensorManager.h
	ChSensorBuffer.h
    ../chrono_thirdparty/nothings/stb_image.h
    ../chrono_thirdparty/nothings/stb_image_write.h
)

source_group("Source" FILES
    ${ChronoEngine_sensor_SOURCES}
	${ChronoEngine_sensor_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE CORE SENSOR LIBRARY
#-----------------------------------------------------------------------------
set(ChronoEngine_sensor_SCENE_SOURCES
    scene/ChScene.cpp
)
set(ChronoEngine_sensor_SCENE_HEADERS
    scene/ChScene.h
    scene/ChWeather.h
    scene/lights.h
)

source_group("Scene" FILES
    ${ChronoEngine_sensor_SCENE_SOURCES}
	${ChronoEngine_sensor_SCENE_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE SENSOR OPTIX LIBRARY
#-----------------------------------------------------------------------------
set(ChronoEngine_sensor_OPTIX_SOURCES
    optixcpp/ChOptixEngine.cpp
    optixcpp/ChOptixUtils.cpp
)
set(ChronoEngine_sensor_OPTIX_HEADERS
    optixcpp/ChOptixEngine.h
    optixcpp/ChOptixUtils.h
)

source_group("Optix" FILES
    ${ChronoEngine_sensor_OPTIX_SOURCES}
	${ChronoEngine_sensor_OPTIX_HEADERS}
)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE FILTERS FOR THE SENSOR LIBRARY
#-----------------------------------------------------------------------------
set(ChronoEngine_sensor_FILTERS_SOURCES
	filters/ChFilter.cpp
	filters/ChFilterOptixRender.cpp
	filters/ChFilterIMUUpdate.cpp
	filters/ChFilterGPSUpdate.cpp
	filters/ChFilterVisualize.cpp
    filters/ChFilterSave.cpp
    filters/ChFilterSaveXYZRGB.cpp
	filters/ChFilterGrayscale.cpp
	filters/ChFilterAccess.cpp
    filters/ChFilterPCfromDepth.cpp
    filters/ChFilterVisualizePointCloud.cpp
)
set(ChronoEngine_sensor_FILTERS_HEADERS
	filters/ChFilter.h
	filters/ChFilterOptixRender.h
    filters/ChFilterIMUUpdate.h
	filters/ChFilterGPSUpdate.h
	filters/ChFilterVisualize.h
    filters/ChFilterSave.h
    filters/ChFilterSaveXYZRGB.h
	filters/ChFilterGrayscale.h
	filters/ChFilterAccess.h
    filters/ChFilterPCfromDepth.h
    filters/ChFilterVisualizePointCloud.h
)

source_group("Filters" FILES
    ${ChronoEngine_sensor_FILTERS_SOURCES}
	${ChronoEngine_sensor_FILTERS_HEADERS}
)



#-----------------------------------------------------------------------------
# LIST THE UTILITY FILES THAT WILL BE EXPOSED TO THE USER
#-----------------------------------------------------------------------------
set(ChronoEngine_sensor_UTILS_SOURCES
    utils/ChSensorUtils.cpp
    utils/ChVisualMaterialUtils.cpp
)
set(ChronoEngine_sensor_UTILS_HEADERS
    utils/ChSensorUtils.h
    utils/ChVisualMaterialUtils.h
	utils/CudaMallocHelper.h
)

source_group(Utils FILES
    ${ChronoEngine_sensor_UTILS_SOURCES}
	${ChronoEngine_sensor_UTILS_HEADERS}
)

#-----------------------------------------------------------------------------
# Create the ChronoEngine_sensor library
#-----------------------------------------------------------------------------
list(APPEND LIBRARIES "ChronoEngine")

# Generate the OBJ files
CUDA_WRAP_SRCS(ChronoEngine_sensor OBJ generated_obj_files ${ChronoEngine_sensor_CUDA_SOURCES} )


# Generate the PTX files
set(CUDA_GENERATED_OUTPUT_DIR "${CMAKE_BINARY_DIR}/generated")
add_definitions( -DPTX_GENERATED_PATH="${CUDA_GENERATED_OUTPUT_DIR}/" )
CUDA_WRAP_SRCS(ChronoEngine_sensor PTX generated_rt_files ${ChronoEngine_sensor_RT_SOURCES} )
source_group("Generated Files" FILES ${generated_rt_files})

# reset the cuda generate directory for cuda files being compiled into obj
set(CUDA_GENERATED_OUTPUT_DIR "")

# ----------------------------------------------------------------------------
# Collect additional include directories necessary for the Sensor module.
# Make some variables visible from parent directory
# ----------------------------------------------------------------------------
set(CH_SENSOR_INCLUDES ${CH_SENSOR_INCLUDES} "${CUDA_TOOLKIT_ROOT_DIR}/include")

SET(CH_SENSOR_INCLUDES            "${CH_SENSOR_INCLUDES}"        PARENT_SCOPE)
SET(SENSOR_LIBRARIES         "${LIBRARIES}"                  PARENT_SCOPE)
set(CH_SENSOR_CXX_FLAGS     "${CH_SENSOR_CXX_FLAGS}"        PARENT_SCOPE)
set(CH_SENSOR_C_FLAGS       "${CH_SENSOR_C_FLAGS}"          PARENT_SCOPE)

message(STATUS "\n\n\n\n\n\n\n\n\n\nSensor Includes: ${CH_SENSOR_INCLUDES} \n\n")
message(STATUS "\n\nOptix Includes: ${OptiX_INCLUDE} \n\n")


add_library(ChronoEngine_sensor SHARED
    ${ChronoEngine_sensor_SOURCES}
    ${ChronoEngine_sensor_HEADERS}
    ${ChronoEngine_sensor_UTILS_SOURCES}
    ${ChronoEngine_sensor_UTILS_HEADERS}
    ${ChronoEngine_sensor_OPTIX_SOURCES}
    ${ChronoEngine_sensor_OPTIX_HEADERS}
    ${ChronoEngine_sensor_FILTERS_SOURCES}
    ${ChronoEngine_sensor_FILTERS_HEADERS}
    ${ChronoEngine_sensor_SCENE_SOURCES}
    ${ChronoEngine_sensor_SCENE_HEADERS}
    ${ChronoEngine_sensor_CUDA_SOURCES}
    ${ChronoEngine_sensor_CUDA_HEADERS}
    ${ChronoEngine_sensor_RT_SOURCES}
	${ChronoEngine_sensor_RT_HEADERS}
    ${generated_obj_files}
    ${generated_rt_files}
)

# windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4661 /wd4005")

    #FIGURE OUT WHERE ALL THE LIBRARY DLLS FROM WINDOWS ARE
    #SET(CH_IRRLICHT_DLL "${IRRLICHT_ROOT}/bin/Win32-VisualStudio/Irrlicht.dll")
endif()

set_target_properties(ChronoEngine_sensor PROPERTIES
                    COMPILE_FLAGS "${CH_CXX_FLAGS}"
                    LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
				  	COMPILE_DEFINITIONS "CH_API_COMPILE_SENSOR")

target_include_directories(ChronoEngine_sensor PUBLIC ${CH_SENSOR_INCLUDES})

target_link_libraries(ChronoEngine_sensor ${LIBRARIES})

install(TARGETS ChronoEngine_sensor
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${ChronoEngine_sensor_HEADERS}
		DESTINATION include/chrono_sensor)
install(FILES ${ChronoEngine_sensor_UTILS_HEADERS}
		DESTINATION include/chrono_sensor/utils)
install(FILES ${ChronoEngine_sensor_OPTIX_HEADERS}
        DESTINATION include/chrono_sensor/optix)
install(FILES ${ChronoEngine_sensor_FILTERS_HEADERS}
        DESTINATION include/chrono_sensor/filters)
install(FILES ${ChronoEngine_sensor_CUDA_HEADERS}
DESTINATION include/chrono_sensor/cuda)
