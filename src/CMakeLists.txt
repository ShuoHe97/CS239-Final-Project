
if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

set(SYN_C_FLAGS "")
set(SYN_CXX_FLAGS "")

find_package(ModernCXX)
#set(CH_CXX_FLAGS "${CH_CXX_FLAGS} ${CMAKE_CXX11_STANDARD_COMPILE_OPTION}")
set(SYN_CXX_FLAGS "${SYN_CXX_FLAGS} ${CMAKE_CXX14_STANDARD_COMPILE_OPTION}")

# This is a hack -- look for a more robust solution
set(SYN_CXX14 FALSE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  message(STATUS "GCC version:  ${CMAKE_CXX_COMPILER_VERSION}")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9.9)
    set(SYN_CXX14 TRUE)
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(STATUS "Clang version:  ${CMAKE_CXX_COMPILER_VERSION}")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 3.4)
    set(SYN_CXX14 TRUE)
  endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  message(STATUS "Visual Studio version:  ${MSVC_VERSION}")
  if(MSVC_VERSION GREATER 1800)
    set(SYN_CXX14 TRUE)
  endif()
endif()
message(STATUS "Compiler supports C++14:  ${SYN_CXX14}")

# cmake/FindModernCXX.cmake behaves strangely with PGI compilers, don't trust it
if (CMAKE_CXX_COMPILER_ID MATCHES "PGI")
  set(SYN_CXX_FLAGS "")
  message(STATUS "PGI version: ${CMAKE_CXX_COMPILER_VERSION}")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 17.4)
    message(STATUS "CMAKE_CXX14_STANDARD_COMPILE_OPTION can't be trusted for PGI, manually adding C++14 support")
    set(SYN_CXX14 TRUE)
    set(SYN_CXX_FLAGS "-std=c++14")
  endif()
endif()


#-----------------------------------------------------------------------------
# Find Chrono
#-----------------------------------------------------------------------------
message(STATUS "Searching for Chrono...")
set(Chrono_DIR "../chrono/chrono-dev/build/cmake" CACHE STRING "")
find_package(Chrono
             OPTIONAL_COMPONENTS
             Irrlicht
             Vehicle
             Sensor
             CONFIG)

message(STATUS "Chrono libraries: ${CHRONO_LIBRARIES}")

#-----------------------------------------------------------------------------
# Find MPI
#-----------------------------------------------------------------------------
find_package(MPI REQUIRED)
message(STATUS "MPI libraries: ${MPI_LIBRARIES}")

#-----------------------------------------------------------------------------
# Find Cuda
#-----------------------------------------------------------------------------
message(STATUS "Searching for CUDA...")

find_package(CUDA QUIET)

if(CUDA_FOUND)
  message(STATUS "  CUDA version:          ${CUDA_VERSION_STRING}")
  message(STATUS "  CUDA toolkit root dir: ${CUDA_TOOLKIT_ROOT_DIR}")
  mark_as_advanced(FORCE CUDA_TOOLKIT_ROOT_DIR)
else()
  message(STATUS "  CUDA not found (consider manually setting CUDA_TOOLKIT_ROOT_DIR)")
  mark_as_advanced(CLEAR CUDA_TOOLKIT_ROOT_DIR)
  mark_as_advanced(FORCE CUDA_USE_STATIC_CUDA_RUNTIME)
endif()

mark_as_advanced(FORCE
    CUDA_BUILD_CUBIN
    CUDA_BUILD_EMULATION
    CUDA_SEPARABLE_COMPILATION
    CUDA_SDK_ROOT_DIR
    CUDA_HOST_COMPILER
    CUDA_rt_LIBRARY)

# Identify the CUDA architecture
if(CUDA_FOUND)

  mark_as_advanced(FORCE CUDA_USE_STATIC_CUDA_RUNTIME)

  set(CUDA_SEPARABLE_COMPILATION OFF)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CUDA_SEPARABLE_COMPILATION OFF)
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CUDA_PROPAGATE_HOST_FLAGS OFF)
    if(${SYN_CXX14})
      		set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std c++14")
      		set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler -std=c++14")
    else()
      		set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std c++11")
      		set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler -std=c++11")
    endif()
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC)
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; --compiler-options -fPIC)
  endif()

  include(${CMAKE_SOURCE_DIR}/cmake/FindCudaArch.cmake)
  SELECT_NVCC_ARCH_FLAGS(NVCC_FLAGS_EXTRA)
  list(APPEND CUDA_NVCC_FLAGS ${NVCC_FLAGS_EXTRA})

  message(STATUS "  CUDA compile flags:   ${CUDA_NVCC_FLAGS}")

endif()


#-----------------------------------------------------------------------------




#-----------------------------------------------------------------------------
# Windows Specific Flags and Settings
#-----------------------------------------------------------------------------
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(MSVC)
        add_definitions( "-D_CRT_SECURE_NO_DEPRECATE" )  
        add_definitions( "-D_SCL_SECURE_NO_DEPRECATE" )  
        add_definitions( "-DNOMINMAX" ) 
        add_definitions( "-MP" ) 
    endif(MSVC)

    if(MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
        set(SYN_CXX_FLAGS "${SYN_CXX_FLAGS} -D_MINGW -D_WINDOWS")
        set(SYN_C_FLAGS "${SYN_C_FLAGS} -D_MINGW -D_WINDOWS")
        set(SYN_LINKERFLAG_EXE "-Wl,--enable-runtime-pseudo-reloc")
        set(SYN_LINKERFLAG_SHARED "-Wl,--export-all-symbols -Wl,--enable-auto-import -Wl,--enable-runtime-pseudo-reloc")

        if(CMAKE_SIZEOF_VOID_P MATCHES 8)
            set(SYN_CXX_FLAGS "${SYN_CXX_FLAGS} -m64")
            set(SYN_C_FLAGS "${SYN_C_FLAGS} -m64")
        endif()
    endif()
endif()



set(SYN_CXX_FLAGS "${SYN_CXX_FLAGS} ${CH_CXX_FLAGS}")
set(SYN_C_FLAGS "${SYN_C_FLAGS} ${CH_C_FLAGS}")
set(SYN_LINKERFLAG_SHARED "${SYN_LINKERFLAG_SHARED} ${CH_LINKERFLAG_SHARED}")


message(STATUS "==== Submodules ====")
if(BUILD_TESTING)
  message(STATUS "Build Google test framework")
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/googletest/CMakeLists.txt")

    option(INSTALL_GTEST "" OFF)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    add_subdirectory(chrono_thirdparty/googletest)


    mark_as_advanced(FORCE BUILD_GMOCK)
    mark_as_advanced(FORCE INSTALL_GTEST)
    set(SYNCHRONO_HAS_GTEST "#define SYNCHRONO_HAS_GTEST")
  else()
    message("  Google test code not found: update git submodules.")
    message("  Building of unit tests was disabled.")
    set(BUILD_TESTING OFF CACHE BOOL "Build the testing tree." FORCE)
    set(SYNCHRONO_HAS_GTEST "")
  endif()
endif()






set(SYN_MPI_INCLUDES ${SYN_MPI_INCLUDES} "${MPI_C_HEADER_DIR}")
set(SYN_MPI_INCLUDES ${SYN_MPI_INCLUDES} "${MPI_CXX_HEADER_DIR}")

set(SYN_INCLUDES "${PROJECT_SOURCE_DIR}/src" ${CHRONO_INCLUDE_DIRS})
set(SYN_INCLUDES "${SYN_INCLUDES}" "${SYN_MPI_INCLUDES}")

include_directories(${SYN_INCLUDES})

add_subdirectory(synchrono_world) 

add_subdirectory(synchrono_mpi) 
add_subdirectory(synchrono_agents) 



add_subdirectory(demos)
if(BUILD_TESTING)
  add_subdirectory(tests/unit_tests)
endif()






set(SYN_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src")
set(SYN_INCLUDE_DIRS ${SYN_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/synchrono_mpi")
set(SYN_INCLUDE_DIRS ${SYN_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/synchrono_world")
set(SYN_INCLUDE_DIRS ${SYN_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/src/synchrono_agents")


set(SYN_BINARY_DIR "${PROJECT_BINARY_DIR}")
set(SYN_DATA_DIR "${PROJECT_SOURCE_DIR}/data/")
configure_file("${PROJECT_SOURCE_DIR}/cmake/ChronoConfig.cmake.in"
               "${PROJECT_BINARY_DIR}/cmake/ChronoConfig.cmake"
               @ONLY)

set(SYN_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(SYN_INCLUDE_DIRS ${SYN_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono")
set(SYN_INCLUDE_DIRS ${SYN_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono/collision/bullet")
set(SYN_INCLUDE_DIRS ${SYN_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono/collision/gimpact")
set(SYN_INCLUDE_DIRS ${SYN_INCLUDE_DIRS} "${CMAKE_INSTALL_PREFIX}/include/chrono/collision/convexdecomposition/HACD")
set(SYN_INCLUDE_DIRS ${SYN_INCLUDE_DIRS} "${EIGEN3_INCLUDE_DIR}")
set(SYN_BINARY_DIR ${CMAKE_INSTALL_PREFIX})
set(SYN_DATA_DIR "${CMAKE_INSTALL_PREFIX}/${SYN_INSTALL_DATA}/")
configure_file("${PROJECT_SOURCE_DIR}/cmake/ChronoConfig.cmake.in"
               "${PROJECT_BINARY_DIR}/cmake/ChronoConfig.cmake.install"
               @ONLY)





