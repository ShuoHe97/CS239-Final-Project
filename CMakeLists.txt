

cmake_minimum_required(VERSION 3.0)



if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message( FATAL_ERROR  "In-source build is not possible and not recommended. Choose an empty directory for build output.")
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})


if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
    cmake_policy(SET CMP0048 NEW)
endif(COMMAND cmake_policy)


if(APPLE)
    set(CMAKE_MACOSX_RPATH OFF)
endif()


set_property(GLOBAL PROPERTY USE_FOLDERS Off)



project(Synchrono)



list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC VCS_REVISION)

if("${VCS_REVISION}" MATCHES ".*-NOTFOUND$")
    set(VCS_REVISION "")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



option(BUILD_TESTING "Build unit tests" OFF)
include(CTest)

set(VALGRIND_COMMAND "/usr/bin/valgrind")
set(VALGRIND_COMMAND_OPTIONS "-v --tool=memcheck --leak-check=full --track-fds=yes --num-callers=50 --show-reachable=yes --track-origins=yes --malloc-fill=0xff --free-fill=0xfe")
set(MEMORYCHECK_COMMAND "/usr/bin/valgrind")
set(CTEST_MEMORYCHECK_COMMAND "/usr/bin/valgrind")
set(CTEST_MEMORYCHECK_COMMAND_OPTIONS "-v --tool=memcheck --leak-check=full --track-fds=yes --num-callers=50 --show-reachable=yes --track-origins=yes --malloc-fill=0xff --free-fill=0xfe")

if (NOT WIN32)
    find_program(CCACHE_EXE ccache)
    if(CCACHE_EXE)
        set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_EXE}")
    endif()
endif()



include(CMakeDependentOption)



if(MSVC OR XCODE_VERSION)
    set(CMAKE_CONFIGURATION_TYPES Release CACHE STRING "Choose the type of build.")
    set_property(CACHE CMAKE_CONFIGURATION_TYPES PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    mark_as_advanced(FORCE CMAKE_BUILD_TYPE)
    mark_as_advanced(CLEAR CMAKE_CONFIGURATION_TYPES)
    set(CPACK_BUILD_TYPE ${CMAKE_CONFIGURATION_TYPES})
else()
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build.")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo)
    set(CPACK_BUILD_TYPE ${CMAKE_BUILD_TYPE})
endif()

if(XCODE_VERSION)
    set(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION "com.apple.compilers.llvmgcc42")
endif()

#-----------------------------------------------------------------------------

set(SYN_COMPILER " " CACHE STRING "Compiler Type" FORCE)

if(MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR CMAKE_COMPILER_IS_CLANGXX)
    if(CMAKE_SIZEOF_VOID_P MATCHES 4)
        set(SYN_COMPILER "COMPILER_GCC" CACHE STRING "Compiler Type" FORCE)
    else()
        set(SYN_COMPILER "COMPILER_GCC_X64" CACHE STRING "Compiler Type" FORCE)
    endif()
endif()

if(MSVC AND CMAKE_CL_64)
    set(SYN_COMPILER "COMPILER_MSVC_X64" CACHE STRING "Compiler Type" FORCE)
elseif(MSVC)
    set(SYN_COMPILER "COMPILER_MSVC" CACHE STRING "Compiler Type" FORCE)
endif()

mark_as_advanced(FORCE SYN_COMPILER)



set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)



set(CPACK_SYSTEM_NAME "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(CMAKE_SIZEOF_VOID_P MATCHES 4)
        set(CPACK_SYSTEM_NAME "win32-${CMAKE_SYSTEM_PROCESSOR}")
    else()
        set(CPACK_SYSTEM_NAME "win64-${CMAKE_SYSTEM_PROCESSOR}")
    endif()
endif()



if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    execute_process(COMMAND "date" "+%Y_%m_%d" OUTPUT_VARIABLE DATE)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    execute_process(COMMAND "date" "+%Y_%m_%d" OUTPUT_VARIABLE DATE)
endif()

string(REGEX REPLACE "(\r?\n)+$" "" DATE "${DATE}")


set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "ChronoEngine is a multibody-dynamics package")
set(CPACK_PACKAGE_FILE_NAME "Synchrono-${CPACK_SYSTEM_NAME}-${CPACK_BUILD_TYPE}-${DATE}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Synchrono")
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "Synchrono")
set(CPACK_PACKAGE_NAME "Synchrono")
set(CPACK_PACKAGE_VENDOR "UWSBEL")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_SOURCE_STRIP_FILES "")


add_subdirectory(src)



include(CPack)
