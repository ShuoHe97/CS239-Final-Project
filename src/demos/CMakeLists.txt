IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (SYN_LINKERFLAG_EXE  "${SYN_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()


set(CMAKE_SKIP_BUILD_RPATH  FALSE)


set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")


set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")



if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()



message(STATUS)
message(STATUS "==== Demo programs ====")
message(STATUS)

add_subdirectory(demo_chrono)
add_subdirectory(demo_MPI_sedans)

if(ENABLE_MODULE_INTERFACE)
	option(BUILD_DEMOS_INTERFACE "Build demo programs for Interface module" TRUE)
	mark_as_advanced(FORCE BUILD_DEMOS_INTERFACE)
	if(BUILD_DEMOS_INTERFACE)
		add_subdirectory(demo_interface)
	endif()
endif()
