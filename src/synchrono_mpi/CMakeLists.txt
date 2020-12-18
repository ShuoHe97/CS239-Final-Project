

message(STATUS "====MPI===")


# set(SYN_MPI_INCLUDES "${CUDA_TOOLKIT_ROOT_DIR}/include")
set(SYN_MPI_INCLUDES "${SYN_MPI_INCLUDES}" PARENT_SCOPE)

include_directories(${SYN_MPI_INCLUDES})

set(SYN_MPI_SOURCES
	test_mpi.cpp
	MPISync.cpp
)

set(SYN_MPI_HEADERS
	test_mpi.h
	MPISync.h
	SynApiMPI.h
)
source_group("MPI" FILES
    ${SYN_MPI_HEADERS}
    ${SYN_MPI_SOURCES}
)

add_library(Synchrono_MPI SHARED
    ${SYN_MPI_HEADERS}
    ${SYN_MPI_SOURCES}
)

set_target_properties(Synchrono_MPI PROPERTIES
                      COMPILE_FLAGS "${SYN_CXX_FLAGS}"
                      LINK_FLAGS "${SYN_LINKERFLAG_SHARED} ${CHRONO_LINKER_FLAGS}"
				  COMPILE_DEFINITIONS "SYN_API_COMPILE_MPI")

target_link_libraries(Synchrono_MPI ${CHRONO_LIBRARIES} ${MPI_LIBRARIES})
