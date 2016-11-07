if(${OpenMP})
    message("utils_vectormaps - OpenMP support enabled!")

    find_package(OpenMP REQUIRED)

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

else()
    message("utils_vectormaps - OpenMP support disabled!")
endif()
