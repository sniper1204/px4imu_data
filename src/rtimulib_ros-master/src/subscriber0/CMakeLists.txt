set(LIBS ${LIBS} zmq)

find_package(Threads)
set(LIBS ${LIBS} ${CMAKE_THREAD_LIBS_INIT})

aux_source_directory   	( . SRC_LIST )
get_filename_component	( EXE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
add_executable			( ${EXE_NAME} ${SRC_LIST} )
target_link_libraries 	( ${EXE_NAME} ${LIBS})