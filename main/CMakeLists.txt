

idf_component_register(SRCS "main.cpp" INCLUDE_DIRS ".")

list(APPEND cxx_compile_options "-std=gnu++17")

string(TIMESTAMP BUILD_DATE "%Y-%m-%d %H:%M:%S UTC" UTC)
configure_file(appVersion.h.in appVersion.h)


FILE(GLOB BSP ${BOARDS_ROOT}/${BOARD}/*.c*)
FILE(GLOB USRC src/*.c*)

target_include_directories(${COMPONENT_LIB} PRIVATE
	${CMAKE_BINARY_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
	${BOARDS_ROOT}/${BOARD}/
	
	${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_sources(${COMPONENT_LIB} PRIVATE
	${BSP}
	${USRC}
)
