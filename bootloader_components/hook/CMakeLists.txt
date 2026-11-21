idf_component_register(SRCS "hooks.c")

# We need to force GCC to integrate this static library into the
# bootloader link. Indeed, by default, as the hooks in the bootloader are weak,
# the linker would just ignore the symbols in the extra. (i.e. not strictly
# required)
# To do so, we need to define the symbol (function) `bootloader_hooks_include`
# within hooks.c source file.

if(NOT BOARD AND SDKCONFIG)
	# written by top-level CMakeLists.txt into the app build dir
	get_filename_component(_app_build_dir "${SDKCONFIG}" DIRECTORY)
	include("${_app_build_dir}/board.cmake" OPTIONAL)
endif()
if(NOT BOARD)
	message(FATAL_ERROR "bootloader hook: BOARD is not known (no board.cmake next to ${SDKCONFIG})")
endif()
message(STATUS "bootloader hook: BOARD=${BOARD}")

set(BOARDS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../boards)

target_include_directories(${COMPONENT_LIB} PRIVATE
	${CMAKE_BINARY_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
	${BOARDS_ROOT}/${BOARD}/
)