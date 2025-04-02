idf_component_register(SRCS "hooks.c")

# We need to force GCC to integrate this static library into the
# bootloader link. Indeed, by default, as the hooks in the bootloader are weak,
# the linker would just ignore the symbols in the extra. (i.e. not strictly
# required)
# To do so, we need to define the symbol (function) `bootloader_hooks_include`
# within hooks.c source file.

if(NOT BOARD)
	set(BOARD esp32s2_rc1)
	message(WARNING "no BOARD set. Set to ${BOARD}")
endif()

set(BOARDS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../boards)

target_include_directories(${COMPONENT_LIB} PRIVATE
	${CMAKE_BINARY_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
	${BOARDS_ROOT}/${BOARD}/
)