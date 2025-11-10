# components/libcorrect/CMakeLists.txt

# 1) Recolecta fuentes
file(GLOB_RECURSE LIBCORRECT_SRCS "${CMAKE_CURRENT_LIST_DIR}/src/*.c")

# 2) Define el include absolute
set(LIBCORRECT_INC "${CMAKE_CURRENT_LIST_DIR}/include")
message(STATUS ">>> libcorrect include dir: ${LIBCORRECT_INC}")

# 3) Registra el componente
idf_component_register(
  SRCS        ${LIBCORRECT_SRCS}
  INCLUDE_DIRS "${LIBCORRECT_INC}"
)