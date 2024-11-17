cmake_minimum_required(VERSION 3.31.0)
set(CMAKE_MODULE_PATH "/home/xm/autonomy_ws/Open3D/3rdparty/CMake")
project(CheckASM_NASM ASM_NASM)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/result.cmake"
  "set(CMAKE_ASM_NASM_COMPILER \"${CMAKE_ASM_NASM_COMPILER}\")\n"
  
  )