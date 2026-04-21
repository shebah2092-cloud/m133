# CMakeLists.txt for cross-compiling acados generated solvers (MPC + MHE) for ARM64 Android
#
# Usage:
#   mkdir build_arm64 && cd build_arm64
#   cmake -DCMAKE_TOOLCHAIN_FILE=<ndk>/build/cmake/android.toolchain.cmake \
#         -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-26 \
#         -DACADOS_INCLUDE_DIR=<acados>/include \
#         ..
#   make -j$(nproc)

cmake_minimum_required(VERSION 3.10)
project(m130_acados_solvers C)

# acados headers
if(NOT DEFINED ACADOS_INCLUDE_DIR)
    set(ACADOS_INCLUDE_DIR "/home/yoga/m13/acados-main/include")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -std=c99 -O2")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/acados
    ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${ACADOS_INCLUDE_DIR}/hpipm/include
)

# ========== MPC (Rocket) Solver ==========
set(MPC_SOURCES
    acados_solver_m130_rocket.c
    acados_sim_solver_m130_rocket.c
    m130_rocket_model/m130_rocket_expl_ode_fun.c
    m130_rocket_model/m130_rocket_expl_vde_forw.c
    m130_rocket_model/m130_rocket_expl_vde_adj.c
)

# ========== MHE Solver ==========
set(MHE_SOURCES
    acados_solver_m130_mhe.c
    acados_sim_solver_m130_mhe.c
    m130_mhe_model/m130_mhe_expl_ode_fun.c
    m130_mhe_model/m130_mhe_expl_vde_forw.c
    m130_mhe_model/m130_mhe_expl_vde_adj.c
    m130_mhe_cost/m130_mhe_cost_y_0_fun.c
    m130_mhe_cost/m130_mhe_cost_y_0_fun_jac_ut_xt.c
    m130_mhe_cost/m130_mhe_cost_y_0_hess.c
    m130_mhe_cost/m130_mhe_cost_y_fun.c
    m130_mhe_cost/m130_mhe_cost_y_fun_jac_ut_xt.c
    m130_mhe_cost/m130_mhe_cost_y_hess.c
)

# Combined static library
add_library(m130_solvers STATIC ${MPC_SOURCES} ${MHE_SOURCES})
