############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Paths to acados and generated solver code
# Use environment variables if set, otherwise fall back to defaults
if(DEFINED ENV{ACADOS_ROOT})
	set(ACADOS_ROOT $ENV{ACADOS_ROOT})
else()
	set(ACADOS_ROOT "/home/yoga/m13/acados-main")
endif()

if(DEFINED ENV{M130_SOLVER_ROOT})
	set(SOLVER_ROOT $ENV{M130_SOLVER_ROOT})
else()
	set(SOLVER_ROOT "/home/yoga/m13/c_generated_code")
endif()

# Collect generated solver C sources
set(SOLVER_SOURCES
	${SOLVER_ROOT}/acados_solver_m130_rocket.c
	${SOLVER_ROOT}/acados_solver_m130_mhe.c
	${SOLVER_ROOT}/acados_sim_solver_m130_rocket.c
	${SOLVER_ROOT}/acados_sim_solver_m130_mhe.c
	${SOLVER_ROOT}/m130_rocket_model/m130_rocket_expl_ode_fun.c
	${SOLVER_ROOT}/m130_rocket_model/m130_rocket_expl_vde_adj.c
	${SOLVER_ROOT}/m130_rocket_model/m130_rocket_expl_vde_forw.c
	${SOLVER_ROOT}/m130_mhe_model/m130_mhe_expl_ode_fun.c
	${SOLVER_ROOT}/m130_mhe_model/m130_mhe_expl_vde_adj.c
	${SOLVER_ROOT}/m130_mhe_model/m130_mhe_expl_vde_forw.c
	${SOLVER_ROOT}/m130_mhe_cost/m130_mhe_cost_y_0_fun.c
	${SOLVER_ROOT}/m130_mhe_cost/m130_mhe_cost_y_0_fun_jac_ut_xt.c
	${SOLVER_ROOT}/m130_mhe_cost/m130_mhe_cost_y_0_hess.c
	${SOLVER_ROOT}/m130_mhe_cost/m130_mhe_cost_y_fun.c
	${SOLVER_ROOT}/m130_mhe_cost/m130_mhe_cost_y_fun_jac_ut_xt.c
	${SOLVER_ROOT}/m130_mhe_cost/m130_mhe_cost_y_hess.c
)

# Build solver sources as a static library (they are plain C, not PX4 module code)
add_library(m130_solvers STATIC ${SOLVER_SOURCES})
target_include_directories(m130_solvers PUBLIC
	${SOLVER_ROOT}
	${SOLVER_ROOT}/m130_rocket_model
	${SOLVER_ROOT}/m130_mhe_model
	${SOLVER_ROOT}/m130_mhe_cost
	${ACADOS_ROOT}/include
	${ACADOS_ROOT}/include/acados
	${ACADOS_ROOT}/include/blasfeo/include
	${ACADOS_ROOT}/include/hpipm/include
)
target_link_libraries(m130_solvers PUBLIC
	${ACADOS_ROOT}/lib/libacados.so
	${ACADOS_ROOT}/lib/libhpipm.so
	${ACADOS_ROOT}/lib/libblasfeo.so
	m
)
# Suppress warnings in generated code
target_compile_options(m130_solvers PRIVATE -w)

px4_add_module(
	MODULE modules__rocket_mpc
	MAIN rocket_mpc
	STACK_MAX 8192
	SRCS
		RocketMPC.cpp
		mpc_controller.cpp
		mhe_estimator.cpp
		los_guidance.cpp
		sensor_bridge.cpp
	INCLUDES
		${SOLVER_ROOT}
		${ACADOS_ROOT}/include
		${ACADOS_ROOT}/include/acados
		${ACADOS_ROOT}/include/blasfeo/include
		${ACADOS_ROOT}/include/hpipm/include
	DEPENDS
		mathlib
		px4_work_queue
	)

target_link_libraries(modules__rocket_mpc PRIVATE m130_solvers)
