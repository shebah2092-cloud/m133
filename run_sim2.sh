#!/bin/bash
export LD_LIBRARY_PATH=/home/yoga/m13/acados-main/lib
export ACADOS_SOURCE_DIR=/home/yoga/m13/acados-main
cd /home/yoga/m13
SIMFILE=$(find /home/yoga/m13 -name 'rocket_6dof_sim.py' -not -path '*v2*' -type f | head -1)
exec python3 "$SIMFILE"
