#!/bin/bash

export LD_LIBRARY_PATH=/usr/local/lib/

cd build
./mp_generate_nlopt_mstart_staged

echo "eval_enterable"
./eval_enterable

echo "eval_leaveable"
./eval_leaveable

echo "JOBS DONE"