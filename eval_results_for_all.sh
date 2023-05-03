#!/bin/bash

set -e
export LD_LIBRARY_PATH=/usr/local/lib/

cd perf_tests_exps/t07
../../run_experiment t07
cd ../..

cd perf_tests_exps/t08
../../run_experiment t08
cd ../..

cd perf_tests_exps/t09
../../run_experiment t09
cd ../..

cd perf_tests_exps/t10
../../run_experiment t10
cd ../..


echo "JOBS DONE"