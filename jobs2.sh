#!/bin/bash

set -e
export LD_LIBRARY_PATH=/usr/local/lib/

cd perf_tests_feas/F1
../../build/run_experiment F1
cd ../..

cd perf_tests_feas/F2
../../build/run_experiment F2
cd ../..

cd perf_tests_feas/F3
../../build/run_experiment F3
cd ../..

cd perf_tests_feas/F4
../../build/run_experiment F4
cd ../..

cd perf_tests_feas/F5
../../build/run_experiment F5
cd ../..

cd perf_tests_feas/F6
../../build/run_experiment F6
cd ../..

echo "JOBS DONE"