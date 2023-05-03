#!/bin/bash

set -e
export LD_LIBRARY_PATH=/usr/local/lib/

cd perf_tests_disc_set3/T1
../../build/run_experiment T1
cd ../..

cd perf_tests_disc_set3/T2
../../build/run_experiment T2
cd ../..

cd perf_tests_disc_set3/T3
../../build/run_experiment T3
cd ../..

cd perf_tests_disc_set3/T4
../../build/run_experiment T4
cd ../..

cd perf_tests_disc_set3/T5
../../build/run_experiment T5
cd ../..

echo "JOBS DONE"