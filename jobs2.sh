#!/bin/bash

set -e
export LD_LIBRARY_PATH=/usr/local/lib/

cd perf_tests_exps_2/V1
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V2
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V3
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V4
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V5
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V6
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V7
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V8
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V9
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V10
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V11
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V12
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V13
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V14
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V15
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V16
../../build/run_experiment 
cd ../..

cd perf_tests_exps_2/V17
../../build/run_experiment 
cd ../..


echo "JOBS DONE"