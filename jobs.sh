#!/bin/bash

set -e
export LD_LIBRARY_PATH=/usr/local/lib/

./eval_config_two_vehicles_leavable_dual_rev 25 25 50 50 2 2
./eval_config_two_vehicles_leavable_dual_rev 25 25 50 50 3 3
./eval_config_two_vehicles_leavable_dual_rev 25 25 50 50 1 1
./eval_config_two_vehicles_leavable_dual_rev 70 70 50 50 1 1
./eval_config_two_vehicles_leavable_dual_rev 10 10 40 40 1 1
./eval_config_two_vehicles_leavable_dual_rev 5 5 40 40 1 1
./eval_config_two_vehicles_leavable_dual_rev 5 5 40 40 5 5
./eval_config_two_vehicles_leavable_dual_rev 5 5 40 40 9 9

echo "JOBS DONE"