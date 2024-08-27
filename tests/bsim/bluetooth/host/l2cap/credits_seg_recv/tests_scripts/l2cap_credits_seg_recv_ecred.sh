#!/usr/bin/env bash
# Copyright (c) 2023 Nordic Semiconductor
# SPDX-License-Identifier: Apache-2.0

source ${ZEPHYR_BASE}/tests/bsim/sh_common.source

verbosity_level=2
simulation_id=$(guess_test_long_name)_ecred
bsim_exe=./bs_${BOARD_TS}_$(guess_test_long_name)_prj_conf

cd ${BSIM_OUT_PATH}/bin

Execute "${bsim_exe}" -v=${verbosity_level} -s=${simulation_id} -d=0 -rs=420 \
    -testid=ecred/central
Execute "${bsim_exe}" -v=${verbosity_level} -s=${simulation_id} -d=1 -rs=100 \
    -testid=ecred/peripheral

Execute ./bs_2G4_phy_v1 -v=${verbosity_level} -s=${simulation_id} -D=2 -sim_length=30e6 $@

wait_for_background_jobs
