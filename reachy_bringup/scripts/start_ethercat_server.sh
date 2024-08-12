#! /bin/bash

cd $HOME/dev/poulpe_ethercat_controller
RUST_LOG=info ./target/release/server config/robot.yaml 