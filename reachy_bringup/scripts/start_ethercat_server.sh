#! /bin/bash

cd $HOME/dev/poulpe_ethercat_controller
RUST_LOG=info cargo run --release config/orbitas.yaml