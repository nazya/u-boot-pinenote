#!/usr/bin/env sh
export CROSS_COMPILE=aarch64-linux-gnu-
make rk3566-pinenote_defconfig
./make.sh
./make.sh trust
