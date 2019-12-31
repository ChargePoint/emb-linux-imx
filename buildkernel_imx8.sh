#! /bin/bash
source ../toolchain64/environment-setup-aarch64-poky-linux
unset LDFLAGS
cores=`cat /proc/cpuinfo | grep processor | wc -l`
threads=`expr $cores + 2`
ARCHTYPE=arm64
mkdir -p build64
make ARCH=$ARCHTYPE mrproper
make ARCH=$ARCHTYPE distclean
make O=build ARCH=$ARCHTYPE ucb_defconfig
make O=build ARCH=$ARCHTYPE -j $threads
