#! /bin/bash
source ../toolchain64/environment-setup-aarch64-poky-linux
unset LDFLAGS
cores=`cat /proc/cpuinfo | grep processor | wc -l`
threads=`expr $cores + 2`
ARCHTYPE=arm64
OUTDIR=build64
mkdir -p $OUTDIR
make O=$OUTDIR ARCH=$ARCHTYPE mrproper
make O=$OUTDIR ARCH=$ARCHTYPE distclean
make O=$OUTDIR ARCH=$ARCHTYPE ucb_defconfig
make O=$OUTDIR ARCH=$ARCHTYPE -j $threads
