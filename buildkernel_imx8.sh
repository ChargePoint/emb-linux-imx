#! /bin/bash
source ../toolchain/environment-setup-aarch64-poky-linux
unset LDFLAGS
cores=`cat /proc/cpuinfo | grep processor | wc -l`
threads=`expr $cores + 2`
ARCHTYPE=arm64
mkdir -p build
make ARCH=$ARCHTYPE mrproper
make ARCH=$ARCHTYPE distclean
#make O=build ARCH=arm imx_v7_pbc_defconfig
make O=build ARCH=$ARCHTYPE defconfig
make O=build ARCH=$ARCHTYPE -j $threads
#make O=build ARCH=$ARCHTYPE dtbs -j $threads
make O=build ARCH=$ARCHTYPE modules -j $threads
