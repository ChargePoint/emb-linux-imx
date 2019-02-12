#! /bin/sh
source /opt/fsl-imx-fb/4.14-sumo/environment-setup-cortexa9hf-neon-poky-linux-gnueabi
cores=`cat /proc/cpuinfo | grep processor | wc -l`
threads=`expr $cores + 2`
mkdir -p build
make ARCH=arm mrproper
make ARCH=arm distclean
make O=build ARCH=arm imx_v7_defconfig
make O=build ARCH=arm zImage -j $threads
make O=build ARCH=arm dtbs -j $threads
make O=build ARCH=arm modules -j $threads
