#!/bin/sh

################################################################################
#
# build.sh
# for build gevfilter.ko
#
################################################################################
DIRNAME=$(cd `dirname $0`; pwd)
DRIVER_HOME=$DIRNAME
SRC_PATH=$DRIVER_HOME/TransportLayer
OUT_PATH=$DRIVER_HOME
MODULE_NAME=gevfilter.ko
PLATFORM=$(uname)

#Print out the error and exit 
# $1 Error message
# $2 Exit code
ErrorOut()
{
	echo ""
	echo "Error: $1"
	echo ""
	exit $2
}

# Check required priviledge
# if [ `whoami` != root ]; then
	# ErrorOut "This script can only be run by root user or sudoer" 1
# fi

# check the arch
if [ "`uname -m`" = "x86_64" ]\
   || [ "`uname -m`" = "i386" ]\
   || [ "`uname -m`" = "aarch64" ]\
   || [ "`uname -m`" = "armv7l" ]\
   || [ "`uname -m`" = "i686" ]
then
	echo "The arch is: `uname -m`"
else
	ErrorOut "Driver Not Support the arch `uname -m` " 1
fi

ARCH=$(uname -m)
if [ ${ARCH} = "aarch64" ]; then
	if grep -q "rk3588" /proc/device-tree/compatible; then
        PLATFORM="rk3588"
	fi
fi

KERENL_SRC=/lib/modules/`uname -r`/build

if [ ${PLATFORM} != "rk3588" ]; then
	if [ ! -d $KERENL_SRC ]; then
		echo ""
		echo "$KERENL_SRC isn't a directory "
		ErrorOut "No kernel source yet, please install kernel source " 1
	fi

	if [ -d "$SRC_PATH" ]; then
		make -C $SRC_PATH clean
		make -C $SRC_PATH
	fi

	if [ -f "$SRC_PATH/$MODULE_NAME" ]; then
		mv $SRC_PATH/$MODULE_NAME $DRIVER_HOME/$MODULE_NAME 
	else
		ErrorOut "*** Fail to create the module gevfilter.ko ***" 1
	fi
else
	if [ -f "$SRC_PATH/$ARCH/$MODULE_NAME" ]; then
		mv $SRC_PATH/$ARCH/$MODULE_NAME $MVS_HOME/$MODULE_NAME 
	else
		echo "*** Fail to create the module ${MODULE_NAME} ***" 
	fi
fi



