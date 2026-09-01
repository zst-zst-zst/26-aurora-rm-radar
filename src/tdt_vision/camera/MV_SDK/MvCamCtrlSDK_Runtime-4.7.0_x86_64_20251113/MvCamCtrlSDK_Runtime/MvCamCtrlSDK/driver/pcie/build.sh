#!/bin/sh

################################################################################
#
# build.sh
# for build gev/cxp/cmlframegrabber.ko
#
################################################################################
DIRNAME=$(cd `dirname $0`; pwd)
MVS_HOME=$DIRNAME
TRA_PATH=$MVS_HOME/TransportLayer
OUT_PATH=$MVS_HOME
PLATFORM=$(uname)

# pcie array
PCIE_ARRAY=("gev" "cxp" "xof" "cml" "virtualserial" "lightcontroller")

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

if [ ${PLATFORM} = "rk3588" ]; then
	PCIE_ARRAY=("gev" "cml" "virtualserial")
fi

KERENL_SRC=/lib/modules/`uname -r`/build

if [ ${PLATFORM} != "rk3588" ]; then
	if [ ! -d $KERENL_SRC ]; then
		echo ""
		echo "$KERENL_SRC isn't a directory "
		ErrorOut "No kernel source yet, please install kernel source " 1
	fi
fi

for pcie in ${PCIE_ARRAY[@]} 
do
	SRC_PATH=$TRA_PATH/$pcie
	
	MODULE_NAME=${pcie}framegrabber.ko

	if [ $pcie = "virtualserial" ]; then
		MODULE_NAME=mvfg${pcie}.ko
	fi

		if [ $pcie = "lightcontroller" ]; then
		MODULE_NAME=${pcie}.ko
	fi

	if [ -f $MVS_HOME/$MODULE_NAME ]; then
		continue
	fi

	if [ ${PLATFORM} != "rk3588" ]; then
		if [ -d "$SRC_PATH" ]; then
			make -C $SRC_PATH clean
			if [ $pcie = "xof" ]; then
				make -C $SRC_PATH XOF_SUPPORTED=Yes
			else
				make -C $SRC_PATH 
			fi
		fi
		if [ -f "$SRC_PATH/$MODULE_NAME" ]; then
			mv $SRC_PATH/$MODULE_NAME $MVS_HOME/$MODULE_NAME 
		else
			echo "*** Fail to create the module ${MODULE_NAME} ***" 
		fi
	else
		if [ -f "$SRC_PATH/$ARCH/$MODULE_NAME" ]; then
			mv $SRC_PATH/$ARCH/$MODULE_NAME $MVS_HOME/$MODULE_NAME 
		else
			echo "*** Fail to create the module ${MODULE_NAME} ***" 
		fi
	fi
done

