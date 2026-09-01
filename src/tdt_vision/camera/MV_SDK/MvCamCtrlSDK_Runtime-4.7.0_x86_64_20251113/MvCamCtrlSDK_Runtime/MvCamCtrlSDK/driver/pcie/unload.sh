#!/bin/sh

################################################################################
#
# unload.sh
# for gev/cxp/cmlframegrabber.ko unload
#
################################################################################

# pcie array
PCIE_ARRAY=("gev" "cxp" "xof" "cml" "virtualserial" "lightcontroller")
PLATFORM=$(uname)

# Display the help for this script
DisplayHelp()
{
    echo ""
    echo "NAME"
    echo "    unload.sh - Unload For mvfg gev driver module "
    echo "                gevframegrabber.ko"
    echo ""
    echo "SYNOPSIS"
    echo "    bash unload.sh [--help]"
    echo ""
    echo "DESCRIPTION"
    echo "    Unload the eBUS Universal Pro For Ethernet module and remove the configure"
	echo "    from the system to be ready to use"
    echo "    This script can only used by root or sudoer"
    echo "    --help             Display this help"
    echo ""
}

# Print out the error and exit 
#  $1 Error message
#  $2 Exit code
ErrorOut()
{
	echo ""
	echo "Error: $1"
	echo ""
	exit $2
}

ARCH=$(uname -m)
if [ ${ARCH} = "aarch64" ]; then
	if grep -q "rk3588" /proc/device-tree/compatible; then
        PLATFORM="rk3588"
	fi
fi

if [ ${PLATFORM} = "rk3588" ]; then
	PCIE_ARRAY=("gev" "cml" "virtualserial")
fi

# Parse the input arguments
for i in $*
do
    case $i in        
        --help)
            DisplayHelp
            exit 0
        ;;
        *)
        # unknown option
        DisplayHelp
        exit 1
        ;;
    esac
done

# Check required priviledge
if [ `whoami` != root ]; then
	ErrorOut "This script can only be run by root user or sudoer" 1
fi

if [ ${PLATFORM} != "rk3588" ]; then
    # save virnic static config
    if [ -f "./static_virnic_config.ini" ]; then
        echo -n > ./static_virnic_config.ini
    fi
    
    ./save_virnic_settings
fi

for pcie in ${PCIE_ARRAY[@]} 
do
    MODULE_NAME=${pcie}framegrabber
	if [ $pcie = "virtualserial" ]; then
		MODULE_NAME=mvfg${pcie}
	fi

    if [ $pcie = "lightcontroller" ]; then
		MODULE_NAME=${pcie}
	fi

    # Ensure the module is in memory
    IMAGEFILTER_LOADED=`lsmod | grep -o ${MODULE_NAME}`
    if [ "$IMAGEFILTER_LOADED" != "${MODULE_NAME}" ];then
	    continue
    fi

    # Unload the module
    if [ $pcie = "virtualserial" ]; then
        echo "Unloading MVFG Virtual Serial..."
    elif [ $pcie = "lightcontroller" ]; then
        echo "Unloading Light Controller..."
    else
        echo "Unloading MVFG ${pcie} Frame Grabber..."
    fi
    
    IMAGEFILTER_LOADED=`lsmod | grep ${MODULE_NAME} | cut -d ' ' -f1`
    /sbin/rmmod $IMAGEFILTER_LOADED $* 
done


