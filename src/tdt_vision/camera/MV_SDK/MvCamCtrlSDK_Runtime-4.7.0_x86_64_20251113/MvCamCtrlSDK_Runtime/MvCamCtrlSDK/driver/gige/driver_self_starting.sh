#!/bin/bash
DRIVER_PATH=$(cd `dirname $0`; pwd)
SDK_HOME=$(cd ${DRIVER_PATH}/../../; pwd)
SDK_PATH_OLD=$(cat ${DRIVER_PATH}/DriverServer | grep "^SDK_HOME=" | awk -F '=' '{print $2}')
FIND_PATH=${SDK_PATH_OLD//\//\\\/}
FIND_PATH=${FIND_PATH//\#/\\\#}
REPLACE_PATH=${SDK_HOME//\//\\\/}
USER_ID=`id -u`
PLATFORM=$(uname)
KERNEL_VERSION=$(uname -r)

# Check required priviledge
if [ "$USER_ID" != "0" ]; then
echo "DriverServer can only be installed by root user or sudoer"
exit 1
fi

if [ -f /etc/debian_version ]; then
    OS=Debian  # XXX or Ubuntu??
    VER=$(cat /etc/debian_version)
elif [ -f /etc/redhat-release ]; then
    # TODO add code for Red Hat and CentOS here
    OS=Redhat  # XXX or CentOs??
    VER=$(uname -r)
elif [ -f /etc/kylin-release ]; then
    OS=Kylin 
    VER=$(uname -r)
elif [ -f /etc/openEuler-release ]; then
    OS=OpenEuler 
    VER=$(uname -r)
else
    OS=$(uname -s)
    VER=$(uname -r)
fi

ARCH=$(uname -m)
if [ ${ARCH} = "aarch64" ]; then
	if grep -q "rk3588" /proc/device-tree/compatible; then
        PLATFORM="rk3588"
	fi
fi

# check parameter
if [ "1" = "$1" ]; then

sed -i "/^SDK_HOME=*/s/${FIND_PATH}/${REPLACE_PATH}/" ${SDK_HOME}/driver/gige/DriverServer
cp -f ${DRIVER_PATH}/DriverServer /etc/init.d/
if [ ${PLATFORM} = "rk3588" ]; then
    mv /etc/init.d/DriverServer /etc/init.d/S91DriverServer
	chmod 777 /etc/init.d/S91DriverServer
else
    chmod 777 /etc/init.d/DriverServer
fi

# 记录当前的内核版本
echo "${KERNEL_VERSION}" > ../kernel_version

if [ "Debian" = "$OS" ]; then
	update-rc.d -f DriverServer defaults 1> /dev/null
elif [ "Redhat" = "$OS" ]; then
	chkconfig --add DriverServer
	chkconfig DriverServer on
elif [ "Kylin" = "$OS" ] || [ "OpenEuler" = "$OS" ]; then
    systemctl enable DriverServer
    systemctl start DriverServer
fi

if [ ${PLATFORM} = "rk3588" ]; then
	/etc/init.d/S91DriverServer start
else
	/etc/init.d/DriverServer start
fi

elif [ "0" = "$1" ]; then

if [ "Debian" = "$OS" ]; then
    update-rc.d -f DriverServer remove 1> /dev/null
elif [ "Redhat" = "$OS" ]; then
    chkconfig DriverServer off
    chkconfig --del DriverServer 
elif [ "Kylin" = "$OS" ] || [ "OpenEuler" = "$OS" ]; then
    systemctl stop DriverServer
    systemctl disable DriverServer
fi

if [ -f /etc/init.d/DriverServer ]; then
rm /etc/init.d/DriverServer
fi

else
 	echo "wrong parameter"
fi

