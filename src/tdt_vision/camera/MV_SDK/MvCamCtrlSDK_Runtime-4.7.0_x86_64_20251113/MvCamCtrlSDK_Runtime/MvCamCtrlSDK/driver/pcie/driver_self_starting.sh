#!/bin/bash
DRIVER_PATH=$(cd `dirname $0`; pwd)
SDK_HOME=$(cd ${DRIVER_PATH}/../../; pwd)
SDK_PATH_OLD=$(cat ${DRIVER_PATH}/FGDriverServer | grep "^SDK_HOME=" | awk -F '=' '{print $2}')
FIND_PATH=${SDK_PATH_OLD//\//\\\/}
FIND_PATH=${FIND_PATH//\#/\\\#}
REPLACE_PATH=${SDK_HOME//\//\\\/}
USER_ID=`id -u`
PLATFORM=$(uname)
KERNEL_VERSION=$(uname -r)

# Check required priviledge
if [ "$USER_ID" != "0" ]; then
echo "FGDriverServer can only be installed by root user or sudoer"
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

sed -i "/^SDK_HOME=*/s/${FIND_PATH}/${REPLACE_PATH}/" ${SDK_HOME}/driver/pcie/FGDriverServer
cp -f ${DRIVER_PATH}/FGDriverServer /etc/init.d/
if [ ${PLATFORM} = "rk3588" ]; then
    mv /etc/init.d/FGDriverServer /etc/init.d/S91FGDriverServer
	chmod 777 /etc/init.d/S91FGDriverServer
else
    chmod 777 /etc/init.d/FGDriverServer
fi

# 记录当前的内核版本
echo "${KERNEL_VERSION}" > ../kernel_version

if [ "Debian" = "$OS" ]; then
	update-rc.d -f FGDriverServer defaults 1> /dev/null
elif [ "Redhat" = "$OS" ]; then
	chkconfig --add FGDriverServer
	chkconfig FGDriverServer on
elif [ "Kylin" = "$OS" ] || [ "OpenEuler" = "$OS" ]; then
    systemctl enable FGDriverServer
    systemctl start FGDriverServer
fi

if [ ${PLATFORM} = "rk3588" ]; then
	/etc/init.d/S91FGDriverServer start
else
	/etc/init.d/FGDriverServer start
fi

elif [ "0" = "$1" ]; then

if [ "Debian" = "$OS" ]; then
    update-rc.d -f FGDriverServer remove 1> /dev/null
elif [ "Redhat" = "$OS" ]; then
    chkconfig FGDriverServer off
    chkconfig --del FGDriverServer 
elif [ "Kylin" = "$OS" ] || [ "OpenEuler" = "$OS" ]; then
    systemctl stop FGDriverServer
    systemctl disable FGDriverServer
fi

rm /etc/init.d/FGDriverServer

else
 	echo "wrong parameter"
fi

