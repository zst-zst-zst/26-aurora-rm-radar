#!/bin/bash

DIRNAME=`dirname $0`
#PWD = `pwd`

cd $DIRNAME
source /etc/profile
MV_CAM_SDK_PATH_OLD=$MVCAM_SDK_PATH

if [ ! -n "${MV_CAM_SDK_PATH_OLD}" ]; then
read -p "Please Input Install Path, Default Path is [/opt]: " InstallPath
echo -e "\n"
if [ ! -n "${InstallPath}" ]; then
	InstallPath=/opt
fi
echo "The install path is:$InstallPath"
#sed -i "s/export LD_LIBRARY_PATH/#export LD_LIBRARY_PATH/g" ~/.bashrc
source ~/.bashrc
if [ ! -d "${InstallPath}" ]; then
	mkdir -p ${InstallPath}
fi
if [ ! -d "${InstallPath}/MvCamCtrlSDK" ]; then
	echo "Install MvCamCtrlSDK, Please wait..."
	tar -C ${InstallPath} -xzf ./MvCamCtrlSDK_Runtime.tar.gz
else
	echo "Uninstall MvCamCtrlSDK, Please wait..."
	rm -rf ${InstallPath}/MvCamCtrlSDK
	echo "Install MvCamCtrlSDK, Please wait..."
	tar -C ${InstallPath} -xzf ./MvCamCtrlSDK_Runtime.tar.gz
fi
CONFIG_NAME=$(sed -n '1p'  ./Config.ini | tr -d '\r\n')
VERSION=$(sed -n '2p' ./Config.ini | tr -d '\r\n')
else
	echo "Already install MvCamCtrlSDK in ${MV_CAM_SDK_PATH_OLD}, Install New MvCamCtrlSDK, Please wait..."
	tar -xzf ./MvCamCtrlSDK_Runtime.tar.gz
if [ -d "${MV_CAM_SDK_PATH_OLD}/logserver" ]; then
	rm -rf ${MV_CAM_SDK_PATH_OLD}/logserver
fi
if [ -d "${MV_CAM_SDK_PATH_OLD}/driver" ]; then
	rm -rf ${MV_CAM_SDK_PATH_OLD}/driver
fi
if [ -d "${MV_CAM_SDK_PATH_OLD}/lib" ]; then
	rm -rf ${MV_CAM_SDK_PATH_OLD}/lib
fi
if [ -d "${MV_CAM_SDK_PATH_OLD}/license" ]; then
	rm -rf ${MV_CAM_SDK_PATH_OLD}/license
fi
if [ -e "${MV_CAM_SDK_PATH_OLD}/ReleaseNote_CH.txt" ]; then
	rm -rf ${MV_CAM_SDK_PATH_OLD}/ReleaseNote*
fi
	read -p "Please Input Install Path, Default Path is [/opt]: " InstallPath
	echo -e "\n"
	if [ ! -n "${InstallPath}" ]; then
		InstallPath=/opt
	fi
	echo "The install path is:$InstallPath"

	CONFIG_NAME=$(sed -n '1p'  ./Config.ini | tr -d '\r\n')
    VERSION=$(sed -n '2p' ./Config.ini | tr -d '\r\n')

if [ ! -d "${InstallPath}/${CONFIG_NAME}" ]; then
	mkdir -p ${InstallPath}/${CONFIG_NAME}
fi
	mv MvCamCtrlSDK/logserver ${InstallPath}/${CONFIG_NAME}
	mv MvCamCtrlSDK/driver ${InstallPath}/${CONFIG_NAME}
	mv MvCamCtrlSDK/lib ${InstallPath}/${CONFIG_NAME}
	mv MvCamCtrlSDK/license ${InstallPath}/${CONFIG_NAME}

	if [ -e MvCamCtrlSDK/ReleaseNote_CH.txt ]; then
		mv MvCamCtrlSDK/ReleaseNote* ${InstallPath}/${CONFIG_NAME}
	fi
#MVFG
if [ -d "${MV_CAM_SDK_PATH_OLD}/MVFG/driver" ]; then
	rm -rf ${MV_CAM_SDK_PATH_OLD}/MVFG/driver
fi
if [ -d "${MV_CAM_SDK_PATH_OLD}/MVFG/logserver" ]; then
	if [ -f "${MV_CAM_SDK_PATH_OLD}/MVFG/logserver/RemoveServer.sh" ]; then
		# 新安装包中删除了MVFG日志服务 执行旧包中的服务卸载脚本再删除
		bash ${MV_CAM_SDK_PATH_OLD}/MVFG/logserver/RemoveServer.sh
	fi
	rm -rf ${MV_CAM_SDK_PATH_OLD}/MVFG/logserver
fi

if [ -d "${MV_CAM_SDK_PATH_OLD}/MVFG" ]; then
	rm -rf ${MV_CAM_SDK_PATH_OLD}/MVFG
fi

	mv MvCamCtrlSDK/MVFG ${InstallPath}/${CONFIG_NAME}

if [ -d "./MvCamCtrlSDK" ]; then
	rm -rf ./MvCamCtrlSDK
fi
fi

#Runtime包安装路径默认与软件相同
if [ -d "${InstallPath}/MvCamCtrlSDK" ]; then
	mv ${InstallPath}/MvCamCtrlSDK ${InstallPath}/${CONFIG_NAME}
fi
#
#

#source ~/.bashrc

echo "Set up the SDK environment..."
bash $DIRNAME/set_usb_priority.sh
bash $DIRNAME/set_virtualserial_priority.sh

if [ -d "${InstallPath}/${CONFIG_NAME}" ]; then
source $DIRNAME/set_env_path.sh ${InstallPath}/${CONFIG_NAME} ${VERSION}
echo "source $DIRNAME/set_env_path.sh ${InstallPath}/${CONFIG_NAME}  ${VERSION}"
if [ -f ${InstallPath}/${CONFIG_NAME}/driver/gige/unload.sh ]; then
	bash ${InstallPath}/${CONFIG_NAME}/driver/gige/unload.sh
fi
if [ -f ${InstallPath}/${CONFIG_NAME}/driver/pcie/unload.sh ]; then
	bash ${InstallPath}/${CONFIG_NAME}/driver/pcie/unload.sh
fi
if [ -f ${InstallPath}/${CONFIG_NAME}/driver/gige/driver_self_starting.sh ]; then
    ${InstallPath}/${CONFIG_NAME}/driver/gige/driver_self_starting.sh 1
fi
if [ -f ${InstallPath}/${CONFIG_NAME}/driver/pcie/driver_self_starting.sh ]; then
    ${InstallPath}/${CONFIG_NAME}/driver/pcie/driver_self_starting.sh 1
fi
if [ -f script_self_starting.sh ]; then
    ./script_self_starting.sh 1
fi
cd ${InstallPath}/${CONFIG_NAME}/logserver
./RemoveServer.sh
./InstallServer.sh
fi


cd -

echo "Install MvCamCtrlSDK complete!"
echo "Tips: You should be launch a new terminal or execute source command for the bash environment!"
cd $PWD



