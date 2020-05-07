# WAM_Ikine_HandGet_UDP
* Editor: LuoZu
* 2020-05-07

> SDK list: LeapSDK3.2.1 Eigen3

> Devices: LeapMotion

> Configuration: 
>> platform: Win32
>> Win SDK: 10.0.16299.0
## Eigen3 SDK Set

1. VC++目录->常规
* 包含目录： 		...\SDK;
2. C/C++ -> 常规： 
* 附加包含目录：	...\SDK;

## Leap SDK Set
1. VC++目录->常规：
* 可执行文件目录：	...\SDK\LeapSDK\lib\x86;
* 包含目录： 		...\SDK;
* 库目录：		...\SDK\LeapSDK\lib\x86;

2. C/C++ -> 常规： 
* 附加包含目录：	...\SDK;

3. 链接器 -> 常规： 
* 附加库目录：	...\SDK\LeapSDK\lib;
* 输入：		Leap.lib

## 说明
1. WAM_Ikine_HandGet_UDP 增加了WAMIkine的注释
2. WAM_Ikine_HandGet_UDP0319 离线Leap Motion 求解
3. LM_HandGet 采集手部掌心和拇指、食指及中指位置，笛卡尔空间调整为世界坐标
## 如果设备不响应：
Leap Motion Service not running

1. Win + R 
2. services.msc
3. find Leap src
	running