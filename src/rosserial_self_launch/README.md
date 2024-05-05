# rosserial启动包

tips:rosserial仅支持ros1

#### [rosserial](https://github.com/ros-drivers/rosserial/tree/noetic-devel)基本介绍：

rosserial是一个专门用于解决与单片机等无法直接运行linux的嵌入式设备进行ROS相关通讯的包。

该包运行在PC端，通过启动一个**中继节点**，来沟通ROS与单片机等无法直接运行ROS或有其他用途的嵌入式设备，该节点通过串口或tcp方式与其他嵌入式设备通讯。

#### 包功能：

该包基于rosserial，包含一个启动（launch）文件来启动rosserial。

## 0.前提

#### 首先需要安装**rosserial**

终端输入

```bash
rospack list | grep rosserial
```

若输出如下

```
rosserial_client /opt/ros/noetic/share/rosserial_client
rosserial_msgs /opt/ros/noetic/share/rosserial_msgs
rosserial_python /opt/ros/noetic/share/rosserial_python
```

则说明已经安装过

否则需要安装rosserial，若ros是noetic版本，则输入

```
sudo apt install ros-noetic-rosserial
```



## 1.启动rosserial

将该包放入工作空间相关目录下

首先需要修改[启动文件](./launch/start_rosserial.launch)

通常情况下只需要修改`port_name`和`baud_value`即可，具体修改看启动文件注释

修改完毕之后，启动rosserial

```bash
roslaunch rosserial_self_launch start_rosserial.launch
```

这样在PC端的`rosserial`已经启动



## 2.使用

### 2.1 启动文件说明：

该包并没有任何可执行文件代码，仅仅是使用了一个启动文件，因此只要保证安装了`rosserial`功能包，启动文件可以任意复制到其他功能包。

### 2.2 找不到串口的问题

若是此时上位机已经与单片机等设备通过串口连接，但是启动后提示如下内容，报告dev设备权限不够问题。

**例如指定的端口为/dev/ttyUSB0**

```
[ERROR] [1704040608.553387]: Error opening serial: [Errno 2] could not open port /dev/ttyUSB0: [Errno 2] No such file or directory: '/dev/ttyUSB0'
```

此时给予权限即可

```
sudo chmod 777 /dev/ttyUSB0
```

不过上述方法只是临时性的解决dev设备权限问题，一旦重新插拔接口或重启电脑等情况，权限又消失了，需要重新给予权限。

为了方便使用，建议永久性地解决dev设备权限问题，提供一个参考博客：

https://blog.csdn.net/HuangChen666/article/details/125626570

### 2.3 无法接收话题、服务、动作的问题

这里就没有必要复述硬件连接以及下位机编写代码等非ROS端造成的原因。

这里解决由于话题、服务、动作所在的功能包未包含在rosserial启动时造成接收无法接收消息的问题。

例如当我们在自定义功能包`rosserial_self_msg`中自定义话题消息`tset`，下位机已经接入ROS作为一个节点，发布该功能包的自定义消息。这时，我们发现ROS端没法接收到类型为`/rosserial_self_msg/test`的话题消息。

造成这个的原因是，在PC端启动`rosserial`的环境变量中没有功能包`rosserial_self_msg`，因此`rosserial`无法识别的消息类型`/rosserial_self_msg/test`

。

这时关闭PC端的`rosserial`，将功能包`rosserial_self_msg`添加进环境变量中，再次开启`rosserial`，这样就能正常接收`/rosserial_self_msg/test`消息类型的数据

