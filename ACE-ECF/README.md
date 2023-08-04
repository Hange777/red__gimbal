# ACE Embedded Common Framework
 ACE Embedded Common Framework 电控通用框架，hal库only


## 如何使用？

首次使用需要clone大的仓库到你的工程，然后更新子模块，在keil添加好路径即可。

```shell
git clone https://github.com/DGUT-ACE-EMBEDDED/ACE-ECF.git
```

```shell
cd /ACE-ECF
```

```shell
git submodule update --init --recursive --remote
```

### Bsp

在Inc里面，会有个Config，用于动态调整你的配置

## ACE-ECF 开发时间表

### V1

目标：完成所有Bsp编写，完成Control、Algorithm大部分

上线日期：**目前打算寒假留校前出**，为了能够更好的准备这赛季的代码通用适配



### V2

目标：适配ROS、C++面向对象的转移、优化细节、补充更多电控能操作的挽救措施（比如钩子函数）等等

上线日期：2023暑假猛肝





## ACE-ECF 版本详细和开发环境

### V1

1. Bsp
   1. Dr16
   2. Can
   3. Canfifo(是上面can的基础)
   4. Referee
   5. Usb_cdc
   6. Buzzer
   7. Motor_Encoder
   8. 
2. Control
   1. Referee (不知道怎么命名，就是裁判系统解码)
   2. （不知道怎么命名，就是板间通讯，不过这个建议就分兵种吧
   3. （遥控器数据处理，不知道你们有没有，工程有
   4. 
3. Algorithm
   1. PID
   2. fifo（这边可能用的是官方开源的，到时候你们可能要做一下适配或者直接cv官方
   3. filter（可以再细分，看你们意愿
   4. maths

### 开发环境

第一个版本还是以C板为开发板，具体配置看各个功能的readme



## 如何开发该框架？

注意！我们开发都是以子模块写完之后，push，之后在到大的仓库去更新子模块，再push，不会在大的仓库上改的。



### 子模块

[子模块设置(Git Submodule操作)_代码托管 CodeHub_用户指南_云端仓库管理_仓库管理_华为云 (huaweicloud.com)](https://support.huaweicloud.com/usermanual-codehub/codehub_ug_1001.html)

[Git submodule 子模块的管理和使用 - 简书 (jianshu.com)](https://www.jianshu.com/p/9000cd49822c)

更新子模块只需两步。
1. 子模块本地文件更新（子模块的main分支要先更新），`git submodule update --remote --recursive`
2. 子模块本地更新到云端，在github desktop 中commit、push


### config配置文件

以Bsp为例，可以看我们大的文件夹ECF里有个ECF_BspConfig.h，这个是对我们bsp的配置文件

配置文件可以参考各大框架（比如说FreeRTOS、hal库、espidf

这里我以espidf的为例

![image-20221115171933374](https://pic.imgdb.cn/item/63735a5116f2c2beb1563017.png)

这是他们的sdkconfig.h文件，命名规范是前面Config，后面就是具体的宏定义了。

具体到各个板间支持，比如说dr16，第一个就是全局开关，同时可以判断这个功能有没有开启

![image-20221115173410293](https://pic.imgdb.cn/item/63735dbd16f2c2beb15aa192.png)

如果定义为0则.c下面的全无



第二个是各个板子的串口有差异，需要定义串口的句柄

![image-20221115213421696](https://pic.imgdb.cn/item/6373960816f2c2beb1b3c7b9.png)



**我的c文件里就直接用宏定义了**

![image-20221115213315068](https://pic.imgdb.cn/item/637395cc16f2c2beb1b3845d.png)



还有其他类型的宏定义

![image-20221115214046301](https://pic.imgdb.cn/item/6373978916f2c2beb1b640fd.png)





其余的你们可以自己发挥，C和C++也不一样，可以参考其他大学开源或者开源库



### 目录结构

- Inc是头文件
- Src是源文件
- README 文件介绍
- ChangeLog 更新日志

### 文件结构规范

#### 头文件

C的大致包含：



1. Doxygen文档说明

2. ```
   #ifndef __文件名字大写_H
   #define __文件名字大写_H
   ```

3. 包含其他头文件include

4. 宏定义

5. 数据结构部分

   比如结构体、枚举、变量等等

6. 函数声明（不知道为什么别人喜欢加extern，我们也加

   

#### 源文件

1. Doxygen文档说明

2. 包含其他头文件include或者各种声明

3. 宏定义

4. 数据结构部分

   比如变量等等的声明

5. 函数

6. 反正按平时喜欢那种就差不多了



#### README文档

各个模块关系平行，每个模块的说明都从一个二号标题开始

需要包含的内容：

1. 如何使用此模块？
   1. 开启什么什么外设，cube那边怎么设置，包含截图
   2. 如果需要扔进it中断什么的，也说明
   3. 该模块的宏定义如何设置？
2. 结构体介绍
3. 函数介绍，如何使用？
4. 版本更新后如何迁移？（你们有可能改函数名或者结构体的嘛
5. 未完待续（你们补充捏，到时候改一下merge就行





#### ChangeLog文档

各个模块关系平行，每个模块的说明都从一个二号标题开始

需要包含的内容：

1. 三号标题日期 + 待办事项复选框
2. 已经修复的bug复选框
3. 我感觉我这个逻辑可能有点问题，到时候再改

### 代码规范

#### Doxygen文件信息注释



example：

```
/************************** Dongguan-University of Technology -ACE**************************
* @file bsp_can_fifo.h
* @brief
* @author pansyhou侯文辉 (1677195845lyb@gmail.com)
* @version 1.0
* @date 2022-07-24
*
*
* @history
* <table>
* Date       Version Author Description
* 2022-07-24   1.0   侯文辉
* @verbatim
* ==============================================================================
 * 文件内容捏  可以做介绍，函数、结构体那些、如何使用、如何理解
* ==============================================================================
* @endverbatim
************************** Dongguan-University of Technology -ACE***************************/


```

**vscode** doxygen配置文件（以我为例

```yaml
"doxdocgen.c.firstLine": "/************************** Dongguan-University of Technology -ACE**************************",
    "doxdocgen.c.lastLine": "************************** Dongguan-University of Technology -ACE***************************/",
    "doxdocgen.generic.authorEmail": "1677195845lyb@gmail.com",
    "doxdocgen.generic.authorName": "pansyhou侯文辉",
    // Doxygen documentation generator set
    // 文件注释：自定义模块，这里我添加一个修改日志
    "doxdocgen.file.customTag": [
    
        "@history",
        "<table>",
        "Date       Version Author Description",
        "{date}   1.0   侯文辉     ",
        "@verbatim ",
        "==============================================================================",
        "==============================================================================",
        "@endverbatim",
    ],
    // 文件注释的组成及其排序
    "doxdocgen.file.fileOrder": [
        "file", // @file
        "brief", // @brief 简介
        "author", // 作者
        "version", // 版本
        "date", // 日期
        "empty", // 空行
        "empty",
        "custom" // 自定义
    ],
    // 下面时设置上面标签tag的具体信息
    "doxdocgen.file.fileTemplate": "@file {name}",
    "doxdocgen.file.versionTag": "@version 1.0",
    "doxdocgen.generic.authorTag": "@author {author} ({email})",
    // 日期格式与模板
    "doxdocgen.generic.dateFormat": "YYYY-MM-DD",
    "doxdocgen.generic.dateTemplate": "@date {date}",
```



#### 基本命名

![image-20221115215451440](https://pic.imgdb.cn/item/63739add16f2c2beb1bac01c.png)



其实按我们平时正常来就好，有问题就下个版本大改

主要在函数和变量命名规范上。

函数能学hal库的学hal库

HAL_GPIO_功能....

**大概是以ECF+外设+功能**

### 写完之后如何提交？

很好的问题

你先下好github desktop

然后clone你要编写的子仓库

改完之后，理论上应该是push？我也没试过多人协作，先试试呗

然后我这边merge还是怎样就能更新了。

### 子模块管理

目前是由我pansy来负责大版本的更新，其实你们也可以自己在子模块慢慢写。

### 其他事项

我们开发的时候尽量看一些开源，不要自己硬想，比如有我们的robomaster icra的，还有官方standrobot，里面都很值得参考一下的。



## 版本管理

主要开发分支都在main，后期出V1之后会单独把V1开出来一个新的分支，后面要是V1分支有问题那就在那小修小补出V1.1之类的补丁版本，分出来后记得发布一次release。

