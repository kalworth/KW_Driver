# KW_Driver

<p align="center">
  <img src="https://img.shields.io/badge/MCU-STM32G431RBTx-2f6fed" alt="MCU">
  <img src="https://img.shields.io/badge/Driver-DRV8323-2ea44f" alt="DRV8323">
  <img src="https://img.shields.io/badge/Control-FOC%20Current%20Loop-f59e0b" alt="FOC Current Loop">
  <img src="https://img.shields.io/badge/Protocol-MIT-111827" alt="MIT">
</p>

基于 `STM32G431 + DRV8323` 的开源无刷电机驱动器固件。

当前版本以 `FOC 电流控制` 为核心，通信接口采用 `MIT 协议`，适用于自研驱动板调试、小型伺服驱动实验与相关二次开发。

## 概述

项目包含完整的驱动器基础链路：

- 三相 PWM 驱动
- 双分流电流采样
- SPI 编码器反馈
- DRV8323 初始化与故障检测
- FOC 电流环
- 串口菜单调试
- Flash 参数保存
- CAN 通信

当前 README 仅描述已确认可用的能力。

## 当前功能

- FOC 电流环运行
- 编码器采样与电角度计算
- DRV8323 寄存器配置与 fault 检测
- 串口参数查看、修改与保存
- 标定流程
- CAN 接口
- MIT 协议控制
- 状态反馈

## 硬件开源

硬件设计开源地址：

- [立创开源硬件平台 - 自制FOC驱动器](https://oshwhub.com/rotay/dgm-qu-dong-qi-fu-ke-v1-2)

该硬件页面给出的项目信息包括：

- 工程名称：`自制FOC驱动器`
- 开源协议：`GPL 3.0`

## 硬件基础

当前工程对应的主要硬件如下：

- MCU：`STM32G431RBTx`
- Gate Driver：`DRV8323`
- PWM：`TIM1`
- 编码器：`SPI 绝对值编码器`
- 调试串口：`USART2 @ 115200`
- CAN：`FDCAN1`

代码中可确认的关键参数：

- PWM 频率：`25kHz`
- 控制周期：`40us`
- 分流电阻：`5mΩ`
- 电流采样增益：`20`

## 快速开始

### 1. 打开工程

- Keil：`MDK-ARM/dgm_v1.2.uvprojx`
- CubeMX：`dgm_v1.2.ioc`

### 2. 烧录固件

连接电机、编码器、功率板、串口与 CAN 后，将固件下载到目标板。

### 3. 串口连接

默认串口参数：

```text
115200 / 8N1
```

### 4. 标定并保存

串口输入：

```text
c
```

标定完成后输入：

```text
u
```

### 5. 控制接口

当前推荐通过 `CAN + MIT 协议` 进行控制。

## 串口命令

| 命令 | 说明 |
| --- | --- |
| `r` | 系统复位 |
| `p` | 读取位置 |
| `x` | 查看配置 |
| `v` | 读取母线电压 |
| `m` | 进入电机模式 |
| `c` | 开始标定 |
| `z` | 清除错误 |
| `s` | 进入参数设置 |
| `u` | 保存配置 |
| `Esc` | 返回菜单 |

## 参数配置

参数支持串口在线修改，并保存到片内 Flash。

输入格式：

```text
key = value
```

示例：

```text
current_limit = 10
can_id = 3
pos_max = 3.14
vel_max = 30
iq_max = 8
```

保存命令：

```text
u
```

## 工程结构

```text
KW_Driver
├─ Core
│  ├─ Inc
│  └─ Src
├─ Drivers
├─ MDK-ARM
├─ 上位机
└─ dgm_v1.2.ioc
```

核心模块：

- `foc_handle.c`：FOC 电流环
- `foc_encoder.c`：编码器采样与角度计算
- `can.c`：CAN 与 MIT 协议接口
- `fsm.c`：串口菜单与状态机
- `calibration.c`：标定流程
- `usr_config.c`：参数管理与 Flash 保存
- `drv8323.c`：驱动器配置

## 说明

该仓库当前更适合作为一套可运行、可继续扩展的驱动器固件基础工程。

## 免责声明

本项目直接驱动电机功率级。调试不当可能导致电机突然转动、大电流、发热或器件损坏。实际使用时应配合限流、电源保护和机械安全措施。
