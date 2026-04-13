# KW_Driver

<p align="center">
  <img src="https://img.shields.io/badge/MCU-STM32G431RBTx-2f6fed" alt="MCU">
  <img src="https://img.shields.io/badge/Gate%20Driver-DRV8323-2ea44f" alt="DRV8323">
  <img src="https://img.shields.io/badge/Control-FOC%20Current%20Loop-f59e0b" alt="Control">
  <img src="https://img.shields.io/badge/Protocol-CAN%20MIT-111827" alt="MIT">
  <img src="https://img.shields.io/badge/Status-Active%20Development-dc2626" alt="Status">
</p>

基于 `STM32G431 + DRV8323` 的开源无刷电机驱动器固件，面向自研驱动板与机器人关节驱动实验。

这个仓库当前的核心目标很明确：

- 做好 `FOC 电流环`
- 跑通 `CAN MIT 协议`
- 保留一套可调试、可标定、可继续扩展的驱动器固件框架

> 当前版本对外只承诺两项能力：`电流控制链路可用`、`MIT 协议接口可用`。  
> 仓库中虽然保留了速度、位置、梯形轨迹等代码框架，但这些模式目前**不作为已完成功能**写进能力承诺。

## 特性

- 基于 `STM32G431RBTx` 的电机驱动器固件
- `DRV8323` 栅极驱动初始化与故障读取
- `25kHz` 三相 PWM 与 FOC 电流环
- 双分流电流采样
- SPI 绝对值编码器反馈
- 串口菜单调试、在线参数查看与修改
- 参数保存到片内 Flash，支持 CRC 校验
- `CAN MIT` 风格控制命令解析
- MIT 状态反馈回传
- 过压、欠压、超速、驱动器故障保护

## 当前能力边界

### 已完成

- FOC 电流环闭环
- 编码器采样、电角度计算与电流环运行链路
- DRV8323 初始化和 fault 检测
- 串口菜单调试
- 标定流程
- Flash 参数保存
- CAN 收发
- MIT 协议使能、控制帧解析、状态反馈

### 当前推荐使用方式

当前项目推荐按下面这条链路使用：

```text
上位主控 -> CAN MIT 命令 -> 驱动器 -> FOC q轴电流控制
```

也就是说，MIT 帧是对外接口，真正稳定的底层控制能力是 `q 轴电流控制`。

### 暂未承诺可用

- 速度模式
- 速度斜坡模式
- 位置模式
- 梯形轨迹位置模式

这些模块代码仍保留在工程里，后续可以继续修，但当前 README 不把它们包装成“已经可用”。

## 硬件基础

当前工程对应的硬件栈：

| 模块 | 说明 |
| --- | --- |
| MCU | `STM32G431RBTx` |
| Gate Driver | `DRV8323` |
| PWM | `TIM1` |
| 电流采样 | 双分流 |
| 编码器 | SPI 绝对值编码器 |
| 调试串口 | `USART2 @ 115200` |
| CAN | `FDCAN1` |

代码中可确认的关键参数：

| 参数 | 数值 |
| --- | --- |
| PWM 频率 | `25kHz` |
| 控制周期 | `40us` |
| 分流电阻 | `5mΩ` |
| 电流采样增益 | `20` |

## 快速开始

### 1. 打开工程

Keil 工程：

```text
MDK-ARM/dgm_v1.2.uvprojx
```

CubeMX 工程：

```text
dgm_v1.2.ioc
```

### 2. 烧录并连接硬件

至少连接：

- 功率板
- 电机
- 编码器
- 串口
- CAN 总线

### 3. 连接串口

默认串口参数：

```text
115200 / 8N1
```

### 4. 执行标定

串口输入：

```text
c
```

标定完成后输入：

```text
u
```

将参数保存到 Flash。

### 5. 进入驱动状态

串口可输入：

```text
m
```

但当前更推荐的主控制入口是：

```text
CAN + MIT 协议
```

## 串口命令

菜单模式下支持：

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

## MIT 协议

### 1. 电机使能 / 失能

向 `can_id` 发送 8 字节命令：

使能：

```text
FF FF FF FF FF FF FF FC
```

失能：

```text
FF FF FF FF FF FF FF FD
```

### 2. 控制帧

控制帧 ID：

```text
0x70 | can_id
```

8 字节数据按 MIT 风格打包，包含：

- `pos`
- `vel`
- `kp`
- `kd`
- `t_ff`

这些量的映射范围由以下参数决定：

- `pos_max`
- `vel_max`
- `iq_max`

### 3. 反馈帧

反馈帧 ID：

```text
0x10 | can_id
```

反馈内容包括：

- 电机 ID
- 错误标志
- 位置
- 速度
- q 轴电流反馈

## 参数配置

驱动器参数保存在 MCU 片内 Flash 中，支持：

- 上电自动加载
- CRC 校验
- 串口在线修改

参数设置模式下输入格式：

```text
key = value
```

例如：

```text
current_limit = 10
can_id = 3
pos_max = 3.14
vel_max = 30
iq_max = 8
```

输入完成后执行：

```text
u
```

保存到 Flash。

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

| 文件 | 作用 |
| --- | --- |
| `foc_handle.c` | FOC 电流环与调制 |
| `foc_encoder.c` | 编码器读取、角度与速度计算 |
| `controller.c` | 控制输入整形 |
| `can.c` | MIT 协议收发 |
| `fsm.c` | 串口菜单与状态机 |
| `calibration.c` | 标定流程 |
| `usr_config.c` | 参数管理与 Flash 保存 |
| `drv8323.c` | DRV8323 寄存器配置 |

## 标定内容

当前标定流程会覆盖：

- 相电阻测量
- 相电感测量
- 电流环 PI 参数计算
- 极对数识别
- 编码器方向识别
- 编码器零点偏移计算
- 编码器 LUT 补偿表生成

## 开发路线

这个仓库当前更像是一个正在持续完善的驱动器项目，而不是已经高度抽象的平台型框架。

后续值得优先推进的方向：

- 完整修复速度环 / 位置环
- 补充 MIT 主控侧示例代码
- 补充原理图、接线图、测试电机信息
- 强化保护策略和异常恢复
- 增加正式 `LICENSE`

## 免责声明

本项目会直接驱动电机功率级。调试不当可能导致：

- 电机突然转动
- 大电流
- 发热
- 驱动器损坏

请务必在限流、电源保护和机械安全条件下进行调试。
