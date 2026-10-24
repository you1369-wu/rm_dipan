# rm_dipan 麦克纳姆底盘控制工程

基于 RoboMaster C 板、STM32F407 和 FreeRTOS 的四轮麦克纳姆底盘固件，实现遥控全向移动、小陀螺、独立轮速 PID、模型功率限制，以及定时前进的自动任务。

本文按 `feature/wuyou-power_prediction` 分支当前代码整理，核对日期：2026-10-24。项目说明统一维护在本文件；修改控制参数时请同步更新。工程和固件目标沿用历史名称 `rm_imu`，实际业务为底盘控制。

> 安全提示：首次测试须可靠支撑底盘、轮子离地并准备硬件急停。右下发送的是零力矩，不保证机械上立即停止；自动任务结束则是主动零速闭环，两者不同。

## 目录

- [已实现功能](#已实现功能)
- [项目架构](#项目架构)
- [硬件与接口](#硬件与接口)
- [遥控操作](#遥控操作)
- [自动任务](#自动任务)
- [控制算法与参数](#控制算法与参数)
- [功率限制](#功率限制)
- [串口绘图与调试](#串口绘图与调试)
- [FreeRTOS 任务](#freertos-任务)
- [开发与验证](#开发与验证)
- [故障排查与边界](#故障排查与边界)
- [版本记录](#版本记录)

## 已实现功能

- DBUS 遥控器接收，左摇杆控制前后/横移，拨轮控制定速小陀螺。
- 麦轮逆运动学，车体速度转换为四轮目标轮轴角速度。
- 手动平移目标加速度限制、摇杆/拨轮死区、四轮同比例轮速限幅。
- 四台 M3508 独立速度 PID，CAN1 集中接收反馈和发送同组控制帧。
- 根据转矩与轮速预测功率，超过 80 W 时同比例缩小四轮转矩。
- 接收 CAN 功率计反馈，串口绘制功率上限与实测功率。
- 左下选择自动：轮轴 12 rad/s 纯前进 4 秒，随后保持零速度；手动流程仍保留。
- 遥控失联、反馈失效、非法输入和调度异常保护，自动故障锁存。
- BMI088/Mahony 基础姿态采样、LED、蜂鸣器和主机行为测试。

自动任务目前是本地定时速度控制，不是位置闭环导航。不包含巡线、障碍物识别、隧道通过检测、结束区识别或与 `rm_yuntai` 的任务同步。

## 项目架构

### 目录与职责

```text
applications/
  control_task.cpp/.hpp       控制入口、手动映射、麦轮解算、PID、功率限制和 CAN 收发
  can_task.cpp/.h             CAN1 和四轮电机对象定义；保留兼容任务入口
  remote_task.cpp/.h          DBUS 接收与遥控快照接口
  mission/
    mission.hpp              自动状态、输入、输出和只读状态接口
    mission.cpp              非阻塞自动任务状态机
    mission_config.hpp       自动轮速、持续时间、调度保护参数
  referee.cpp/.hpp            PM02 对象和接收初始化
  usart_callbacks.cpp         DBUS/PM02 串口接收及错误回调
  usart_task.cpp/.h           串口辅助任务入口
  imu_task.cpp/.h             BMI088 采样和 Mahony 姿态解算
  tools/plot_task.cpp         功率数据绘图
  pid_train.cpp/.hpp          PID 调试辅助
  led_task.cpp                LED
  buzzer_task.cpp             蜂鸣器
  power_on_melody.cpp/.hpp    开机旋律
Core/                        CubeMX 外设初始化、IRQ 和 FreeRTOS 任务创建
Drivers/                     STM32 HAL 与 CMSIS
Middlewares/                 FreeRTOS 等第三方中间件
sp_middleware/               DBUS、CAN、电机、PID、麦轮、Plotter 等 Git 子模块
tests/                       自动任务、真实控制路径与硬件桩测试
cmake/                       ARM 工具链与 CubeMX 构建配置
rm_imu.ioc                   CubeMX 工程
CMakeLists.txt               固件源文件清单
CMakePresets.json            Debug / Release 预设
openocd.cfg / .vscode/        烧录与调试配置
```

### 控制与数据流

```text
USART3 / DBUS -> 遥控快照 -> 手动 / 自动选择
                                  |
                 手动速度映射 / mission 速度请求
                                  |
                          麦轮解算 + 轮速限幅
                                  |
CAN1 四轮反馈 ----------------> 四轮速度 PID
                                  |
                         预测功率 + 转矩缩放
                                  |
                     发送前复查遥控器与反馈
                                  |
                         统一发送 CAN1 0x200
```

`control_task` 拥有电机控制和发送权限；CAN 接收回调按 ID 更新反馈。`mission` 只生成速度请求，不新建 FreeRTOS 任务、不运行另一套 PID，也不单独发 CAN。模式变化和自动行驶转停车时清除旧目标与 PID 历史。

## 硬件与接口

目标为 STM32F407 C 板；CubeMX 芯片族标识 `STM32F407I(E-G)Hx`、封装 `UFBGA176`。实物接线以 `Core/Inc/main.h`、外设初始化和 `rm_imu.ioc` 为准。

| 接口 | 用途 | 当前配置 / MCU 引脚 |
| --- | --- | --- |
| CAN1 | 四台电机与功率计 | 1 Mbit/s；PD1 TX、PD0 RX，经 CAN 收发器连接 |
| USART1 | Plotter 输出 | 115200、8N1、无流控；PA9 TX、PB7 RX |
| USART3 | DBUS 遥控器 | 100000、8 数据位 + 偶校验 + 1 停止位；PC11 RX；HAL 为 9B + EVEN |
| USART6 | PM02 / 裁判协议接收 | 115200、8N1；PG14 TX、PG9 RX |
| SPI1 | BMI088 | PB3 SCK、PB4 MISO、PA7 MOSI；片选 PA4 / PB0，低有效 |
| GPIO | RGB LED | PH12 红、PH11 绿、PH10 蓝 |

USB-TTL 适配器应使用匹配的 3.3 V TTL 电平、共地，RX 接板端 PA9 TX。板载 USB/探针虚拟串口不一定直连 PA9。BMI088 使用 SPI，不占 USART1；当前底盘运动控制没有使用 IMU 做航向闭环。

### 四轮 ID 与方向

四个电机对象均为 `M3508` 类型，配置减速比 `13.7`。角度、速度和力矩已折算到减速输出端，应用层不要重复换算；实际减速机构必须与配置一致。

| 轮位 | 对象 | 实物 ID | 反馈帧 | 控制帧槽位 |
| --- | --- | --- | --- | --- |
| 左前 LF | `motor_lf` | 4 | `0x204` | `0x200` 第 4 槽 |
| 左后 LR | `motor_lr` | 1 | `0x201` | `0x200` 第 1 槽 |
| 右前 RF | `motor_rf` | 3 | `0x203` | `0x200` 第 3 槽 |
| 右后 RR | `motor_rr` | 2 | `0x202` | `0x200` 第 2 槽 |

`wheels[]` 的逻辑顺序是 LF、LR、RF、RR，不是 ID1、2、3、4。右侧安装方向在麦轮解算中取反；纯前进对应目标轮速 `+12/+12/-12/-12 rad/s`，并非让左右轮驱动车辆反向运动。

四台电机共用 `0x200` 帧。若更改电调 ID，须同时核对帧分组和发送逻辑；不能只改 ID 后仍假定一帧覆盖全部电机。

## 遥控操作

| 右拨杆 | 左拨杆 | 行为 |
| --- | --- | --- |
| 下 | 任意 | 四轮零力矩，清除自动计时、确认自动故障并准备启动 |
| 中 | 中或上 | 手动控制 |
| 上 | 中或上 | 手动来源失能，双上也不会启用底盘手动运动 |
| 中或上 | 下 | 自动前进任务 |

手动使能时：

| 输入 | 映射 |
| --- | --- |
| 左摇杆上下 `ch_lv` | `vx`，前后平移，正值为向前 |
| 左摇杆左右 `ch_lh` | `vy`，横移，按当前代码取反，车体正 y 为向左 |
| 拨轮 `ch_lu` | 死区外固定 `wz=±1 rad/s`，正值逆时针、负值顺时针，回中为 0 |
| 右摇杆 / 键盘 | 当前未用于运动控制 |

小陀螺 `1 rad/s` 是车体旋转角速度，不是电机轮轴速度。摇杆和拨轮方向应先在轮子离地时核对实物；安装变化后不能仅凭变量名推断行驶方向。

建议右下上电，等待遥控器与四轮反馈正常，再进入手动或自动。自动首次启动必须先经过一次遥控在线的右下确认；直接上电保持左下、右中/上会锁存 `NotArmed`。

## 自动任务

`applications/mission/` 的状态为：

```text
Disabled -> WaitReady -> Running（4 秒）-> Finished（持续零速闭环）
                            异常 -> Fault（锁存失能）
```

| 配置项 | 当前值 | 含义 |
| --- | --- | --- |
| `kForwardWheelSpeedRadS` | 12.0 rad/s | 纯前进时轮轴角速度幅值 |
| `kDriveDurationMs` | 4000 ms | 前进指令持续时间 |
| `kMaxControlGapMs` | 5 ms | 自动调度间隔保护 |

- 左下、右中或右上选择自动，等待四轮反馈就绪，实际进入 Running 才开始计时。
- 自动以 `vx = 12 * 0.077 = 0.924 m/s`、`vy=wz=0` 请求前进，不使用手动平移加速度斜坡；轮速限幅、PID、功率限制和发送前检查仍生效。
- 运行满 4 秒进入 Finished，清除旧积分，以四轮零速目标主动闭环停车，不是仅撤销力矩滑行。
- 保持左下不会周期性重复前进。正常退出左下清除计时，重新进入可启动新一轮；右下优先失能并清除计时。
- 自动运行故障后，恢复反馈或切手动不能绕过锁存，需右下确认。
- 理想无滑、忽略加减速时距离约 `0.924*4=3.696 m`。这是速度指令积分，不是位置目标；实际停止时间和距离必须实测。

云台的自动阶段目前为 15 秒，底盘前进为 4 秒，两者没有同步消息或共同开始时间，不能认为同时拨左下就能证明安全过隧道。

## 控制算法与参数

控制周期目标为 1 ms，`control_task` 使用 `vTaskDelayUntil`；当前要求 RTOS tick 为 1000 Hz。车体坐标为 x 向前、y 向左，旋转角速度单位 rad/s。

主要参数位于 `applications/control_task.cpp`：

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `kWheelRadiusM` | 0.077 m | 轮轴速度与线速度换算 |
| `Mecanum` 几何参数 | 0.165 / 0.185 m | 前后 / 左右半间距，须与实物一致 |
| `kMaxVx / kMaxVy` | 4.0 / 4.0 m/s | 手动映射的车体目标上限，不是可保证的实测速度 |
| `kStickDeadband` | 0.05 | 平移摇杆死区，死区外重新缩放 |
| `kDialDeadband` | 0.10 | 小陀螺拨轮死区 |
| `kSpinWz` | 1.0 rad/s | 车体小陀螺角速度幅值 |
| `kAcceleration` | 0.5 m/s² | 手动平移目标变化率 |
| `kMaxWheelSpeedRadS` | 24.0 rad/s | 任一轮目标超限时四轮同比例缩小 |
| 四轮速度 PID | kp=0.6、ki=0.3、kd=0 | 四个独立 PID，dt=0.001 s |
| PID 输出 / 积分限幅 | 3.0 / 0.5 N·m | 功率缩放前的输出端转矩限幅 |

纯平移在 24 rad/s 轮速上限下，理想轮缘速度为 `24*0.077=1.848 m/s`；平移与旋转叠加时还会同比例限幅，实际速度也受功率、负载、打滑影响。不能把 `kMaxVx=4` 理解为当前可达到 4 m/s。

修改顺序建议：先确认轮位、ID、轮向、减速比和轮径，再做低速轮速闭环测试，最后调四轮 PID、加速度和功率模型。不能仅提高车体目标而忽略轮速及功率上限。

## 功率限制

当前启用了基于模型的转矩缩放，功率上限在 `run_wheel_speed_control()` 中固定为 `80.0 W`。PM02 接收已初始化，但从 `pm02.robot_status.chassis_power_limit` 动态读取上限的表达式目前被注释，并未参与限功率。

预测使用 PID 原始转矩与实际轮轴转速：

```text
P = k3 + sum(torque[i]*omega[i] + k1*torque[i]^2 + k2*omega[i]^2)
k1=2.6，k2=0.006，k3=4.0
```

预测超限时求四轮统一缩放系数 K，使模型中的缩放后功率接近上限；K 约束为 0～1，再将各轮转矩乘 K 后发送。无可用模型解时保守使用零缩放；预测值非有限时返回失能路径。该机制不改变目标轮速，只减小本周期驱动力矩。

| 变量 | 含义 |
| --- | --- |
| `chassis_predicted_power_w` | 缩放前预测功率 |
| `chassis_real_power_w` | 最近接收的功率计数值 |
| `chassis_power_limit_w` | 当前运行上限；失能时置 0 |
| `chassis_power_scale` | 转矩缩放 K，正常不超限为 1 |

实测功率来自 CAN1 标准数据帧 `0x301`：8 字节载荷中 `data[4:5]` 为大端整数，单位 0.01 W。当前仅显示最近接收值，没有用它闭环修正模型或判断功率计超时；读数为 0 不一定表示当前真实功率为 0。

模型系数是待实测标定的经验值。限制预测功率不保证实际输入功率严格不超过 80 W，需用匹配的功率计、正确转矩/轮速单位和实际负载验证。

## 串口绘图与调试

`applications/tools/plot_task.cpp` 使用 USART1，`osDelay(10)`，当前仅输出 2 通道，预测功率通道未启用：

| 通道 | 内容 | 单位 |
| --- | --- | --- |
| 1 | `chassis_power_limit_w` | W；运动时当前为 80，失能为 0 |
| 2 | `chassis_real_power_w` | W；最近功率计读数 |

串口配置为 115200、8N1、无流控；二进制格式 `AA BB 08` 加 2 个小端 float32，总长 11 字节，不是文本 CSV。PC 上位机需要打开正确 COM 口并匹配帧头、长度和数据格式，不要与其他 USART1 文本输出混用。

常用观察项：

- `control_source`：MANUAL / AUTOMATIC；`mission_status.state/fault/armed/elapsed_ms`：自动阶段、故障、启动许可和计时。
- `chassis_vx_set/vy_set/wz_set`：车体目标，自动 Running 时 vx 为 0.924 m/s。
- `wheels[i].data.target_speed_set/given_torque`：LF、LR、RF、RR 的轮速目标和最终转矩。
- `motor_lf/lr/rf/rr.speed`：对应输出轴实际角速度。
- `wheel_pid_p/i/d[]`：四轮 PID 分量。
- `chassis_remote_online/motors_online/right_switch_mid`：控制检查状态；部分变量在失能路径会清零，不能单独作为物理在线诊断。

## FreeRTOS 任务

`Core/Src/freertos.c` 当前创建下列任务；底盘 `mission` 无独立任务，`can_task` 和 `usart_task` 入口也未单独创建为工作任务。

| 入口 | 优先级 | 栈 / word | 作用 |
| --- | --- | --- | --- |
| `control_task` | Realtime | 512 | 1 ms 控制与 CAN |
| `imu_task` | High | 512 | 基础姿态采样 |
| `remote_task` | Normal | 512 | 遥控接收 |
| `StartDefaultTask` | Normal | 128 | 默认任务 |
| `plot_task` | Low | 512 | 功率绘图 |
| `led_task` | Low | 512 | LED |
| `buzzer_task` | Idle | 512 | 蜂鸣器 |

本端口每 word 为 4 字节；tick 1000 Hz，动态堆 15360 字节。底盘控制任务依赖 `INCLUDE_vTaskDelayUntil=1`，重新用 CubeMX 生成后必须检查保留。其它任务优先级、栈大小和 DMA/IRQ 配置也应核对，新增任务先测栈高水位。

## 开发与验证

### 环境与构建

需要 Git、CMake 3.22+、Ninja 和 GNU Arm 工具链，固件使用 C11 / C++17。STM32CubeMX 用于修改 `rm_imu.ioc`；烧录使用 OpenOCD 或 STM32CubeProgrammer，VS Code 可配合 CMake Tools 与 Cortex-Debug。

在本仓库根目录执行：

```powershell
git submodule update --init --recursive
arm-none-eabi-gcc --version
cmake --preset Debug
cmake --build --preset Debug
```

Release 使用 `cmake --preset Release`、`cmake --build --preset Release`。产物为 `build/Debug/rm_imu.elf` 或 `build/Release/rm_imu.elf`，链接 map 位于同一构建目录。不要提交本地 `build/`。

编译器找不到时检查 PATH；工具链报找不到 `cc1.exe/cc1plus.exe` 时检查安装完整性；中间件源码找不到时初始化子模块。

### 烧录与调试

当前 `openocd.cfg` 使用 CMSIS-DAP / STM32F4，在根目录执行：

```powershell
openocd -f openocd.cfg -c "program build/Debug/rm_imu.elf verify reset exit"
```

使用 ST-Link 时先更换探针配置。VS Code 提供 `OpenOCD: flash` 构建任务与 `OpenOCD: Debug` 启动配置。烧录前保持右下、支撑底盘；verify/reset 成功只表示程序下载与复位成功，不代表运动参数安全。

### 主机测试

使用桌面 GCC 和 Ninja，与 ARM 构建分开：

```powershell
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_CXX_COMPILER=g++
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

也可在已加载 MSVC 的 Developer PowerShell 中，将编译器改为 `cl`，并使用不同构建目录。当前主机测试设置为 C++14，固件仍为 C++17。

已注册 `mission_behavior` 和 `chassis_control_behavior`，覆盖自动边界、计时回绕、单次运行、零速停车、手动回归、拨杆优先级、异常保护，以及真实麦轮/PID/功率路径。测试用桩模拟遥控、传感器、时钟和 CAN，不包含实际动力学、摩擦、负载或功率计误差，不能代替整车验证。

### 维护约定

新增源码同步修改根 `CMakeLists.txt`；重新生成 CubeMX 时保留 USER CODE、任务外部入口和 RTOS 延时配置。应用控制与驱动分层，避免为改变业务参数修改 `Drivers` 或 `sp_middleware`。调参时只修改明确的参数位置，并同步本 README。

## 故障排查与边界

| 现象 | 排查重点 |
| --- | --- |
| 车不动 | 手动是否右中、遥控输入有效性、四轮反馈、`mission_status.fault`、发送与电机供电 |
| 左下自动不启动 | 先右下确认；检查 `NotArmed`、WaitReady、四轮 CAN ID 和在线状态 |
| 满 4 秒后仍保持使能 | Finished 是主动零速闭环；需要零力矩则切右下 |
| 自动结束后不再前进 | 设计为单次运行；退出自动再进入才重新计时 |
| 小陀螺速度与预期不符 | `kSpinWz=1 rad/s` 为车体量；同时检查轮向、几何、24 rad/s 轮速限幅和功率缩放 |
| 目标设 4 m/s 却达不到 | 当前轮速限幅对应纯平移约 1.848 m/s，且可能继续受功率限制 |
| 功率曲线是 0 或不更新 | `0x301` 帧是否存在、字节位置/单位是否匹配；当前没有功率计超时诊断 |
| 收不到串口 / 无曲线 | PA9 接线、共地、COM 是否打开、115200、2 通道 float32 和二进制帧格式 |
| 切手动后仍失能 | 自动故障锁存未清，先排除原因再右下确认 |

自动故障枚举包含 `NotArmed`、`RemoteLost`、`InvalidInput`、`FeedbackLost`、`Timing`、`ControlFault`。恢复反馈并不自动清除锁存。手动异常主要走本周期失能检查，不应把自动故障状态当成所有手动故障的完整记录。

已知边界：手动/自动均依赖遥控在线和 CAN 正常；当前软件零力矩无法替代断电急停；尚无基于地图、位置或 IMU 航向的导航闭环；功率模型需要实测标定，功率计数据无超时保护；定时前进无法保证精确行程。两板若接入同一 CAN 总线，还必须核对 ID 冲突、接线与控制所有权，当前代码没有定义完整的整车协调协议。

## 版本记录

本节按当前分支可达的 Git 提交整理重要里程碑，最新在前，不是完整提交清单。当前未建立正式版本标签，因此使用“日期 + 变更说明 + 提交哈希”，不将每个日期视为一次正式发布。历史参数可能已被后续提交覆盖，当前行为以本文功能和参数章节为准。

### 未发布（2026-10-24）

- 重写中文 README，统一说明底盘架构、硬件接口、手动/自动操作、功率限制、绘图、构建测试和安全边界。
- 将旧 `docs/mission.md` 的有效内容合并到根 README，清理分散说明，并补充可追溯的历史记录。
- 本项为当前工作区文档变更，尚无对应提交哈希。

### 历史里程碑

| 日期 | 主要变更 | 提交 |
| --- | --- | --- |
| 2026-10-24 | 功率绘图改为“功率上限、实测功率”两通道，暂不输出预测功率。 | `47d4f24` |
| 2026-10-23 | 调整自动前进速度和轮速上限；当前自动任务为轮轴 12 rad/s 前进 4 秒，全局轮速上限 24 rad/s。 | `623c90b`、`0d61913` |
| 2026-10-23 | 加入底盘自动任务，保留手动入口和右下优先失能；自动完成后维持零速闭环。 | `2b95bd2` |
| 2026-10-23 | 调整小陀螺速度及控制参数；当前车体小陀螺角速度幅值为 1 rad/s。 | `9abaab9`、`1025116` |
| 2026-10-22 | 增加独立的小陀螺角速度参数 `kSpinWz`，调整前后/横移等运动参数。 | `95ab822`、`4d081a5`、`6a06272` |
| 2026-10-16 | 调整限功率和轮速 PID 参数，增加功率预测模型绘图测试。 | `78188e5`、`8d70d9b`、`8210673` |
| 2026-10-15 | 接入功率计实测功率反馈，支持与模型预测结果对比。 | `f340767` |
| 2026-10-15 | 加入底盘功率预测与限制，在速度控制输出之后缩放电机转矩。 | `6475f16` |
| 2026-10-15 | 调整四轮电机 ID 配置，扩展 USART1 的 PID 调试绘图。 | `d716f25`、`1ae9b5a`、`ad1cf3c` |
| 2026-10-14 | 建立四台 M3508 的底盘控制及 PID 绘图，并整理控制任务结构。 | `1988aa8`、`9506de5` |
| 2026-10-05 | 共用工程初期集中电机控制任务，加入失能、PID 训练/调试绘图，以及 LED、蜂鸣器等基础功能。 | `b325edd`、`d20e85e`、`654d729`、`e44af4e`、`c9c4fa3`、`e0020b0` |
| 2026-10-02～04 | 建立 CubeMX/FreeRTOS 基础工程，接入中间件、BMI088、DT7 遥控器、早期 GM6020 CAN 通信与串口输出；尚非当前四轮底盘业务。 | `cca6ece`、`9eb82b6`、`b8028be`、`e4155d4`、`d72f434`、`0a4d436` |

完整历史用 `git log --oneline` 查看，单条变更用 `git show <提交哈希>` 查看。后续有重要功能、参数或架构变更时在本节顶部追加；正式打版本标签后，再记录真实标签，不凭空补造发布版本号。

## 依赖与许可

`sp_middleware` 为 [TongjiSuperPower/sp_middleware](https://github.com/TongjiSuperPower/sp_middleware) 子模块。STM32 HAL/CMSIS、FreeRTOS 及其它第三方代码遵循各自许可，应保留版权声明和许可文件。本项目根目录目前没有单独的项目许可证，不能据此假定全部代码可任意再分发。
