# PMSM驱动控制与软件验证

入口：`index.html`。页面运行由C源码编译的WebAssembly；控制波形、PWM占空比、采样值、状态与CAN应答来自C执行结果。

## 工程内容

| 子系统 | 实现与验证 |
| --- | --- |
| 电机控制 | 两相电流采样、Clarke/Park、速度PI、dq电流PI、解耦前馈、逆Park、SVPWM |
| 工业伺服 | 位置环、五次多项式轨迹、速度/加速度前馈；48V表贴式PMSM定位工况 |
| 电流分配 | 内嵌式PMSM的MTPA、母线电压约束弱磁、电流圆限制 |
| 执行链 | 20kHz采样/电流控制，2kHz速度环，1kHz位置与管理；12bit ADC量化与17bit编码器 |
| 通信 | 自定义CAN应用报文、8字节载荷、滚动计数器、CRC8-SAE J1850、实际C编码/解析 |
| 标定 | 电机物性、惯量、环路带宽与限流；范围、ADC量程及L/R积分约束检查；JSON导入导出 |
| 诊断 | 过流、欠压、编码器失联、CAN指令超时、控制执行周期监测；锁存、有条件复位、显式重新使能 |
| 验证 | 主机GCC回归、同一C核心的WASM一致性、独立RK4、MATLAB ode45与Simulink对象模型 |

六种运行工况：速度/负载阶跃、伺服定位、基速以上升速、正反转、转矩模式、故障关断。修改参数会重新执行当前工况，保持图表与计算结果一致。三维轴系由计算角度驱动，可回放已计算的时间序列。停止驱动关闭PWM，转速仍由惯性、剩余电流与阻力决定；回放不改变控制核心。

## 软件结构

```
CAN / 标定参数 → 运行服务与状态管理 → 控制算法 → PWM占空比
                       ↑                  ↑           ↓
                故障与周期监测        电流/角度反馈 ← 平均逆变器 + PMSM对象
```

- `firmware/control.c`：位置、速度、电流控制与电流分配。
- `firmware/plant.c`：dq对象、MTPA与SVPWM。
- `firmware/runtime.c`：采样量化、编码器反馈、周期分频和保护。
- `firmware/protocol.c`：应用报文、序号和CRC。
- `firmware/servo.h`：参数、状态及模块接口。应用核心无动态内存分配。
- `firmware/tests.c`：主机回归与独立MTPA数值搜索。
- `reference.js`：独立RK4电机方程参考；不是控制代码的JavaScript替代。
- `matlab/build_mil.m`、`matlab/PMSM_DQ_MIL.slx`：可见Simulink电机对象与独立MATLAB交叉复核。
- `native-validation.json`、`validation.json`、`matlab-validation.json`：可复现测试记录。

## 模型与控制

使用机械角速度ωm、电角速度ωe=pωm，电流与电压均在转子dq坐标系：

```
did/dt = (vd − Rs·id + ωe·Lq·iq) / Ld
diq/dt = (vq − Rs·iq − ωe·(Ld·id + ψf)) / Lq
Te = 1.5·p·[ψf + (Ld−Lq)·id]·iq
dωm/dt = [Te − Tload·ωm/sqrt(ωm²+0.04) − B·ωm] / J
dθm/dt = ωm
```

Tload为阻力矩，使用平滑符号避免未上电的轴被该阻力模型主动反驱。C对象用12.5μs中点积分，控制周期50μs；参考用RK4和ode45。PWM以三相占空比还原平均相电压后驱动对象，未建立开关损耗、死区或电流重构窗口模型。

电控示例参数：Vdc=300V，Rs=0.12Ω，Ld=0.4mH，Lq=0.8mH，ψf=0.085Wb，极对数4，J=0.006kg·m²，限流80A。伺服示例：Vdc=48V，Rs=0.55Ω，Ld=Lq=1.2mH，ψf=0.025Wb，J=0.00012kg·m²，限流8A。参数定义为分析工况，不对应某家产品的实测规格。

MTPA在给定转矩下最小化id²+iq²。令δ=Lq−Ld，有

```
id = [ψf − sqrt(ψf²+4δ²iq²)] / (2δ)
iq = T* / [1.5p(ψf−δid)]
```

通过迭代求解，并与独立数值搜索比较；Ld=Lq时退化为id=0。图表比较同一转矩下电流幅值，不将电流幅值差直接称为整机效率提升。弱磁依据电压利用率增加负id，并受电流圆和负d轴限幅约束。dq电压矢量限值为0.94Vdc/√3，PI采用反计算抗积分饱和。

## 复现

需要GCC、Node.js；重建WASM时增加支持wasm32的Clang。浏览器只导入标准sin/cos/sqrt数学函数，无额外WASM运行库。

```
python build.py --gcc path/to/gcc --clang path/to/clang
node tests.cjs
python -m http.server 8000
```

打开`http://localhost:8000/`。本次执行环境为MinGW GCC 8.1、Clang 16.0.0和MATLAB R2024a，编译启用`-Wall -Wextra -Werror`。

MATLAB复核：在`matlab`目录运行`build_mil`。脚本搭建可打开的dq对象模型，运行Simulink、独立ode45并比较C对象记录，写出`matlab-validation.json`。该Simulink文件是对象模型；闭环控制器与运行服务在C工程中，不把对象模型称为完整自动代码生成工程。

## CAN应用报文

命令ID0x201，反馈ID0x181，DLC=8。CRC8采用SAE J1850多项式0x1D、初值0xFF、输出异或0xFF，覆盖前7个载荷字节；标准测试串`123456789`为0x4B。此为自定义应用协议，独立于CAN链路层CRC。

| 字节 | 命令 |
| --- | --- |
| 0 | 8bit滚动计数器；首次帧接受任意初值，后续须加1并自然回绕 |
| 1 | 1使能、2停止、3给定、4模式、5复位、6标定参数、8心跳 |
| 2 | 0速度、1转矩、2位置；给定命令须与当前模式一致 |
| 3 | 标定参数索引，普通命令置零 |
| 4–5 | 小端有符号16bit；速度1rpm、转矩0.01Nm、位置0.1°每LSB |
| 6 | 保留，置零 |
| 7 | 应用层CRC8 |

反馈：计数器、使能、故障位、实际转速int16、实际转矩int16、CRC8。拒绝帧不会修改目标；CRC错误不消耗计数器。正确载荷但被状态机拒绝的命令仍消耗计数器，防止重复执行。CAN物理收发器与总线调度不在网页中执行。

## 验证范围与板级适配

测试包含波形响应、约束、状态迁移、保护响应、报文错误路径及主机/浏览器一致性。网页验证是主机软件执行，非HIL或板端PIL；不以浏览器计算耗时声称MCU实时性能。执行周期异常工况跳过控制调用，由独立管理周期按最后控制时间触发诊断。

接入C2000/TI或其他MCU时，将采样对象替换为ADC/编码器接口，将占空比出口接入PWM影子寄存器与保护关断；另行处理时钟、中断、Flash运行和实时性。当前类型与序列化在8bit字节主机上验证，C28x的16bit char需适配协议逻辑字节与驱动类型。手写应用层模块不等同完整AUTOSAR RTE工程；本项目不将回归通过作为MISRA-C或ISO 26262合规证明。

## 官方技术资料

- [TI C2000 MotorControl SDK](https://www.ti.com/tool/C2000WARE-MOTORCONTROL-SDK)
- [TI C2000 CAN软件说明](https://software-dl.ti.com/C2000/docs/software_guide/c2000ware/can.html)
- [AUTOSAR Classic Platform](https://www.autosar.org/standards/classic-platform/)

Bug反馈：18662846539@163.com。
