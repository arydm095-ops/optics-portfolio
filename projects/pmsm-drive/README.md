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

六种运行工况：速度/负载阶跃、伺服定位、基速以上升速、正反转、转矩模式、故障关断。修改参数会重新执行当前工况并更新曲线，三维播放停在起点。点击播放后按模型时间推进；暂停保留进度，再次点击继续；播放至末尾后点击可从头播放。进度条按采样时间定位，支持末段非均匀采样。停止驱动关闭PWM，转速仍由惯性、剩余电流与阻力决定；播放不改变控制核心。

“控制与采样”中的控制器参数可切换速度PI、d/q电流PI、位置P。速度环Kp的单位为Nm/(rad/s)，Ki为Nm/rad；电流环Kp为V/A，Ki为V/(A·s)；位置Kp为s⁻¹。这些参数通过`sv_set(20..26)`进入C控制算法，实际有效值可从`sv_get(60..66)`读回。位置控制使用P加轨迹速度/加速度前馈，未加入微分项。

“按带宽整定”由当前物性与带宽计算PI初值：速度Kp=2ζωbJ−B，Ki=Jωb²，ζ=0.9；电流Kpd=Ldωc、Kpq=Lqωc、Kid=Kiq=Rsωc。直接修改增益后切换为手动参数；`sv_set(27,0)`恢复自动模式。JSON标定和运行数据包含有效控制增益。回归检查逐项修改增益，比较C读回值和闭环执行波形。

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
- `playback.js`、`playback-tests.cjs`：播放时钟与姿态定位，验证暂停续播、速度切换、非均匀采样、拖动和重算复位。
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
node playback-tests.cjs
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


## 工作区、频域分析与机械传动

左侧按工况、电机与采样、机械传动、控制器、共振抑制和频域分析折叠。右侧每次显示选中的时域、频域、传动、通信或验证页面；曲线视图单独切换。电感界面单位为mH，传入C时转为H；速度限值界面为rpm，传入C时转为rad/s。电机预设与运行工况独立，可定义极对数、编码器计数、ADC量程、Rs/Ld/Lq/磁链、惯量、摩擦、母线和电流限值。完整参数及有效增益可用JSON保存和导入。

单惯量模型使用输入的电机惯量Jm；选双惯量模型后，加入折算到电机轴的负载惯量Jl、轴刚度K、轴间黏性阻尼C和负载摩擦Bl。轴扭矩Ts=K(θm−θl)+C(ωm−ωl)，分别进入电机侧和负载侧的运动方程。库仑摩擦采用Tc·ω/sqrt(ω²+0.04)。负载侧位置、速度、轴扭角和轴扭矩由C积分得到。固有频率fr=sqrt[K(1/Jm+1/Jl)]/(2π)，反共振频率fa=sqrt(K/Jl)/(2π)，为无阻尼参考；真实峰值受阻尼影响。采用固定12.5μs中点积分，并在界面限制ωr·dt≤0.08、阻尼/摩擦衰减率·dt≤0.1、电角频率·dt≤0.1、L/R≥25μs，拒绝超出当前积分精度的参数组合。

速度PI后串联数字陷波N(s)=(s²+2ζzωns+ωn²)/(s²+2ζpωns+ωn²)，在500μs速度周期用预畸变双线性系数执行。ζz≤ζp，中心增益ζz/ζp，DC与Nyquist增益为1。抗饱和回算比较限幅输出与滤波后的指令。陷波改变控制激励，不改变机械阻尼。开关对照使用独立C实例、相同机械参数与PI，并输出定位后的负载侧残余误差RMS；并不假定任何参数下开启陷波都能改善结果。

`analysis.js`计算离散PI、RL电流对象及双惯量速度对象的线性频响。PI使用旧积分状态输出，因此C(z)=Kp+Ki·T·z⁻¹/(1−z⁻¹)；电流RL对象采用精确零阶保持离散；速度对象采用连续双惯量传函，乘电流闭环、差分测速/低通和半周期保持延时的近似。库仑摩擦按工作转速的局部斜率线性化。分析假定dq解耦成立，不包含饱和、弱磁切换、ADC/编码器量化或大幅非线性轨迹。此为模型频域扫频，不是ADC注入得到的实机SFRA测量。

频率轴使用对数坐标。在共振和反共振附近自适应加密，并对全部0dB与奇数倍180°交叉点进行对数频率二分求根；汇总显示离0°/0dB最近的裕度，同时保留全部交叉记录。未在扫描窗口发现交叉时显示“范围内未检出”，不推断无限裕度。裕度不是闭环稳定性认证。推荐整定搜索有单一增益交叉、满足目标相位裕度且闭环峰值小于6dB的候选初值；应用后保留原方案对照，并用C时域运行检查实际响应。

运行`node analysis-tests.cjs`可复核：无阻尼双惯量解析状态、经典已知裕度、三个增益交叉点、预畸变数字系数、C陷波正弦响应，以及同参数的机械开关对照。`analysis-validation.json`为执行记录。MATLAB/Simulink原工程仍验证电机dq对象，双惯量与陷波在C/WASM及独立解析公式间另行校核。

参考：[MathWorks陷波离散化](https://www.mathworks.com/help/control/ug/discretizing-a-notch-filter.html)、[全部稳定裕度](https://www.mathworks.com/help/control/ref/dynamicsystem.allmargin.html)、[TI SFRA](https://www.ti.com/tool/SFRA)。
