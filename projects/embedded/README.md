# 嵌入式设备控制系统

项目包含可编译的C应用核心、FreeRTOS任务工程、通信与存储测试，以及运行相同C核心的网页调试界面。

## 模块

| 模块 | 实现 |
| --- | --- |
| 采集 | 16点DMA双缓冲流程，完成块经8项有界队列交给消费任务 |
| 控制 | 一阶执行器对象、PI输出限幅、条件积分抗饱和 |
| 通信 | 64字节环形接收缓冲、分包重组、长度检查、CRC16/MODBUS、应答帧 |
| 保护 | 过流、母线欠压、编码器失联、控制任务喂狗超时，故障锁存和显式复位 |
| 参数 | 显式小端存储格式、CRC32、掉电恢复、损坏记录回退默认值 |
| 更新 | 非活动分区写入、镜像CRC、版本检查、试启动确认及回退 |
| 任务 | 官方FreeRTOS V11.2.0，消息队列、互斥锁及四任务工程 |

## 执行方式

`firmware/device.c`同时编译为本机程序和`device.wasm`。网页调用WebAssembly导出的C接口，协议与保护逻辑不是JavaScript重写版本。模块不依赖外部WebAssembly运行库。

浏览器提供逻辑ADC输入与一阶执行器对象，按逻辑毫秒驱动采集、控制、通信和健康任务；任务视图显示C核心的实际调用记录。该调度视图没有运行FreeRTOS内核。

`firmware/freertos_app.c`另外使用真实FreeRTOS内核和官方Windows主机端口运行：采集任务通过消息队列发送40个样本，控制任务核对顺序并消费数据，通信任务执行命令，互斥锁保护共享核心，健康任务检查运行结果。主机端口不提供MCU实时性测量。

Flash和DMA为软件模型，不访问真实板级寄存器。示例镜像是8KB校验测试载荷，不是可烧录的MCU程序。接入真实MCU需另外实现ADC、DMA、通信和Flash驱动，并复核数据类型、字节宽度、时钟与中断配置。

## 通信格式

`AA 55 | SEQ | CMD | LEN | PAYLOAD | CRC_LO CRC_HI`

CRC16/MODBUS初值0xFFFF，多项式0xA001，覆盖帧头至载荷。多字节载荷采用小端序。

| 命令 | 载荷 | 含义 |
| --- | --- | --- |
| 0x01 | 1字节，0或1 | 停止/启动 |
| 0x02 | 2字节，0–2400 | 目标速度 |
| 0x03 | 2字节，1–20 | 采样周期/ms |
| 0x10 | 空 | 保存参数 |
| 0x11 | 空 | 故障确认复位 |

应答命令号为原命令号或0x80，载荷为状态码：0成功、1CRC错误、2参数越界、3设备忙、4命令/长度不支持、5条件不安全。故障未移除时禁止复位；异常移除不会自动恢复驱动。

## 复现

需要Python 3和GCC，C核心不需要FreeRTOS依赖：

```text
python build.py --gcc path/to/gcc
```

如需重建WebAssembly，使用支持wasm32的Clang：

```text
python build.py --gcc path/to/gcc --clang path/to/clang
node tests.cjs
```

FreeRTOS任务工程采用固定版本V11.2.0和Windows/MinGW端口：

```text
git clone --branch V11.2.0 --depth 1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git
python build.py --gcc path/to/gcc --freertos path/to/FreeRTOS-Kernel
```

`firmware/host_compat/timeapi.h`仅为旧MinGW提供WinMM声明兼容，不修改内核源码。FreeRTOS内核采用MIT许可证，从[官方仓库](https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V11.2.0)获取。

网页自检在独立WebAssembly实例中执行，包含CRC标准向量、GCC/WASM快照一致性、故障锁存、参数恢复及镜像验证。`validation.json`保存本次主机执行及WebAssembly测试记录；逻辑计时不作为板端性能指标。

## 文件

- `firmware/device.c`：采集、通信、控制、存储及升级核心。
- `firmware/device.h`：接口、状态和事件定义。
- `firmware/freertos_app.c`：FreeRTOS主机任务工程。
- `firmware/FreeRTOSConfig.h`：内核配置。
- `firmware/tests.c`：C核心测试。
- `tests.cjs`：WebAssembly与本机结果一致性检查。
- `build.py`：可复现编译入口。
- `device.wasm`：由C核心编译的浏览器模块。
- `firmware-v101.bin`：镜像校验测试载荷。

Bug反馈联系邮箱：18662846539@163.com。
