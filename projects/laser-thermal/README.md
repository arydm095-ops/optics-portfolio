# 激光传输与热透镜分析

计算流程：有限体积导热 → 三维温度场显示 → 热光程差与热透镜 → Gaussian 光束传播 → 补偿与验证。

## 使用与复现

入口 `index.html`。拖动 3D 图旋转，滚轮缩放；改变吸热功率、材料导热率、dn/dT、探测腰斑、M²、传播距离后自动更新计算，也可点击计算按钮。

“光学介质”提供熔石英（HPFS 7980）、N-BK7和氟化钙预置。切换介质会同时更新导热率、dn/dT、温度场、OPD和光束传播，保留当前吸热功率、尺寸与补偿镜。手动改导热率或dn/dT后转为“自定义介质”；选择自定义不会覆盖当前数值。导出数据与报告保留介质名称及参考来源。

| 介质 | k / W/(m·K) | dn/dT / 10⁻⁶/K | 参考条件 |
| --- | ---: | ---: | --- |
| Corning HPFS 7980熔石英 | 1.38 | 9.598 | k：25°C；dn/dT：1064nm、20–25°C，按厂家多项式计算并取三位小数 |
| SCHOTT N-BK7 | 1.114 | 1.1 | 绝对折射率温度系数，1060nm、20–40°C |
| Crystran CaF₂ | 9.71 | −10.6 | 供应商代表值，原表未注明dn/dT参考波长和温度 |

物性按参考常数使用，不随当前波长或温度自动换算，也不以材料名称推定吸收率。其他波长或高温工况可用“自定义介质”输入对应物性。N-BK7没有使用空气相对折射率的2.4×10⁻⁶/K。CaF₂的负dn/dT会产生负光焦度，这不等于忽略膨胀和应力之后的实际镜片总热透镜效应。

推荐先看默认工况，再将吸热功率置零对比，最后恢复参数并点击“抵消热透镜”。注意热透镜使光束先会聚后发散，不能根据某个近处光斑变小就判断远场发散改善。

```sh
# 仓库根目录
python -m http.server 8000
# 打开 http://localhost:8000/projects/laser-thermal/
node projects/laser-thermal/tests.cjs
node projects/laser-thermal/materials-tests.cjs
```

导出 JSON 包含工况、温度、OPD、传播曲线和验证数据。HTML 报告包含当前 3D 图与曲线，可打印为 PDF。3D 绘图使用本仓库的 Three.js，许可证见 `../../demos/vendor/LICENSE-three.txt`。

## 热模型

圆柱介质半径 R、长度 L，圆周恒温 T_b；热源轴向均匀、径向 Gaussian。吸收热功率 P 是总沉积热量，不等于未经吸收率换算的入射功率。吸热半径 w_p 是热源的 1/e² 半径，可与探测光束宽度不同。

```
C = 1 − exp(−2R²/w_p²)
Q'''(r) = 2P / (π w_p² L C) · exp(−2r²/w_p²)
(1/r) d/dr [r k dT/dr] + Q''' = 0
dT/dr |_(r=0) = 0,  T(R) = T_b
```

环形控制体吸收功率通过 Gaussian 的积分精确分配；相邻中心之间的导热系数为 `G=2πkL / ln(r_j/r_i)`。求解三对角温度方程，并计算外边界散热与吸收热的残差。

独立参考使用导热方程的积分解：

```
T(0)−T_b = P/(2πkLC) · ∫_0^R [1−exp(−2s²/w_p²)] / s ds
```

中心温度使用第一控制体温度加近轴抛物修正；三维温度图是径向解旋转形成的轴对称场，剖去四分之一显示内部。这里没有任意三个方向的三维导热求解，轴向温度采用均匀假设。

## 热光接口

`OPD(r) = L (dn/dT) [T(r)−T(0)]`。轴心近轴展开得到薄透镜光焦度：

```
D_T = 1/f_T = P (dn/dT) / (π k w_p² C)
OPD_paraxial(r) = −D_T r²/2
```

焦距符号随 dn/dT 改变。展示完整径向 OPD 与二次近轴项，并报告高阶残差：用探测光束在介质位置的 Gaussian 强度进行径向加权，去活塞后算 RMS，单位 λ。

高阶项没有进入完整波动传播，M² 保持输入值，不计算热致 Strehl。未建模热应力光学效应、端面膨胀、吸收率随温度变化及实际夹持条件。

## 光束传输

使用有效波长 `λ_eff=M²λ` 的 Gaussian 包络模型，初始腰斑半径 w0 在 z=0。冷态 Rayleigh 长度 `z_R=πw0²/λ_eff`。

```
q_in = z_material + i z_R
1/q_out = 1/q_in − D_T − D_comp
w²(z) = λ_eff / [−π Im(1/q(z))]
```

自由传播使 q 的实部增加传播距离。出射腰斑位置 `z_w=z_material−Re(q_out)`，腰斑半径 `sqrt(λ_eff Im(q_out)/π)`，远场发散半角 `λ_eff/(π w_waist)`。

补偿镜同面等效：`D_comp=1/f_comp`，f=0 表示无补偿。按钮设置 `D_comp=−D_T`，还原冷态传输；这是等效设计基准，不表示真实放置一片镜子必然消除所有像差。3D 光束采用纵向/径向不同显示比例，并只画 1/e² 包络，不将颜色当作实际激光可见颜色。

模型忽略介质内厚透镜传播、端面散热、瞬态、孔径裁切、散射、非近轴效应与损伤。M² 是输入光束参数，未进行热致模式变化求解。

## 默认工况与验证

默认HPFS 7980熔石英：k=1.38 W/(m·K)，dn/dT=9.598×10⁻⁶/K。P=1.5 W，w_p=1 mm，R=5 mm，L=5 mm，圆周 25°C；探测束 1064 nm、w0=0.35 mm、M²=1；介质位置 0.15 m，观察面 3 m。

- 120 控制体轴心温度约 102.705°C，独立积分参考约 102.661°C。
- 热透镜焦距约 0.30113 m。
- 冷态/热态远场发散半角约 0.96766 / 1.25966 mrad。
- 3 m 观察面光束半径约 2.92401 / 3.33392 mm。

15 项自动检查覆盖热功率分配、全局能量平衡、积分解析参考、零吸热、功率/导热率线性、Rayleigh 长度、自由 Gaussian 传播、发散角、同面补偿、q 变换可逆性、dn/dT 符号及网格收敛。完整结果在 `validation.json`。

介质检查另覆盖三种预置的解析温度、能量守恒、零吸热和补偿工况，以及CaF₂的负焦距、OPD符号、自定义参数保留。页面下方网格收敛表固定显示默认工况基准；切换介质不改写该验证记录。

## 资料

- [Corning HPFS 材料参数手册](https://www.corning.com/content/dam/corning/media/worldwide/csm/documents/HPFS_Product_Brochure_All_Grades_2022_6.pdf)
- [SCHOTT N-BK7 数据表](https://media.schott.com/api/public/content/41e799d0bf874807a0bb8e702fbb75b5?v=54856406)
- [Crystran CaF₂ 数据表](https://www.crystran.co.uk/optical-materials/calcium-fluoride-caf2/)
- [NBS：熔石英温度相关折射率研究](https://pmc.ncbi.nlm.nih.gov/articles/PMC6715984/)
- [Thorlabs：Gaussian 光束传播、腰斑和 Rayleigh 长度](https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=12211&tabname=Optical+Isolators)
- [Gaussian 热透镜研究](https://pmc.ncbi.nlm.nih.gov/articles/PMC8509296/)
