# 简单三角隶属度模糊 PID

## 打开运行

把 `simple_fuzzy_pid.slx` 和 `triangle_pid.fis` 放在同一目录。双击 SLX，点击运行，打开 `Reference and output` 示波器。模型初始化时自动读取同目录 FIS，不需要先运行脚本。

依赖：MATLAB、Simulink、Fuzzy Logic Toolbox。构建与验证环境：R2025b。

在 MATLAB 中查看、调整三角形与规则：

```matlab
fuzzyLogicDesigner('triangle_pid.fis')
```

修改后从设计器导出覆盖同目录 `triangle_pid.fis`，重新仿真即可生效。需要重建全部交付物时，在本目录执行 `build_fuzzy_pid`；重建会恢复初始 FIS 配置。

## 模型结构

单位阶跃 → 误差 → 可变增益 PID → 二阶对象 → 负反馈。

对象 G(s)=1/(s+1)^2，阶跃发生在 1 s，仿真到 20 s，固定步长 0.01 s。它是便于学习的示例对象；接入自己的对象后应重新调整参数。

模糊调度器输入为 E=clip(3e,-3,3)、EC=clip(0.6 ec,-3,3)，ec 使用 20s/(s+20) 滤波微分。采用 Mamdani 推理，min AND/min 蕴含/max 聚合/重心解模糊。

输出为 dKp、dKi、dKd，增益计算是每个时刻的直接修正，不逐步累加：

- Kp=3+0.4*dKp
- Ki=1+0.12*dKi
- Kd=0.6+0.08*dKd

PID 明确分为 P、I、D 三条支路：u=Kp*e+Ki*integral(e)+Kd*filtered_derivative(e)。双击 `Variable gain PID` 可查看。没有执行器限幅，因此未加入抗积分饱和；本模型用于基础仿真演示。

## 三角隶属度配置（两个输入、三个输出一致）

| 集合 | 含义 | trimf 参数 [a b c] |
|---|---|---|
| NB | 负大 | [-3 -3 -2] |
| NM | 负中 | [-3 -2 -1] |
| NS | 负小 | [-2 -1 0] |
| ZO | 零 | [-1 0 1] |
| PS | 正小 | [0 1 2] |
| PM | 正中 | [1 2 3] |
| PB | 正大 | [2 3 3] |

相邻三角形在隶属度 0.5 处相交；NB/PB 是端点退化三角形，保证 ±3 的隶属度为 1。输入限幅保证不进入论域外。

49 条规则覆盖 e/ec 的全部 7×7 组合。教学规则采用对称幅值调度：误差大时增强比例、减弱积分；接近零误差且变化慢时增强积分；误差变化快时增强微分。规则矩阵见 `build_fuzzy_pid.m` 的 pRule/iRule/dRule，数字 1~7 对应 NB~PB。这些是示例规则，不是针对任意对象的最优整定。

## 文件

- simple_fuzzy_pid.slx：可运行 Simulink 模型。
- triangle_pid.fis：可编辑的三角隶属度和规则。
- triangular_memberships.png：五个变量的隶属度图。
- step_response.png：阶跃响应和 PID 增益曲线。
- model_diagram.png：顶层框图。
- build_fuzzy_pid.m：可复现构建、仿真和验证脚本。
- validation.txt：实测验证结果。
- simulation_results.mat：响应、增益和指标数据。

模块接口参考：[MathWorks Fuzzy Logic Controller 文档](https://www.mathworks.com/help/fuzzy/fuzzylogiccontroller.html)。
