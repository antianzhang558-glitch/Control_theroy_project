# 控制理论 MATLAB / Simulink 实践

控制理论学习项目，按主题组织模型、脚本、说明和结果图。进入一个目录即可找到该练习的相关文件。


## 2026-10-16目录导航

当前根目录按主题归类：[PID控制](PID控制/README.md)、[MPC](MPC/README.md)、[LQR](LQR/README.md)、[Kalman](Kalman/README.md)、[状态观测器](状态观测器/README.md)、[模糊控制](模糊控制/README.md)、[经典控制](经典控制/README.md)。
各练习目录整体移动，SLX、M脚本、FIS与成果一起保留；原有未提交修改与Git暂存内容未重置。

## 串级控制新练习

- **先学简单水箱**：[中文说明](PID控制/2026-09-20-串级PID电机控制/简单水箱说明.md)。打开 `simple_cascade_tank.slx`，观察“水位外环 → 目标流量 → 流量内环 → 水泵 → 水箱”。模型中使用中文标注，参数直接保存在模块中。
- **进阶电机位置控制**：[模型说明](PID控制/2026-09-20-串级PID电机控制/README.md)。打开 `cascade_pid_motor.slx`，学习位置 PID / 转速 PI、负载扰动、输出限幅和抗积分饱和。

两个模型均位于 `PID控制/2026-09-20-串级PID电机控制`，已在 MATLAB R2025b 中完成仿真；文件夹内附响应图及验证记录。仅需 MATLAB 和 Simulink。

## 最近新增的练习

| 主题 | 学习内容 | 运行入口 |
|---|---|---|
| [卡尔曼滤波教学](Kalman/README.md) | 从"两个高斯相乘 = 按精度加权平均"讲到五步递推、增益最优性、向量形式、可观测性与一致性检验（NIS/NEES） | `run_all_kalman` / `compare_simulink_kf` |
| [全维状态观测器入门](状态观测器/2026-09-21-全维观测器入门/README.md) | 只测位置的弹簧–质量–阻尼系统，估计位置与速度并做反馈 | `run_observer_demo` |
| [串级 PID 电机控制](PID控制/2026-09-20-串级PID电机控制/README.md) | 位置外环 + 转速内环，含负载扰动与抗积分饱和 | `run_cascade_pid` |
| [简单水箱入门](PID控制/2026-09-20-串级PID电机控制/简单水箱说明.md) | 串级结构的极简模型，先看这个 | `simple_cascade_tank.slx` |

卡尔曼滤波教学目录里的两个 Simulink 模型（一维标量、二维匀速）与 MATLAB 脚本做了逐点对照：因为 K 和 P 的递推不含随机量，二维模型的增益序列与脚本**差 0.00e+00**。详见该目录的 `validation.txt`、`validation_simulink.txt`。

## 从哪里开始

| 主题 | 学习内容 | 运行入口 |
|---|---|---|
| [基础 LQR](LQR/simple_lqr/README.md) | 单状态系统 dx/dt=u，理解 Q、R 与反馈增益 | `run_simple_lqr` |
| [直流电机 LQR](LQR/motor_lqr/README.md) | 转速与电流双状态反馈、工作点前馈 | `run_motor_lqr` |
| [PID](PID控制/pid/README.md) | 三个已有 PID 练习模型 | 打开对应 `.slx` |
| [超前滞后校正](经典控制/lead_lag/README.md) | 频域校正设计与 Simulink 练习 | 查看脚本后打开模型 |
| [Smith](经典控制/smith/README.md) | Smith 控制练习 | `Smith.slx` |
| [典型系统](经典控制/typical_system/README.md) | 典型系统练习 | `typical_system.slx` |
| [模糊控制](模糊控制/fuzzy_control/README.md) | 手写模糊算法及 FIS 控制模型 | `blur` / `blur_control.slx` |
| [模糊设计器](模糊控制/fuzzy_logic_designer/README.md) | Mamdani 隶属度函数和规则编辑 | `mamdanitype1.fis` |
| [三角隶属度模糊 PID](PID控制/2026-09-19-模糊PID模型/README.md) | 7 个三角隶属度函数、49 条规则、在线调整 PID 增益 | `simple_fuzzy_pid.slx` |
| [卡尔曼滤波](Kalman/README.md) | 按精度加权平均、增益最优性与收敛、向量形式、可观测性、NIS/NEES 校验 | `run_all_kalman` |

学习 LQR 时，建议先完成基础 LQR，再进入电机 LQR。模糊控制可以先阅读 `blur.m`，再查看 FIS 规则及模糊 PID 示例。

## 环境与运行

- MATLAB、Simulink。
- LQR 和传递函数脚本需要 Control System Toolbox。
- 使用 FIS 的模型和设计器需要 Fuzzy Logic Toolbox。
- 卡尔曼滤波教学不需要任何工具箱；全维观测器入门按该目录说明也不需要 Control System Toolbox。
- LQR、三角隶属度模糊 PID、卡尔曼滤波与观测器示例此前在 R2025b 验证。

下载或克隆仓库后，在 MATLAB 中将当前文件夹切换到要学习的主题目录。以仓库根目录为起点：

```matlab
cd('LQR/simple_lqr')
run_simple_lqr          % 默认 Q=1、R=1
run_simple_lqr(4,1)     % 增大状态偏差权重
```

随后切换到电机示例：

```matlab
cd('../motor_lqr')
run_motor_lqr           % 默认目标转速 10 rad/s
run_motor_lqr(20,10,1)  % 修改目标转速
```

模糊 PID：切换到 `PID控制/2026-09-19-模糊PID模型`，打开 `simple_fuzzy_pid.slx` 并运行。保留模型与 `triangle_pid.fis` 在同一目录。详细规则、参数和重建方式见对应说明。

卡尔曼滤波：切换到 `Kalman`，执行 `run_all_kalman`（约 20 秒，出图与验证记录），再执行 `compare_simulink_kf`（约 30 秒，Simulink 模型与脚本逐点对照）。

```matlab
cd('../../Kalman')
run_all_kalman          % 9 张图 + validation.txt + kalman_summary.txt
compare_simulink_kf     % validation_simulink.txt + 模型结构快照
```

## 文件组织约定

- 每个主题目录包含自己的 `README.md`，模型和配套脚本、FIS、结果图放在一起。
- 保留原有模型和脚本名称，方便继续已有练习。
- 根目录只放总览和仓库配置；新实验优先放入对应主题或独立实验目录。
- 版本管理保留源文件、教学结果图及必要验证数据；忽略 `slprj/`、`.slxc`、日志、自动保存文件和本地归档备份。

## 验证与使用说明

两个 LQR 运行函数包含解析解对比检查。修改参数后，脚本会更新内存中的模型和响应图；已有 `.slx` 不会自动覆盖，可在 Simulink 中手动保存需要保留的参数。

模糊 PID 的验证记录见 [validation.txt](PID控制/2026-09-19-模糊PID模型/validation.txt)。其构建脚本会重建模型与 FIS，覆盖该示例的已有配置，修改规则前请保留自己的版本。

卡尔曼滤波教学的验证记录见 [validation.txt](Kalman/validation.txt)（18 项自动检查）与 [validation_simulink.txt](Kalman/validation_simulink.txt)（Simulink 与 MATLAB 逐点对照）。两个 `.slx` 由 `build_kalman_simple`、`build_kalman_cv` 生成，脚本不会覆盖已存在的模型。

本次整理仅调整目录与说明，没有修改控制算法，也没有重新执行全部仿真。原有 PID、校正、Smith、典型系统和模糊控制练习的初始化变量及具体运行条件，以各自脚本与模型为准。卡尔曼滤波部分重做为教学版（`Kalman`），原 `2026-09-22-卡尔曼滤波入门` 目录已被其替代。
