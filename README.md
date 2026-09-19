# 控制理论 MATLAB / Simulink 实践

控制理论学习项目，按主题组织模型、脚本、说明和结果图。进入一个目录即可找到该练习的相关文件。

## 从哪里开始

| 主题 | 学习内容 | 运行入口 |
|---|---|---|
| [基础 LQR](simple_lqr/README.md) | 单状态系统 dx/dt=u，理解 Q、R 与反馈增益 | `run_simple_lqr` |
| [直流电机 LQR](motor_lqr/README.md) | 转速与电流双状态反馈、工作点前馈 | `run_motor_lqr` |
| [PID](pid/README.md) | 三个已有 PID 练习模型 | 打开对应 `.slx` |
| [超前滞后校正](lead_lag/README.md) | 频域校正设计与 Simulink 练习 | 查看脚本后打开模型 |
| [Smith](smith/README.md) | Smith 控制练习 | `Smith.slx` |
| [典型系统](typical_system/README.md) | 典型系统练习 | `typical_system.slx` |
| [模糊控制](fuzzy_control/README.md) | 手写模糊算法及 FIS 控制模型 | `blur` / `blur_control.slx` |
| [模糊设计器](fuzzy_logic_designer/README.md) | Mamdani 隶属度函数和规则编辑 | `mamdanitype1.fis` |
| [三角隶属度模糊 PID](2026-09-19-模糊PID模型/README.md) | 7 个三角隶属度函数、49 条规则、在线调整 PID 增益 | `simple_fuzzy_pid.slx` |

学习 LQR 时，建议先完成基础 LQR，再进入电机 LQR。模糊控制可以先阅读 `blur.m`，再查看 FIS 规则及模糊 PID 示例。

## 环境与运行

- MATLAB、Simulink。
- LQR 和传递函数脚本需要 Control System Toolbox。
- 使用 FIS 的模型和设计器需要 Fuzzy Logic Toolbox。
- LQR 与三角隶属度模糊 PID 示例此前在 R2025b 验证。

下载或克隆仓库后，在 MATLAB 中将当前文件夹切换到要学习的主题目录。以仓库根目录为起点：

```matlab
cd('simple_lqr')
run_simple_lqr          % 默认 Q=1、R=1
run_simple_lqr(4,1)     % 增大状态偏差权重
```

随后切换到电机示例：

```matlab
cd('../motor_lqr')
run_motor_lqr           % 默认目标转速 10 rad/s
run_motor_lqr(20,10,1)  % 修改目标转速
```

模糊 PID：切换到 `2026-09-19-模糊PID模型`，打开 `simple_fuzzy_pid.slx` 并运行。保留模型与 `triangle_pid.fis` 在同一目录。详细规则、参数和重建方式见对应说明。

## 文件组织约定

- 每个主题目录包含自己的 `README.md`，模型和配套脚本、FIS、结果图放在一起。
- 保留原有模型和脚本名称，方便继续已有练习。
- 根目录只放总览和仓库配置；新实验优先放入对应主题或独立实验目录。
- 版本管理保留源文件、教学结果图及必要验证数据；忽略 `slprj/`、`.slxc`、日志、自动保存文件和本地归档备份。

## 验证与使用说明

两个 LQR 运行函数包含解析解对比检查。修改参数后，脚本会更新内存中的模型和响应图；已有 `.slx` 不会自动覆盖，可在 Simulink 中手动保存需要保留的参数。

模糊 PID 的验证记录见 [validation.txt](2026-09-19-模糊PID模型/validation.txt)。其构建脚本会重建模型与 FIS，覆盖该示例的已有配置，修改规则前请保留自己的版本。

本次整理仅调整目录与说明，没有修改控制算法，也没有重新执行全部仿真。原有 PID、校正、Smith、典型系统和模糊控制练习的初始化变量及具体运行条件，以各自脚本与模型为准。
