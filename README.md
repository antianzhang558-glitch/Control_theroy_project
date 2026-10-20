# 控制理论 MATLAB / Simulink 实践

按控制主题和具体用途组织的学习项目。每个模型与配套脚本、参数、说明和必要结果放在一起，目录名不再包含聊天日期或时间戳。

## 主题导航

| 主题 | 内容与入口 |
|---|---|
| [状态反馈](状态反馈/README.md) | 二阶状态空间系统，使用 u=-Kx 让状态收敛到零 |
| [卡尔曼滤波](Kalman/README.md) | 一维温度、二维温度与变化速度、位置/速度估计及理论教学 |
| [状态观测器](状态观测器/README.md) | 全维状态观测器，包含简洁入门版 |
| [PID控制](PID控制/README.md) | 常规 PID、串级水箱与电机、模糊 PID |
| [LQR](LQR/README.md) | 单状态入门与直流电机 LQR |
| [MPC](MPC/README.md) | 基础模型、直流电机 MPC、人工参考与参考治理 |
| [经典控制](经典控制/README.md) | 超前滞后校正、Smith 与典型系统 |
| [模糊控制](模糊控制/README.md) | 模糊算法与 Mamdani FIS 设计 |

## 状态反馈与卡尔曼滤波入门

| 项目 | 模型 | 搭建说明 |
|---|---|---|
| [基础状态反馈](状态反馈/基础状态反馈/) | `simple_state_feedback.slx` | [使用说明](状态反馈/基础状态反馈/使用说明.md) |
| [一维温度估计](Kalman/基础模块入门/一维温度估计/) | `kalman_beginner_temperature.slx` | [基础模块逐步接线](Kalman/基础模块入门/一维温度估计/先看这个_初学者搭建说明.md) |
| [二维温度估计](Kalman/基础模块入门/二维温度估计/) | `kalman_beginner_2d.slx` | [搭建与性能评价](Kalman/基础模块入门/二维温度估计/二维入门_搭建与性能评价.md) |
| [位置速度估计](Kalman/位置速度估计/) | `kalman_position_velocity.slx` | [使用说明](Kalman/位置速度估计/使用说明.md) |

这四个模型直接打开即可仿真，不需要先执行生成脚本。入门版滤波器用基础模块展示预测、增益和校正；位置速度版使用 MATLAB Function。生成脚本用于重建模型，会覆盖已有配置，手工修改前先另存副本。

## 其他练习

- 串级控制先看 [简单水箱](PID控制/串级PID/简单水箱说明.md)，再看 [电机位置控制](PID控制/串级PID/README.md)。
- LQR 先看 [单状态入门](LQR/simple_lqr/README.md)，再看 [电机 LQR](LQR/motor_lqr/README.md)。
- MPC 先看 [基础与电机模型](MPC/入门模型/MPC电机模型说明.md)，再看 [跟踪MPC与参考治理](MPC/跟踪MPC与参考治理/simulink_demos/README.md)。
- 原卡尔曼理论教学继续保留在 `Kalman` 根目录：`run_all_kalman` 与 `compare_simulink_kf`；运行前切换到该目录。

## 环境与验证

Simulink 模型需要 MATLAB 和 Simulink。LQR/传递函数脚本、MPC、FIS 模型分别按各目录说明使用对应工具箱。基础模块卡尔曼滤波不依赖专门的卡尔曼滤波模块。

各项目保留已有响应图、必要数据和验证记录。目录整理后的检查见 [迁移验证](项目维护/验证结果.md)。历史验证结果对应各自模型与参数，不能直接当作手工改过模型的新结果。

## 后续组织规则

新内容先归入已有主题，跨聊天继续复用同一项目；根目录保留主题入口和仓库配置。技能 [content-project-layout](skills/content-project-layout/SKILL.md) 与 [AGENTS.md](AGENTS.md) 保存该约定。

个人学习笔记、简历、本地备份、MATLAB 缓存及运行日志保留在本机并由 `.gitignore` 排除，不作为模型项目上传。
