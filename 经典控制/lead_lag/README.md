# 超前滞后校正

| 文件 | 内容 |
|---|---|
| [lead_lag.m](lead_lag.m) | 对象 2/((s+2)(s²+s+1))，按静态增益和相位裕度设计校正器；滞后部分当前设为 1 |
| [practice1.m](practice1.m) | 对象 16/(s²+1.6s+16)，超前与滞后校正练习，计算闭环响应 |
| [lead_lad.slx](lead_lad.slx) | 原有 Simulink 校正练习，保留原文件名 |

## 使用

需要 MATLAB、Control System Toolbox；打开模型还需要 Simulink。在 MATLAB 中切换到本目录，先阅读并运行所选脚本，再查看工作区的 `G`、`G_c` 等变量。

两个脚本对应不同对象，且都执行 `clear; clc;`，后运行的脚本会替换工作区变量。运行 `lead_lad.slx` 前，检查模型参数是否与所选脚本一致，不能将两个设计的参数混用。

本次仅归档，未重新验证校正指标或模型仿真。

[返回项目导航](../../README.md)
