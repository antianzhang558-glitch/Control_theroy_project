# 直流电机 MPC 第一版

打开 `dc_motor_mpc.slx`，点击 Run 即可仿真。同目录的 `dc_motor_mpc_controller.mat` 必须保留，模型 InitFcn 会自动载入控制器。已验证清除工作区控制器变量并重新打开模型后能够仿真。需要 Simulink、Control System Toolbox 和 Model Predictive Control Toolbox。

原来的 `dc_motor_open_loop.slx` 未被修改。新模型复用了其中电气、机械子系统，并在新模型中提供独立的电机参数。

## 从左往右看

- Speed_reference：0.2 s 时转速目标从 0 变为 60 rad/s。
- Reference_w_i：将目标组成 `[目标转速; 0]`。第二个数只是电流参考占位，因为电流跟踪权重为零；不是要求电流为零。
- MPC_Controller：mo 接 `[实测转速; 实测电流]`，ref 接参考值，mv 输出电压。qp.status 只用于检查优化求解状态。
- Electrical、Mechanical：连续电机对象，积分器分别产生电流和转速。
- Load_step_at_4s：4 s 时施加 0.2 N·m 外部负载，只接电机，未直接送给 MPC。
- Measured_w_i：把实测转速和电流反馈给控制器。
- 三个 Scope：分别观察目标与转速、电流、电压。

## 控制器设置

模型状态为 `[电流; 转速]`，测量输出顺序为 `[转速; 电流]`。连续模型经零阶保持准确离散化。采样周期 0.02 s，预测 60 步（1.2 s），控制时域 10 步。电压硬约束 0～12 V；预测电流硬约束 ±3.8 A。用 0.2 A 裕量并检查实际连续电流是否低于 4 A；这不是对任意扰动、模型误差的鲁棒保证。

转速、电流、电压缩放尺度分别为 60 rad/s、4 A、12 V。输出权重 `[1 0]`，输入大小权重 0，输入变化权重 0.1。MPC 工具箱权重作用于缩放后的量，不能直接和此前手算的原始单位权重逐项比较。

内部使用状态/扰动估计器，以及常值负载的随机游走模型。虽然电流和转速都测量，未知负载仍需估计；不是无观测器版本。电机 A、B 固定，不会随负载自动辨识更新。本次没有额外注入传感器噪声。

## 已运行的验证

使用最大 0.001 s 的连续求解步长，检查采样点之间的电流。

| 项目 | 空载 | 4 s 加入负载 |
|---|---:|---:|
| 8 s 转速 rad/s | 60.0000 | 60.1562 |
| 8 s 电流 A | 0.6000 | 2.6169 |
| 8 s 电压 V | 6.6000 | 8.6314 |
| 峰值电流 A | 3.8180 | 3.8180 |
| 峰值电压 V | 9.6492 | 9.6492 |

两组仿真所有 QP 求解状态均为正。负载后的最大转速下降约 6.77 rad/s；8 s 的剩余误差为 0.1562 rad/s，不应称为已经证明零稳态误差。后续可以研究扰动估计速度与权重调节。

原始结果见 `dc_motor_mpc_results.mat`，曲线见 `dc_motor_mpc_response.png`，建模与验证脚本为 `build_dc_motor_mpc.m`。脚本重新运行会创建带序号的新模型，不覆盖现有模型。

官方参考：
- https://www.mathworks.com/help/mpc/ref/mpccontroller.html
- https://www.mathworks.com/help/mpc/ref/mpc.setindist.html
- https://www.mathworks.com/help/mpc/ref/mpc.setoutdist.html
