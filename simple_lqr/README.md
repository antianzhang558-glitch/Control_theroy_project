# 第一个 LQR 模型

此模型是理解 LQR 的单状态教学模型，还不是完整的电机模型。
需要 MATLAB、Simulink 和 Control System Toolbox（计算 lqr）。

## 打开与运行

在 MATLAB 中将当前文件夹切换到本文件所在目录，运行：

```matlab
run_simple_lqr
```

如果模型尚不存在，会自动创建 `simple_lqr_intro.slx`。
运行后打开模型，并绘制状态 x 和控制量 u。
模型保存了默认参数，直接打开 .slx 并按 Run 也可仿真；双击 Scope 观察两条曲线。

## 四个模块如何工作

- Plant：积分器，输入是 u=dx/dt，输出是 x，初始值 x0=10。
- LQR feedback：增益 -K，把当前 x 变成控制量 u=-Kx，并反馈给积分器。
- View x and u：Mux，仅将两个信号合并，方便观察。
- Scope：显示 x 与 u。

目标是 x 回到 0，所以无需 Step 模块。初始值 10 激发了系统的响应。
若初始值改成 0，且没有任何扰动，则两条曲线始终为 0。

## LQR 在哪里

模型为 dx/dt=u，因此 A=0、B=1。
MATLAB 通过 `K=lqr(0,1,Q,R)` 最小化从 0 到无穷大的累计代价
`integral(Q*x^2 + R*u^2)dt`，得到反馈增益。
对于这个特定模型，结果恰好是 K=sqrt(Q/R)。
LQR 的计算在仿真前完成；运行中的控制器只是增益块 -K。

默认 Q=R=1，所以 K=1，闭环为 dx/dt=-x。
理论曲线为 x(t)=10*exp(-t)，u(t)=-10*exp(-t)。
开始时 x=10、u=-10；随着 x 变小，控制作用也逐渐减弱。
1 秒时 x 约为 3.68，5 秒时 x 约为 0.0674。
这里只把 x 看作状态偏差、u 看作状态变化率，不把 u 当成实际电机电压。

## 只改一个参数，比较控制效果

```matlab
run_simple_lqr(1,1) % K=1，基础情况
run_simple_lqr(4,1) % K=2，更重视状态偏差，回零更快，初始 u=-20
run_simple_lqr(1,4) % K=0.5，更重视控制代价，回零更慢，初始 u=-5
```

脚本同时更新模型工作区中的 Q、R、K、x0；不会修改基础工作区。
参数实验不会自动覆盖磁盘上的默认模型，可在 Simulink 中自行保存。
不要只修改 Q 或 R 而不重新计算 K：实际参与反馈的是增益块使用的 K。
每次运行会更新 `simple_lqr_response.png`，并自动检查仿真是否符合理论解。

官方参考：https://www.mathworks.com/help/control/ref/lti.lqr.html

[返回项目导航](../README.md)
