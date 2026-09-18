# 第二课：两状态直流电机 LQR 调速

## 运行

在 MATLAB 当前文件夹切换到本目录，执行：

```matlab
run_motor_lqr
```

首次运行创建 motor_lqr_intro.slx；之后运行会加载这个模型。
需要 MATLAB、Simulink 和 Control System Toolbox。不需要 Simscape。
打开 .slx 后也可直接按 Run，双击 Speed scope、Current scope、Voltage scope 看曲线。
所有默认参数已经保存在模型工作区中，不依赖 MATLAB 基础工作区。

## 比上一课多了什么

上一课用积分器描述 dx/dt=u，只有一个状态。
这一课用直流电机模型，有两个状态：转速 omega、电流 i。
控制输入 u 是电枢电压，目标是从静止达到 10 rad/s（约 95.5 rpm）。
这是教学参数的理想线性模型，电压无限幅，无外部负载，状态均可获得。

## 电机的两个方程

机械部分：J*domega/dt = Kt*i - b*omega。
电气部分：L*di/dt = u - Ra*i - Ke*omega。

参数采用 SI 单位：

| 参数 | 数值 | 单位 |
|---|---:|---|
| 转动惯量 J | 0.01 | kg*m^2 |
| 黏性摩擦系数 b | 0.01 | N*m*s/rad |
| 转矩常数 Kt | 0.1 | N*m/A |
| 反电动势常数 Ke | 0.1 | V*s/rad |
| 电枢电阻 Ra | 1 | ohm |
| 电枢电感 L | 0.1 | H |

令 x=[omega;i]，则 dx/dt=A*x+B*u，其中：

```matlab
A = [-1  10;
     -1 -10];
B = [0;10];
Q = diag([10 0.1]);  % 转速偏差、电流偏差的权重
Ru = 1;             % 电压修正量的权重，区别于电阻 Ra
K = lqr(A,B,Q,Ru);   % K 是 1x2 行向量
```

State-Space 模块的 C=eye(2)、D=zeros(2,1)，所以同时输出转速和电流。
初始状态是 [0;0]。反馈需要知道两个状态；实际工程中需要测量或估计。

## 非零目标怎样得到

稳态时速度、电流均不再变化，令两个导数为零：

```matlab
omega_ref = 10;
i_ref = b*omega_ref/Kt;        % 1 A，用于克服摩擦
u_ref = Ra*i_ref+Ke*omega_ref; % 2 V
x_ref = [omega_ref;i_ref];
```

注意：虽然没有外部负载，但仍有黏性摩擦，所以需要非零电流和电压。
该参考工作点满足 A*x_ref+B*u_ref=0。

## 控制器怎样接

模型使用“目标减实际”的偏差，所以增益块使用正的 K：

```text
x_ref - x -> K -> delta_u
u = u_ref + delta_u
u -> DC motor -> x -> 返回偏差计算模块
```

展开就是：

u = u_ref + k_speed*(omega_ref-omega) + k_current*(i_ref-i)。

它与 u=u_ref-K*(x-x_ref) 完全相同。不要再额外加一个负号。
K 在仿真前计算一次，在整个仿真过程中不变。

LQR 最小化的是偏差的累计代价：
integral((x-x_ref)'*Q*(x-x_ref)+(u-u_ref)'*Ru*(u-u_ref))dt。
惩罚的是电压修正量，而不是把持续供给的稳态电压也算进无限时间成本。

## 应该看到什么

- 转速从 0 趋向 10 rad/s。
- 电流在加速过程中变化，最终趋向 1 A。
- 电压在开始时较高，随后最终趋向 2 V。
- 归零的是状态偏差和电压修正量，并不是实际转速、电流或总电压。

## 小实验

每次只改一个设置：

```matlab
run_motor_lqr(10,40,1) % 提高转速偏差权重，观察速度响应与电压
run_motor_lqr(10,10,4) % 提高电压修正代价，观察控制力度
run_motor_lqr(20,10,1) % 只改目标：K 不变，目标电流和基础电压随之变化
```

参数修改保留在当前内存模型中，不自动覆盖磁盘上的已有模型。
如需保留本次设置，可手动保存 Simulink 模型。
修改目标请使用脚本，以同时更新 x_ref、u_ref 和示波器参考值。
修改 Q、Ru 后也必须重新计算 K，仅改权重数值不会自动改变反馈。

脚本把仿真与矩阵指数解析解逐点比较，同时检查闭环稳定性、稳态工作点、
Riccati 方程及最终转速。每次运行会更新 motor_lqr_response.png。
模型不包含积分控制，未知负载或参数偏差下不能保证零稳态误差；这是之后的课程。

参考：
- https://www.mathworks.com/help/control/ug/dc-motor-control.html
- https://www.mathworks.com/help/simulink/slref/statespace.html
