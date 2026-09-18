%% 纯手工打造：基础模糊控制算法揭秘 (控制一阶热水器)
clear; clc; close all;

%% 1. 基础物理系统设置
dt = 0.05; 
time = 0:dt:10; 
N = length(time);
y = zeros(1, N); % 实际水温
u = zeros(1, N); % 加热器输出功率
target = 1.0;    % 目标温度

% 缩放因子 (决定控制器的敏感度)
Ke = 2.0;   % 误差放大倍数
Kec = 1.0;  % 变化率放大倍数
Ku = 2.0;   % 输出火力放大倍数

%% 2. 核心控制循环
for k = 2:N
    % 获取当前的精确物理误差 (Crisp Inputs)
    e = target - y(k-1);
    ec = (e - (target - y(max(1, k-2)))) / dt;
    
    % 缩放并限制在 [-1, 1] 的模糊论域内
    E_fuz = max(-1, min(1, e * Ke));
    EC_fuz = max(-1, min(1, ec * Kec));
    
    % =========================================================
    % 【步骤一：模糊化 Fuzzification】
    % 把具体的数字翻译成 N(负), Z(零), P(正) 三个模糊集合的隶属度
    % 数学原理：使用 max(0, 1 - abs(输入值 - 中心点)) 来生成交叉的三角形
    % =========================================================
    % 1. 对误差 E 的感觉
    mu_E_N = max(0, 1 - abs(E_fuz - (-1))); % 觉得"太热(超温)"的程度
    mu_E_Z = max(0, 1 - abs(E_fuz - 0));    % 觉得"刚好"的程度
    mu_E_P = max(0, 1 - abs(E_fuz - 1));    % 觉得"太冷"的程度
    
    % 2. 对误差变化率 EC 的感觉
    mu_EC_N = max(0, 1 - abs(EC_fuz - (-1))); % 觉得"正在快速升温"的程度
    mu_EC_Z = max(0, 1 - abs(EC_fuz - 0));    % 觉得"温度没变"的程度
    mu_EC_P = max(0, 1 - abs(EC_fuz - 1));    % 觉得"正在降温"的程度
    
    % =========================================================
    % 【步骤二：模糊推理 Inference】
    % 查老司机经验表，并用 MIN 取小原则计算每条规则的"话语权 (权重 W)"
    % =========================================================
    num = 0; % 重心法的天平分子
    den = 0; % 重心法的天平分母 (总权重)
    
    % 规则 1: IF 太冷(E_P) AND 温度没变(EC_Z) THEN 输出加热(P = 1.0)
    w1 = min(mu_E_P, mu_EC_Z);
    num = num + w1 * 1.0;  den = den + w1;
    
    % 规则 2: IF 太冷(E_P) AND 正在快速升温(EC_N) THEN 输出保持(Z = 0) -> 提前防超调
    w2 = min(mu_E_P, mu_EC_N);
    num = num + w2 * 0;    den = den + w2;
    
    % 规则 3: IF 刚好(E_Z) AND 温度没变(EC_Z) THEN 输出保持(Z = 0) -> 稳态
    w3 = min(mu_E_Z, mu_EC_Z);
    num = num + w3 * 0;    den = den + w3;
    
    % 规则 4: IF 太热(E_N) AND 温度没变(EC_Z) THEN 输出降温(N = -1.0)
    w4 = min(mu_E_N, mu_EC_Z);
    num = num + w4 * (-1.0); den = den + w4;
    
    % 规则 5: IF 刚好(E_Z) AND 正在降温(EC_P) THEN 输出微加热(P = 1.0) -> 防掉温
    w5 = min(mu_E_Z, mu_EC_P);
    num = num + w5 * 1.0;  den = den + w5;
    
    % (注：为保持代码极简，这里仅列出5条核心规则。未触发的规则权重为0，不影响重心)
    
    % =========================================================
    % 【步骤三：解模糊 Defuzzification】
    % 使用重心法 (Center of Gravity) 算出确切的物理控制量
    % =========================================================
    if den == 0
        u_raw = 0; % 防御性编程：如果没有规则被触发
    else
        u_raw = num / den; % 加权平均
    end
    
    % 将模糊输出作为"火力变化量"，累加到上一次的火力上 (增量式：查到0，意思是保持当前火力不变！)
    u(k) = u(k-1) + u_raw * Ku * dt;
    
    % =========================================================
    % 将火力输入一阶系统物理模型 (tau * y' + y = u, 假设 tau=1.5)
    % =========================================================
    tau = 1.5;
    y(k) = y(k-1) + (u(k) - y(k-1)) / tau * dt;
    
end

%% 3. 绘图
figure('Name', 'Pure Fuzzy Control');
plot(time, target*ones(1,N), 'k--', 'LineWidth', 1.5); hold on;
plot(time, y, 'r-', 'LineWidth', 2);
title('纯手工模糊控制算法 (一阶系统阶跃响应)');
xlabel('时间 (秒)'); ylabel('系统状态 y');
grid on; legend('目标值', '实际响应', 'Location', 'SouthEast');