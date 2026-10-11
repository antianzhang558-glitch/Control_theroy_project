function build_kalman_cv()
%BUILD_KALMAN_CV  生成 kalman_cv.slx —— 二维匀速(constant velocity)卡尔曼滤波模型
%
% 与 run_all_kalman 第 6 节是同一道题: 状态 [位置; 速度], 只测位置。
% 想说明两件事:
%   1) 向量卡尔曼和标量卡尔曼结构完全一样, 只是标量变成矩阵;
%   2) 只测位置也能估速度 —— 靠协方差矩阵的互相关项 P12 这座桥。
%
% 系统:
%   真值:  x(k+1) = F*x(k) + B*w(k),  w ~ N(0, sig_a^2),  F=[1 Ts;0 1], B=[Ts^2/2; Ts]
%   测量:  z(k)   = H*x(k) + v(k),    v ~ N(0, R),        H=[1 0]
%
% 卡尔曼滤波(向量形式, 与 run_all_kalman 的 kfRun 逐字对应):
%   预测: x^- = F*x            P^- = F*P*F' + Q
%   增益: S = H*P^-*H' + R     K = P^-*H'/S
%   校正: x = x^- + K*(z-H*x^-)   P = (I-K*H)*P^-
%
% 关于建模方式(和一维模型的差别, 值得对照看):
%   一维模型 kalman_simple.slx 用 Sum/Product/Divide 把五个方程一个个摆出来,
%   适合逐个模块对公式; 这个二维模型乘法是矩阵乘法, 同样的搭法要处理一大堆
%   "一维信号 vs 二维矩阵"的维度问题(Simulink 里常量 [1 0] 是一维,
%   矩阵乘法模块要求二维), 反而看不清算法。
%   所以这里改用 MATLAB Function 块: 代码就是矩阵形式的五个方程, 一眼能对上
%   run_all_kalman 第 6 节的公式, 也顺便示范工程上推荐的做法。
%
% 脚本不会覆盖已存在的模型: 要重建请先删除 kalman_cv.slx。

mdl = 'kalman_cv';

% ---------- 参数(与 run_all_kalman 第 6 节逐字一致) ----------
% 第 6 节的"真值"是正弦轨迹(速度近似常量), 所以滤波器给的 Q 很小,
% 表示"我基本相信速度不变" —— 这正是匀速模型的用法: Q 是"加速度有多少"的先验。
Ts    = 0.1;
v0    = -1.4;                                    % 初速度 m/s
acc0  = 0.05;                                    % 速度缓慢变化率 m/s^2
Fm    = [1 Ts; 0 1];
Bm    = [0.5*Ts^2; Ts];
Hm    = [1 0];
sig_a = 0.08;                                    % 滤波器假设的加速度扰动 std
Qm    = diag([0.25*Ts^4, Ts^2]) * sig_a^2;       % 与 run_all_kalman 第 6 节的 Q6 相同
R     = 0.25;                                    % 测量方差 (std = 0.5 m)
P0    = [1 0; 0 4];                              % 位置不确定 1 m, 速度不确定 2 m/s
Tstop = 20;
seed_a = 2026;
seed_v = 777;
% 注: Bm 只用于文档说明, 真值用标量方程搭(见下), 所以这里不引用它。
clear Bm

folder = fileparts(mfilename('fullpath'));
target = fullfile(folder, [mdl '.slx']);
if exist(target, 'file') || bdIsLoaded(mdl)
    fprintf('已存在 %s, 不覆盖。要重建请先删除该文件。\n', [mdl '.slx']);
    return;
end

new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(Ts), 'StopTime', num2str(Tstop));

% ================= 顶层 =================
P = [mdl '/真实对象与传感器'];
K = [mdl '/卡尔曼滤波器'];
mkSub(P, [40 120 400 430]);
mkSub(K, [450 60 1060 690]);

add_block('simulink/Commonly Used Blocks/Scope', [mdl '/看位置'], ...
    'Position', [1130 130 1180 270], 'NumInputPorts', '3');
add_block('simulink/Commonly Used Blocks/Scope', [mdl '/看速度和增益'], ...
    'Position', [1130 310 1180 450], 'NumInputPorts', '3');
add_block('simulink/Sinks/Display', [mdl '/K末值'], ...
    'Position', [1130 480 1230 550]);

toWS('kf2_x_true', [1130 590 1230 610]);
toWS('kf2_z_meas', [1130 630 1230 650]);
toWS('kf2_xhat',   [1130 670 1230 690]);
toWS('kf2_Kvec',   [1130 710 1230 730]);
toWS('kf2_Pmat',   [1130 750 1230 770]);

addNote('note1', '状态 = [位置; 速度], 只测位置。灰=测量  黑=真值  红=卡尔曼估计', [40 70 900 90]);
addNote('note2', '滤波器内部就是一个 MATLAB Function 块, 代码就是矩阵形式的五个方程(见 run_all_kalman 第 6 节)', ...
    [450 20 1130 40]);
addNote('note3', sprintf('参数: Ts=%.2g s, 加速度扰动 std=%.2g, 测量 std=%.2g, P0=[%.1f 0;0 %.1f]', ...
    Ts, sig_a, sqrt(R), P0(1,1), P0(2,2)), [450 710 1130 730]);

% ================= 真实对象与传感器 =================
% 真值: 一辆以 -1.4 m/s 匀速行驶的车, 速度按 0.05 m/s^2 缓慢变化, 再叠一个有界扰动:
%       速度: v(k) = v(k-1) + (0.05*Ts) + Ts*w(k)
%       位置: p(k) = p(k-1) + Ts*v(k-1)
%       p(0) = 0, v(0) = -1.4
%       这样"匀速"这个模型假设基本成立(只差 w 这一项), 所以滤波器给的 Q
%       就是"加速度有多少"的先验, 收敛的 K 才有明确物理含义。
% 测量: z(k) = p(k) + v_meas(k),   v_meas ~ N(0, R)
%
% 这一段故意用纯标量方程(两个积分器), 因为匀速模型的标量写法最直观,
% 也避免矩阵常量在 Simulink 里"一维 vs 二维"的歧义。
% 矩阵形式的五个方程由下面的 kfstep(MATLAB Function 块)实现。
add_block('simulink/Discrete/Unit Delay', [P '/速度延迟'], ...
    'Position', [40 120 120 160], 'InitialCondition', num2str(v0), ...
    'SampleTime', num2str(Ts));
add_block('simulink/Discrete/Unit Delay', [P '/位置延迟'], ...
    'Position', [40 240 120 280], 'InitialCondition', '0', ...
    'SampleTime', num2str(Ts));
add_block('simulink/Commonly Used Blocks/Constant', [P '/加速度'], ...
    'Position', [40 330 120 360], 'Value', num2str(0.05*Ts), 'OutDataTypeStr', 'double');
add_block('simulink/Sources/Random Number', [P '/速度扰动w'], ...
    'Position', [40 400 110 430], 'Mean', '0', 'Variance', num2str(sig_a^2), ...
    'Seed', num2str(seed_a), 'SampleTime', num2str(Ts));
add_block('simulink/Math Operations/Gain', [P '/乘Ts'], ...
    'Position', [160 400 210 430], 'Gain', num2str(Ts));
add_block('simulink/Math Operations/Sum', [P '/速度增量'], ...
    'Position', [260 360 280 430]);
setSum([P '/速度增量'], '|++');
add_block('simulink/Math Operations/Sum', [P '/速度递推'], ...
    'Position', [340 300 360 370]);
setSum([P '/速度递推'], '|++');
add_block('simulink/Math Operations/Gain', [P '/乘Ts位置'], ...
    'Position', [160 175 210 205], 'Gain', num2str(Ts));
add_block('simulink/Math Operations/Sum', [P '/位置递推'], ...
    'Position', [280 175 300 245]);
setSum([P '/位置递推'], '|++');
add_block('simulink/Math Operations/Sum', [P '/加测量噪声'], ...
    'Position', [400 250 420 320]);
setSum([P '/加测量噪声'], '|++');
add_block('simulink/Sources/Random Number', [P '/测量噪声v'], ...
    'Position', [280 490 350 520], 'Mean', '0', 'Variance', num2str(R), ...
    'Seed', num2str(seed_v), 'SampleTime', num2str(Ts));
add_block('simulink/Signal Routing/Mux', [P '/拼x_true'], ...
    'Position', [450 190 455 240], 'Inputs', '2');
add_block('simulink/Commonly Used Blocks/Out1', [P '/x_true'], ...
    'Position', [500 205 530 220], 'Port', '1');
add_block('simulink/Commonly Used Blocks/Out1', [P '/z'], ...
    'Position', [500 275 530 290], 'Port', '2');

% 位置: p(k) = p(k-1) + Ts*v(k-1)
L(P, '速度延迟/1', '乘Ts位置/1');
L(P, '乘Ts位置/1', '位置递推/1');
L(P, '位置延迟/1', '位置递推/2');
L(P, '位置递推/1', '位置延迟/1');
% 速度: v(k) = v(k-1) + 0.05*Ts + Ts*w(k)
L(P, '速度延迟/1', '速度递推/1');
L(P, '加速度/1', '速度增量/1');
L(P, '速度扰动w/1', '乘Ts/1');
L(P, '乘Ts/1', '速度增量/2');
L(P, '速度增量/1', '速度递推/2');
L(P, '速度递推/1', '速度延迟/1');
% 测量: z(k) = p(k) + v_meas(k)
L(P, '位置延迟/1', '加测量噪声/1');
L(P, '测量噪声v/1', '加测量噪声/2');
L(P, '位置延迟/1', '拼x_true/1');
L(P, '速度延迟/1', '拼x_true/2');
L(P, '拼x_true/1', 'x_true/1');
L(P, '加测量噪声/1', 'z/1');

% ================= 卡尔曼滤波器 =================
add_block('simulink/Commonly Used Blocks/In1', [K '/z'], ...
    'Position', [30 300 60 315], 'Port', '1');
bpk = [K '/kfstep'];
add_block('simulink/User-Defined Functions/MATLAB Function', bpk, ...
    'Position', [150 250 320 460]);
setChart(bpk, ...
    { 'z'; 'xprev'; 'Pprev'; 'f'; 'q'; 'h'; 'r' }, ...
    { '[1 1]'; '[2 1]'; '[2 2]'; '[2 2]'; '[2 2]'; '[1 2]'; '[1 1]' }, ...
    { 'xnew'; 'Pnew'; 'Kgain' }, ...
    { '[2 1]'; '[2 2]'; '[2 1]' }, ...
    sprintf(['xp = f*xprev;\n' ...
             'Pp = f*Pprev*f'' + q;\n' ...
             'S  = h*Pp*h'' + r;\n' ...
             'Kgain = Pp*h''/S;\n' ...
             'xnew = xp + Kgain*(z - h*xp);\n' ...
             'Pnew = (eye(2) - Kgain*h)*Pp;\n']));
add_block('simulink/Commonly Used Blocks/Constant', [K '/F'], ...
    'Position', [150 490 200 520], 'Value', mat2str(Fm), 'OutDataTypeStr', 'double');
add_block('simulink/Commonly Used Blocks/Constant', [K '/Q'], ...
    'Position', [150 540 200 570], 'Value', mat2str(Qm), 'OutDataTypeStr', 'double');
add_block('simulink/Commonly Used Blocks/Constant', [K '/H'], ...
    'Position', [150 590 200 620], 'Value', mat2str(Hm), 'OutDataTypeStr', 'double');
add_block('simulink/Commonly Used Blocks/Constant', [K '/R'], ...
    'Position', [150 640 200 670], 'Value', num2str(R), 'OutDataTypeStr', 'double');
add_block('simulink/Discrete/Unit Delay', [K '/状态延迟'], ...
    'Position', [390 280 470 320], 'InitialCondition', mat2str([0; 0]), ...
    'SampleTime', num2str(Ts));
add_block('simulink/Discrete/Unit Delay', [K '/协方差延迟'], ...
    'Position', [390 380 470 420], 'InitialCondition', mat2str(P0), ...
    'SampleTime', num2str(Ts));
add_block('simulink/Commonly Used Blocks/Out1', [K '/xhat'], ...
    'Position', [530 200 560 215], 'Port', '1');
add_block('simulink/Commonly Used Blocks/Out1', [K '/Pmat'], ...
    'Position', [530 300 560 315], 'Port', '2');
add_block('simulink/Commonly Used Blocks/Out1', [K '/Kvec'], ...
    'Position', [530 400 560 415], 'Port', '3');

L(K, 'z/1', 'kfstep/1');
L(K, '状态延迟/1', 'kfstep/2');
L(K, '协方差延迟/1', 'kfstep/3');
L(K, 'F/1', 'kfstep/4');
L(K, 'Q/1', 'kfstep/5');
L(K, 'H/1', 'kfstep/6');
L(K, 'R/1', 'kfstep/7');
L(K, 'kfstep/1', '状态延迟/1');
L(K, 'kfstep/2', '协方差延迟/1');
L(K, 'kfstep/1', 'xhat/1');
L(K, 'kfstep/2', 'Pmat/1');
L(K, 'kfstep/3', 'Kvec/1');

% ---------- 顶层连线 ----------
L(mdl, '真实对象与传感器/1', '看位置/1');
L(mdl, '真实对象与传感器/1', 'kf2_x_true/1');
L(mdl, '真实对象与传感器/2', '看位置/2');
L(mdl, '真实对象与传感器/2', 'kf2_z_meas/1');
L(mdl, '真实对象与传感器/2', '卡尔曼滤波器/1');
L(mdl, '卡尔曼滤波器/1', '看位置/3');
L(mdl, '卡尔曼滤波器/1', 'kf2_xhat/1');
L(mdl, '卡尔曼滤波器/2', '看速度和增益/1');
L(mdl, '卡尔曼滤波器/3', '看速度和增益/2');
L(mdl, '卡尔曼滤波器/3', 'K末值/1');
L(mdl, '卡尔曼滤波器/3', 'kf2_Kvec/1');
L(mdl, '卡尔曼滤波器/2', 'kf2_Pmat/1');

save_system(mdl, target);
fprintf('已生成 %s\n', target);
fprintf('参数: Ts=%.2g, 滤波器假设加速度 std=%.2g, 测量 std=%.2g, P0=[%.1f 0;0 %.1f], 仿真 %.0f s\n', ...
    Ts, sig_a, sqrt(R), P0(1,1), P0(2,2), Tstop);
fprintf('参考: run_all_kalman 第 6 节同一参数下, K 从 [0.806 0.310] 收敛到 [0.055 0.016]\n');
fprintf('运行 compare_simulink_kf 可以把本模型结果和 MATLAB 脚本逐点对照。\n');

% ================= 局部函数 =================
    function L(sys, src, dst)
        try
            add_line(sys, src, dst, 'autorouting', 'on');
        catch ME
            fprintf('连线失败: %s: %s -> %s (%s)\n', sys, src, dst, ME.message);
        end
    end

    function mkSub(path, pos)
        add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', pos);
        Simulink.SubSystem.deleteContents(path);
    end

    function toWS(varName, pos)
        add_block('simulink/Sinks/To Workspace', [mdl '/' varName], ...
            'Position', pos, 'VariableName', varName, 'SaveFormat', 'Timeseries');
    end

    function addNote(nm, txt, pos)
        add_block('built-in/Note', [mdl '/' nm], 'Position', pos, 'Text', txt);
    end

    function setSum(path, signs)
        try
            set_param(path, 'Inputs', signs);
        catch
            set_param(path, 'Inputs', signs(signs ~= '|'));
            fprintf('警告: %s 不支持 "%s", 已退回 "%s"\n', path, signs, signs(signs ~= '|'));
        end
    end

    function setChart(blkPath, inNames, inSizes, outNames, outSizes, body)
        % 用 Stateflow API 配置 MATLAB Function 块:
        % 先写函数体(Stateflow 会据此建出端口), 再删掉默认的 u/y 端口,
        % 最后按声明建立带显式维度的输入/输出数据对象。
        chart = sfroot().find('-isa', 'Stateflow.EMChart', '-and', 'Path', blkPath);
        if isempty(chart)
            error('找不到 MATLAB Function 块: %s', blkPath);
        end
        chart.Script = body;
        old = chart.find('-isa', 'Stateflow.Data');
        for i = numel(old):-1:1
            old(i).delete;
        end
        for i = 1:numel(inNames)
            dd = Stateflow.Data(chart);
            dd.Name = inNames{i};
            dd.Scope = 'Input';
            dd.Props.Type.Method = 'Built-in';
            dd.Props.Type.Primitive = 'double';
            dd.Props.Array.Size = inSizes{i};
        end
        for i = 1:numel(outNames)
            dd = Stateflow.Data(chart);
            dd.Name = outNames{i};
            dd.Scope = 'Output';
            dd.Props.Type.Method = 'Built-in';
            dd.Props.Type.Primitive = 'double';
            dd.Props.Array.Size = outSizes{i};
        end
    end
end
