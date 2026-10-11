function build_kalman_simple()
%BUILD_KALMAN_SIMPLE  生成 kalman_simple.slx —— 一维(标量)卡尔曼滤波 Simulink 模型
%
% 与 run_all_kalman 第 3~5 节用的是同一套方程, 参数直接写进模块,
% 运行模型不需要初始化工作区变量。
%
% 系统(标量):
%   真值:  x(k+1) = a*x(k) + w(k),   w ~ N(0, Q)
%   测量:  y(k)   = x(k)   + v(k),   v ~ N(0, R)
%
% 卡尔曼滤波(标量):
%   预测:  xminus = a*xhat_prev          Pminus = a^2*P_prev + Q
%   校正:  K = Pminus/(Pminus+R)         xhat = xminus + K*(y-xminus)
%          P = (1-K)*Pminus
%
% 模型结构:
%   真实对象与传感器 -> y -> 卡尔曼滤波器(预测 + 校正) -> xhat, K, Pminus
%
% 所有 Sum 都用 "|" 形式(端口同侧竖排), 连线不交叉。
% 脚本不会覆盖已存在的模型: 要重建请先删除 kalman_simple.slx。

mdl = 'kalman_simple';

% ---------- 参数(与 run_all_kalman 第 3 节一致, 方便手算核对) ----------
Ts    = 0.1;     % 采样周期 s
a     = 0.98;    % 状态转移系数
Q     = 0.25;    % 过程噪声方差
R     = 1.00;    % 测量噪声方差
P0    = 1.00;    % 估计误差方差初值
x0    = 1.00;    % 真值初值
xhat0 = 0.00;    % 估计初值(故意猜错)
Tstop = 30;      % 仿真时长 s
seed_w = 23341;
seed_v = 9871;

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

mkSub(P, [40 90 380 350]);
mkSub(K, [420 90 1120 610]);

add_block('simulink/Commonly Used Blocks/Scope', [mdl '/看三条曲线'], ...
    'Position', [1180 110 1230 250], 'NumInputPorts', '3');
add_block('simulink/Commonly Used Blocks/Scope', [mdl '/看K和P'], ...
    'Position', [1180 290 1230 410], 'NumInputPorts', '2');
add_block('simulink/Sinks/Display', [mdl '/K稳态值'], ...
    'Position', [1180 440 1270 470]);

toWS('kf1_x_true', [1180 490 1270 510]);
toWS('kf1_y_meas', [1180 530 1270 550]);
toWS('kf1_xhat',   [1180 570 1270 590]);
toWS('kf1_K',      [1180 610 1270 630]);
toWS('kf1_Pminus', [1180 650 1270 670]);

addNote('note1', '① 灰=测量(脏)  黑=真值  红=卡尔曼估计', [40 40 700 60]);
addNote('note2', '② K 由 P 自己递推出来, 与随机数据无关(见 run_all_kalman 第 4 节)', [420 30 1120 50]);
addNote('note3', '③ 参数: a=0.98, Q=0.25, R=1, P0=1, Ts=0.1s', [420 640 1120 660]);

% ================= 真实对象与传感器 =================
add_block('simulink/Discrete/Unit Delay', [P '/真值延迟'], ...
    'Position', [30 90 100 120], 'InitialCondition', num2str(x0), 'SampleTime', num2str(Ts));
add_block('simulink/Math Operations/Gain', [P '/a'], ...
    'Position', [140 80 190 110], 'Gain', num2str(a));
add_block('simulink/Math Operations/Sum', [P '/状态递推'], ...
    'Position', [215 90 235 160]);
setSum([P '/状态递推'], '|++');
add_block('simulink/Sources/Random Number', [P '/过程噪声w'], ...
    'Position', [120 190 190 220], 'Mean', '0', 'Variance', num2str(Q), ...
    'Seed', num2str(seed_w), 'SampleTime', num2str(Ts));
add_block('simulink/Sources/Random Number', [P '/测量噪声v'], ...
    'Position', [120 260 190 290], 'Mean', '0', 'Variance', num2str(R), ...
    'Seed', num2str(seed_v), 'SampleTime', num2str(Ts));
add_block('simulink/Math Operations/Sum', [P '/加测量噪声'], ...
    'Position', [215 235 235 305]);
setSum([P '/加测量噪声'], '|++');
add_block('simulink/Commonly Used Blocks/Out1', [P '/x_true'], ...
    'Position', [265 95 295 110], 'Port', '1');
add_block('simulink/Commonly Used Blocks/Out1', [P '/y'], ...
    'Position', [265 265 295 280], 'Port', '2');

L(P, '真值延迟/1', 'a/1');
L(P, '真值延迟/1', '加测量噪声/1');
L(P, '真值延迟/1', 'x_true/1');
L(P, 'a/1', '状态递推/1');
L(P, '过程噪声w/1', '状态递推/2');
L(P, '状态递推/1', '真值延迟/1');
L(P, '测量噪声v/1', '加测量噪声/2');
L(P, '加测量噪声/1', 'y/1');

% ================= 卡尔曼滤波器 =================
PR = [K '/预测'];
UP = [K '/校正'];

add_block('simulink/Commonly Used Blocks/In1', [K '/y'], ...
    'Position', [30 240 60 255], 'Port', '1');
mkSub(PR, [110 40 470 210]);
mkSub(UP, [110 260 560 540]);
add_block('simulink/Discrete/Unit Delay', [K '/估计延迟'], ...
    'Position', [600 290 680 320], 'InitialCondition', num2str(xhat0), ...
    'SampleTime', num2str(Ts));
add_block('simulink/Discrete/Unit Delay', [K '/方差延迟'], ...
    'Position', [600 380 680 410], 'InitialCondition', num2str(P0), ...
    'SampleTime', num2str(Ts));
add_block('simulink/Commonly Used Blocks/Out1', [K '/xhat'], ...
    'Position', [600 110 630 125], 'Port', '1');
add_block('simulink/Commonly Used Blocks/Out1', [K '/K'], ...
    'Position', [600 180 630 195], 'Port', '2');
add_block('simulink/Commonly Used Blocks/Out1', [K '/Pminus'], ...
    'Position', [600 250 630 265], 'Port', '3');

% ---------- 预测: x^- = a*xhat_prev,  P^- = a^2*P_prev + Q ----------
add_block('simulink/Commonly Used Blocks/In1', [PR '/xhat_prev'], ...
    'Position', [20 40 50 55], 'Port', '1');
add_block('simulink/Commonly Used Blocks/In1', [PR '/P_prev'], ...
    'Position', [20 120 50 135], 'Port', '2');
add_block('simulink/Math Operations/Gain', [PR '/a'], ...
    'Position', [100 35 155 65], 'Gain', num2str(a));
add_block('simulink/Math Operations/Gain', [PR '/a^2'], ...
    'Position', [100 110 165 140], 'Gain', num2str(a^2));
add_block('simulink/Commonly Used Blocks/Constant', [PR '/Q'], ...
    'Position', [100 165 150 188], 'Value', num2str(Q));
add_block('simulink/Math Operations/Sum', [PR '/Pminus求和'], ...
    'Position', [215 115 235 180]);
setSum([PR '/Pminus求和'], '|++');
add_block('simulink/Commonly Used Blocks/Out1', [PR '/xminus'], ...
    'Position', [215 45 245 60], 'Port', '1');
add_block('simulink/Commonly Used Blocks/Out1', [PR '/Pminus'], ...
    'Position', [290 140 320 155], 'Port', '2');

L(PR, 'xhat_prev/1', 'a/1');
L(PR, 'a/1', 'xminus/1');
L(PR, 'P_prev/1', 'a^2/1');
L(PR, 'a^2/1', 'Pminus求和/1');
L(PR, 'Q/1', 'Pminus求和/2');
L(PR, 'Pminus求和/1', 'Pminus/1');

% ---------- 校正: K = P^-/(P^-+R),  xhat = x^-+K*(y-x^-),  P = (1-K)*P^- ----------
add_block('simulink/Commonly Used Blocks/In1', [UP '/y'], ...
    'Position', [15 40 45 55], 'Port', '1');
add_block('simulink/Commonly Used Blocks/In1', [UP '/xminus'], ...
    'Position', [15 100 45 115], 'Port', '2');
add_block('simulink/Commonly Used Blocks/In1', [UP '/Pminus'], ...
    'Position', [15 160 45 175], 'Port', '3');
add_block('simulink/Math Operations/Sum', [UP '/新息'], ...
    'Position', [95 60 125 115]);
setSum([UP '/新息'], '|+-');                       % y - xminus
add_block('simulink/Math Operations/Product', [UP '/K乘新息'], ...
    'Position', [180 65 220 105], 'Inputs', '**');
add_block('simulink/Math Operations/Sum', [UP '/估计更新'], ...
    'Position', [270 60 300 115]);
setSum([UP '/估计更新'], '|++');                   % xminus + K*(y-xminus)
add_block('simulink/Commonly Used Blocks/Out1', [UP '/xhat'], ...
    'Position', [350 70 380 85], 'Port', '1');
add_block('simulink/Commonly Used Blocks/Constant', [UP '/R'], ...
    'Position', [95 175 145 198], 'Value', num2str(R));
add_block('simulink/Math Operations/Sum', [UP '/Pminus+R'], ...
    'Position', [180 150 210 215]);
setSum([UP '/Pminus+R'], '|++');
add_block('simulink/Math Operations/Divide', [UP '/算K'], ...
    'Position', [255 155 295 205], 'Inputs', '*/'); % 端口1 = Pminus, 端口2 = Pminus+R
add_block('simulink/Commonly Used Blocks/Out1', [UP '/K'], ...
    'Position', [350 165 380 180], 'Port', '2');
add_block('simulink/Commonly Used Blocks/Constant', [UP '/1'], ...
    'Position', [95 245 145 268], 'Value', '1');
add_block('simulink/Math Operations/Sum', [UP '/1-K'], ...
    'Position', [180 230 210 290]);
setSum([UP '/1-K'], '|+-');
add_block('simulink/Math Operations/Product', [UP '/方差更新'], ...
    'Position', [255 240 295 280], 'Inputs', '**');
add_block('simulink/Commonly Used Blocks/Out1', [UP '/P'], ...
    'Position', [350 250 380 265], 'Port', '3');

L(UP, 'y/1', '新息/1');
L(UP, 'xminus/1', '新息/2');
L(UP, 'xminus/1', '估计更新/1');
L(UP, 'Pminus/1', 'Pminus+R/1');
L(UP, 'R/1', 'Pminus+R/2');
L(UP, 'Pminus/1', '算K/1');
L(UP, 'Pminus+R/1', '算K/2');
L(UP, '算K/1', 'K/1');
L(UP, '算K/1', 'K乘新息/1');
L(UP, '新息/1', 'K乘新息/2');
L(UP, 'K乘新息/1', '估计更新/2');
L(UP, '估计更新/1', 'xhat/1');
L(UP, '1/1', '1-K/1');
L(UP, '算K/1', '1-K/2');
L(UP, '1-K/1', '方差更新/1');
L(UP, 'Pminus/1', '方差更新/2');
L(UP, '方差更新/1', 'P/1');

% ---------- 卡尔曼滤波器内部 ----------
L(K, '估计延迟/1', '预测/1');
L(K, '方差延迟/1', '预测/2');
L(K, 'y/1', '校正/1');
L(K, '预测/1', '校正/2');
L(K, '预测/2', '校正/3');
L(K, '预测/2', 'Pminus/1');
L(K, '校正/1', 'xhat/1');
L(K, '校正/1', '估计延迟/1');
L(K, '校正/2', 'K/1');
L(K, '校正/3', '方差延迟/1');

% ---------- 顶层 ----------
L(mdl, '真实对象与传感器/1', '看三条曲线/1');
L(mdl, '真实对象与传感器/1', 'kf1_x_true/1');
L(mdl, '真实对象与传感器/2', '看三条曲线/2');
L(mdl, '真实对象与传感器/2', 'kf1_y_meas/1');
L(mdl, '真实对象与传感器/2', '卡尔曼滤波器/1');
L(mdl, '卡尔曼滤波器/1', '看三条曲线/3');
L(mdl, '卡尔曼滤波器/1', 'kf1_xhat/1');
L(mdl, '卡尔曼滤波器/2', '看K和P/1');
L(mdl, '卡尔曼滤波器/2', 'K稳态值/1');
L(mdl, '卡尔曼滤波器/2', 'kf1_K/1');
L(mdl, '卡尔曼滤波器/3', '看K和P/2');
L(mdl, '卡尔曼滤波器/3', 'kf1_Pminus/1');

save_system(mdl, target);
fprintf('已生成 %s\n', target);
fprintf('参数: a=%.2f, Q=%.2f, R=%.2f, P0=%.1f, Ts=%.2g, 仿真 %.0f s\n', ...
    a, Q, R, P0, Ts, Tstop);
fprintf('解析稳态: Kinf = 0.381244, P^-inf = 0.616147 (见 run_all_kalman 第 3 节)\n');
fprintf('运行 compare_simulink_kf 可以把本模型的结果和 MATLAB 脚本逐点对照。\n');

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
end
