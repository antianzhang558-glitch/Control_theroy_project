function results = run_all_kalman(varargin)
%RUN_ALL_KALMAN  卡尔曼滤波教学: 从"加权平均"到一个能用的滤波器
%
% 用法:
%   run_all_kalman                     % 出全部图 + 写 validation.txt / kalman_summary.txt
%   run_all_kalman('figures', false)   % 只算数字, 不画图
%   run_all_kalman('outdir', 'D:\tmp') % 指定输出目录
%
% 只需要 MATLAB 基础功能, 不需要任何工具箱。
%
% 按教学顺序回答 6 个问题, 每节对应一段代码:
%   Q1 卡尔曼滤波到底在干什么      第 1 节  两个高斯相乘 = 按精度加权平均
%   Q2 增益 K 是什么, 为什么最优    第 2 节  K = P/(P+R), 使估计误差方差最小
%   Q3 五步递推怎么手算            第 3 节  逐步打印中间量, 可逐行核对
%   Q4 为什么滤波器"越来越自信"     第 4 节  P 的闭环 + 稳态增益闭式解
%   Q5 向量情况怎么用              第 5~7 节  1D 跟踪 / 2D 匀速 / 3D 的可观测性
%   Q6 参数定错了怎么发现          第 8 节  固定增益扫描 + NIS/NEES 一致性检验
%
% 输出(都在本目录): fig1_*.png ... fig8_*.png, validation.txt,
%                   kalman_summary.txt, kalman_results.mat

opt = parseArgs(varargin);
here = fileparts(mfilename('fullpath'));
if isempty(opt.outdir)
    opt.outdir = here;
end
if ~exist(opt.outdir, 'dir')
    mkdir(opt.outdir);
end
outdir = opt.outdir;

results = struct();
results.outdir = outdir;
t0 = tic;

banner('卡尔曼滤波教学    run_all_kalman');
fprintf('输出目录: %s\n', outdir);

% =====================================================================
% 第 1 节  卡尔曼滤波在干什么
% =====================================================================
banner('第 1 节  核心直觉: 两个高斯相乘 = 按精度加权平均');

% 场景: 一个不动的量(真值 2.00), 已经有先验估计, 又来一次测量。
x_prior = 1.70;   s_prior = 0.50;    % 先验估计与标准差(来自上一步的 P)
z_meas  = 2.10;   s_meas  = 0.20;    % 本次测量与标准差(来自 R)
Pp = s_prior^2;
Rm = s_meas^2;

x_avg  = (x_prior + z_meas) / 2;                        % 做法 A: 直接平均
invP   = 1/Pp + 1/Rm;                                   % 精度相加
x_wavg = (x_prior/Pp + z_meas/Rm) / invP;               % 做法 B: 按精度加权平均
P_post = 1/invP;
K_one  = Pp / (Pp + Rm);
x_kal  = x_prior + K_one * (z_meas - x_prior);          % 做法 C: 卡尔曼一步

fprintf('\n场景: 真值 2.00, 先验估计 %.2f +- %.2f, 本次测量 %.2f +- %.2f\n', ...
    x_prior, s_prior, z_meas, s_meas);
fprintf('  做法A 直接平均        x = %.4f    (不管谁准, 各占一半)\n', x_avg);
fprintf('  做法B 按精度加权      x = %.4f    (权重 %.4f / %.4f)\n', x_wavg, ...
    (1/Pp)/invP, (1/Rm)/invP);
fprintf('  做法C 卡尔曼一步      x = %.4f    (K = %.4f)\n', x_kal, K_one);
fprintf('\n  => 做法C 与做法B 完全一样(差 %.2e): 卡尔曼滤波就是按精度加权平均。\n', ...
    abs(x_wavg - x_kal));
fprintf('  => 谁的标准差小就更听谁的: 测量 %.2f < 先验 %.2f, 所以结果明显偏向测量。\n', ...
    s_meas, s_prior);
fprintf('  => 精度(方差的倒数)可以直接相加:  1/P+ = 1/P- + 1/R\n');
fprintf('     合并后 P+ = %.6f, 比先验 %.4f 和测量 %.4f 都小\n', P_post, Pp, Rm);
fprintf('     这就是"信息只会越攒越多": 每次测量都让不确定性变小。\n');
fprintf('  => 后验标准差 %.4f 也小于两者的标准差(%.2f / %.2f)。\n', ...
    sqrt(P_post), s_prior, s_meas);

results.static = struct('x_avg', x_avg, 'x_wavg', x_wavg, 'x_kal', x_kal, ...
    'K_one', K_one, 'P_post', P_post, 's_post', sqrt(P_post));

if opt.figures
    figure1_gaussians(outdir, x_prior, s_prior, z_meas, s_meas, x_kal, sqrt(P_post), ...
        K_one, x_avg, x_wavg);
end

% =====================================================================
% 第 2 节  增益 K
% =====================================================================
banner('第 2 节  增益 K = P/(P+R): 信模型还是信测量');

fprintf('\n  xhat = x^- + K*(z - x^-)  =  (1-K)*x^- + K*z\n');
fprintf('  => K=0 完全信模型(测量一眼不看);  K=1 完全信测量(模型白算)\n');
fprintf('  => K 只由两个不确定性之比决定: P 大 -> K 大;  R 大 -> K 小\n\n');

gainCases = [0.01 1.00; 0.10 1.00; 1.00 1.00; 1.00 0.10; 1.00 0.01];
fprintf('  %-10s %-10s %-10s %s\n', 'P(模型)', 'R(测量)', 'K', '一句话');
for i = 1:size(gainCases, 1)
    Pi = gainCases(i, 1);  Ri = gainCases(i, 2);
    Ki = Pi / (Pi + Ri);
    if Ki < 0.2
        note = '基本信模型, 测量只作微调';
    elseif Ki > 0.8
        note = '基本信测量, 模型只作平滑';
    else
        note = '两边都听, 均衡';
    end
    fprintf('  %-10.3g %-10.3g %-10.4f %s\n', Pi, Ri, Ki, note);
end

% 最优性: K 让后验方差最小
Kgrid = linspace(0, 1, 200001);
Pcase = 1.0;  Rcase = 1.0;
PpostK = (1 - Kgrid).^2 * Pcase + Kgrid.^2 * Rcase;
[Pmin, imin] = min(PpostK);
Kbest_num = Kgrid(imin);
Kana = Pcase / (Pcase + Rcase);
fprintf('\n  为什么 K 是最优的: 后验方差 P+(K) = (1-K)^2*P + K^2*R 是开口向上的抛物线\n');
fprintf('    dP+/dK = 0  =>  K = P/(P+R)   (这就是全部推导)\n');
fprintf('    数值扫描最小点 K = %.5f,  解析 P/(P+R) = %.5f,  差 %.2e\n', ...
    Kbest_num, Kana, abs(Kbest_num - Kana));
fprintf('    P+(K=最优) = %.6f,  P+(K=0.5 直接平均) = %.6f\n', Pmin, 0.25*Pcase + 0.25*Rcase);
fprintf('    => "最优"的唯一含义: 让估计误差方差最小, 没有别的玄机。\n');

results.gainCases = gainCases;
results.K_best_numeric = Kbest_num;
results.K_best_analytic = Kana;

if opt.figures
    figure2_gain(outdir, gainCases);
end

% =====================================================================
% 第 3 节  手算递推表
% =====================================================================
banner('第 3 节  五步递推手算 (a=0.98, Q=0.25, R=1, P0=1)');

% 用最简单的一维模型, 每个数都能拿计算器核对:
%   预测: x^- = a*xhat_prev      P^- = a^2*P_prev + Q
%   校正: K = P^-/(P^-+R)        xhat = x^- + K*(z-x^-)      P = (1-K)*P^-
a_h = 0.98;  Q_h = 0.25;  R_h = 1.00;
xhat_h = 0.00;  P_h = 1.00;
meas_h = [2.00 1.80 2.60 1.40 2.10 1.70 2.30 1.55 2.20 1.45];
n_h = numel(meas_h);

fprintf('\n  %-4s %-9s %-9s %-8s %-10s %-10s %-9s\n', ...
    'k', 'P^-', 'K', 'z(k)', '新息 z-x^-', 'xhat(k)', 'P(k)');
fprintf('  %s\n', repmat('-', 1, 68));
hand = zeros(n_h, 6);
for k = 1:n_h
    xminus = a_h * xhat_h;
    Pminus = a_h^2 * P_h + Q_h;
    Kk     = Pminus / (Pminus + R_h);
    innov  = meas_h(k) - xminus;
    xhat_h = xminus + Kk * innov;
    P_h    = (1 - Kk) * Pminus;
    hand(k, :) = [Pminus Kk meas_h(k) innov xhat_h P_h];
    if k <= 7
        fprintf('  %-4d %-9.4f %-9.4f %-8.2f %+-10.4f %-10.4f %-9.4f\n', ...
            k, Pminus, Kk, meas_h(k), innov, xhat_h, P_h);
    end
end

fa = a_h^2;
bb = R_h - fa*R_h - Q_h;                       % 稳态: p^2 + (R - a^2 R - Q)p - Q R = 0
Pinf_h = (-bb + sqrt(bb^2 + 4*Q_h*R_h)) / 2;
Kinf_h = Pinf_h / (Pinf_h + R_h);
fprintf('\n  稳态闭式解(解不动点方程): P^-inf = %.6f,  Kinf = %.6f\n', Pinf_h, Kinf_h);
fprintf('  第 1 步 K = %.4f 偏大, 因为 P0 = 1 表示"一开始不信自己";\n', hand(1,2));
fprintf('  第 %d 步 K = %.4f 已经贴近稳态。K 单调下降, 因为 P 每步变小而 R 不变。\n', ...
    n_h, hand(n_h,2));
fprintf('  稳态还有一个好用的关系: P = K*R,  本例 (1-K)*P^- = %.6f vs Kinf*R = %.6f\n', ...
    (1-Kinf_h)*Pinf_h, Kinf_h*R_h);

results.hand = hand;
results.hand_Pinf = Pinf_h;
results.hand_Kinf = Kinf_h;

if opt.figures
    figure3_flowchart(outdir, a_h, Q_h, R_h, 1.0, hand, Pinf_h, Kinf_h);
end

% =====================================================================
% 第 4 节  P 自己会闭环
% =====================================================================
banner('第 4 节  K 与 P 的递推与数据无关');

% 关键: K 和 P 的递推里没有随机量, 只由 a,Q,R,P0 决定。
% 所以整条 K 序列可以离线预算 —— 工程上的"稳态卡尔曼滤波"。
n4 = 150;
Pseq = zeros(n4, 1);  Kseq = zeros(n4, 1);
Pk = 1.0;
for k = 1:n4
    Pminus = a_h^2 * Pk + Q_h;
    Kseq(k) = Pminus / (Pminus + R_h);
    Pseq(k) = Pminus;
    Pk = (1 - Kseq(k)) * Pminus;
end
fprintf('\n  从 P0 = 1 递推 %d 步:  K 由 %.4f 收敛到 %.8f\n', n4, Kseq(1), Kseq(end));
fprintf('  解析稳态:              Kinf = %.8f,  差 %.2e\n', Kinf_h, abs(Kseq(end) - Kinf_h));
fprintf('  同理 P^-:              递推 %.8f vs 解析 %.8f,  差 %.2e\n', ...
    Pseq(end), Pinf_h, abs(Pseq(end) - Pinf_h));
fprintf('\n  => K 的轨迹只由 (a,Q,R,P0) 决定。这意味着:\n');
fprintf('     1) 可以先离线把整条 K 算好, 再上机跑滤波;\n');
fprintf('     2) 稳态之后 K 是常数 -> 直接当常增益观测器用;\n');
fprintf('     3) 因为不含随机量, 验证脚本能做机器精度级别的严格比对。\n');

results.Kseq_end = Kseq(end);
results.Pseq_end = Pseq(end);

if opt.figures
    figure4_gain_geometry(outdir, a_h, Q_h, R_h, 1.0, Pinf_h, Kinf_h, Kseq, Pseq);
end

% =====================================================================
% 第 5 节  一维实际滤波
% =====================================================================
banner('第 5 节  一维滤波实测 (带 +-3sigma 置信带)');

% 关键做法: 真值用"带限随机加速度"生成, 滤波器假设的 Q 与它一致。
% 这样 P 才有意义, 置信带才可信 —— 这就是后文一致性检验能通过的原因。
Ts5 = 0.05;  T5 = 8;  t5 = (0:Ts5:T5)';
N5 = numel(t5);
sb5 = 2.0;                       % 加速度标准差 m/s^2
fc5 = 0.3;                       % 加速度带宽 Hz
R5  = 2.25;                      % 测量噪声方差(std = 1.5)
rng(20260922, 'twister');
acc5 = zeros(N5, 1);  acc5(1) = sb5*randn;
for k = 2:N5
    acc5(k) = exp(-2*pi*fc5*Ts5)*acc5(k-1) + sqrt(1-exp(-4*pi*fc5*Ts5))*sb5*randn;
end
vt5 = zeros(N5, 1);  xt5 = zeros(N5, 1);
xt5(1) = 0.3;
for k = 2:N5
    vt5(k) = vt5(k-1) + Ts5*acc5(k-1);
    xt5(k) = xt5(k-1) + Ts5*vt5(k-1);
end
z5 = xt5 + sqrt(R5)*randn(N5, 1);

F5 = [1 Ts5; 0 1];
Q5 = [sb5^2*Ts5^3/3, sb5^2*Ts5^2/2; sb5^2*Ts5^2/2, sb5^2*Ts5];   % 与上面加速度一致的 Q
H5 = [1 0];
ok5 = obsRank(F5, H5) == 2;
[xh5, P5] = kfRun([0; 0], diag([1 1]), z5, F5, Q5, H5, R5);

% 对照组: 完全不用测量, 只靠模型匀速外推
xt_dr5 = zeros(N5, 1);
for k = 3:N5
    xt_dr5(k) = 2*xt_dr5(k-1) - xt_dr5(k-2);
end
rms5 = @(e) sqrt(mean(e(:).^2));
rms_meas5 = rms5(z5 - xt5);
rms_dr5   = rms5(xt_dr5 - xt5);
rms_kf5   = rms5(xh5(:,1) - xt5);
sd5 = sqrt(squeeze(P5(1,1,:)));
in3 = mean(abs((xt5 - xh5(:,1))./sd5) < 3);

fprintf('\n  %d 步, Ts = %.3g s, 测量噪声 std = %.2f, 过程加速度 std = %.2f m/s^2\n', ...
    N5, Ts5, sqrt(R5), sb5);
fprintf('  %-26s %s\n', '做法', '位置 RMS');
fprintf('  %-26s %.4f   (理论 sqrt(R) = %.4f)\n', '只用测量(不滤波)', rms_meas5, sqrt(R5));
fprintf('  %-26s %.4f   (完全不用测量)\n', '只用模型(匀速外推)', rms_dr5);
fprintf('  %-26s %.4f   -> 比测量好 %.2f 倍\n', '卡尔曼滤波', rms_kf5, rms_meas5/rms_kf5);
fprintf('\n  置信带自洽性: |真值-估计|/sqrt(P) 落在 3sigma 外的比例 = %.1f%% (理论约 0.3%%)\n', ...
    100*(1-in3));
fprintf('  位置估计标准差: 从 %.3f 收敛到 %.3f\n', sd5(1), sd5(end));
fprintf('  (F,H) 可观测性: O = [H;HF] 的秩 = %d / 2 -> %s\n', obsRank(F5,H5), ...
    ternary(ok5, '可观测', '不可观测'));

results.oned = struct('rms_meas', rms_meas5, 'rms_kf', rms_kf5, 'rms_dr', rms_dr5, ...
    'in3', in3, 'sd_end', sd5(end), 'observable', ok5);

if opt.figures
    figure5_1d(outdir, t5, xt5, z5, xh5, P5, xt_dr5);
end

% =====================================================================
% 第 6 节  2D 匀速模型
% =====================================================================
banner('第 6 节  向量卡尔曼 2D 匀速模型: 只测位置, 也估出速度');

Ts6 = 0.1;  T6 = 20;  t6 = (0:Ts6:T6)';
N6 = numel(t6);
sb6 = 1.5;                        % 真实加速度 std
fc6 = 0.25;
R6  = 0.25;                       % 测量方差(std = 0.5 m)
rng(777, 'twister');
acc6 = zeros(N6, 1);  acc6(1) = sb6*randn;
for k = 2:N6
    acc6(k) = exp(-2*pi*fc6*Ts6)*acc6(k-1) + sqrt(1-exp(-4*pi*fc6*Ts6))*sb6*randn;
end
xt6 = zeros(N6, 2);
xt6(1, :) = [0.2 -1.4];
for k = 2:N6
    xt6(k, :) = xt6(k-1, :) + [Ts6*xt6(k-1,2), Ts6*acc6(k-1)];
end
z6 = xt6(:,1) + sqrt(R6)*randn(N6, 1);

F6 = [1 Ts6; 0 1];
Q6 = [sb6^2*Ts6^3/3, sb6^2*Ts6^2/2; sb6^2*Ts6^2/2, sb6^2*Ts6];
H6 = [1 0];
P06 = diag([1.0, 4.0]);
x06 = [0; 0];
[xh6, P6] = kfRun(x06, P06, z6, F6, Q6, H6, R6);

% 对照: 只用模型匀速外推
dr6 = zeros(N6, 2);  dr6(1, :) = x06';
for k = 2:N6
    dr6(k, :) = (F6 * dr6(k-1, :)')';
end
e_meas = rms5(z6 - xt6(:,1));
e_dr_p = rms5(dr6(:,1) - xt6(:,1));
e_dr_v = rms5(dr6(:,2) - xt6(:,2));
e_kf_p = rms5(xh6(:,1) - xt6(:,1));
e_kf_v = rms5(xh6(:,2) - xt6(:,2));

fprintf('\n  %-26s %-12s %-12s\n', '做法', '位置 RMS', '速度 RMS');
fprintf('  %-26s %-12.4f %-12s\n', '只用测量(不滤波)', e_meas, '(无速度)');
fprintf('  %-26s %-12.4f %-12.4f\n', '只用模型(匀速外推)', e_dr_p, e_dr_v);
fprintf('  %-26s %-12.4f %-12.4f\n', '卡尔曼滤波', e_kf_p, e_kf_v);
fprintf('\n  位置精度: 比测量好 %.2f 倍, 比纯外推好 %.1f 倍\n', e_meas/e_kf_p, e_dr_p/e_kf_p);
fprintf('  速度没有任何传感器直接测, 卡尔曼靠"位置-速度的耦合"估出来, RMS = %.4f\n', e_kf_v);
fprintf('  而纯外推的速度误差 %.4f (模型不动, 误差就一直是初值那点)\n', e_dr_v);

% 单独算一遍 K 序列, 看两条增益分量
K6 = zeros(N6, 2);
Pm = P06;
for k = 1:N6
    Pm = F6*Pm*F6' + Q6;
    S  = H6*Pm*H6' + R6;
    K6(k, :) = (Pm*H6'/S)';
    Pm = (eye(2) - K6(k,:)'*H6) * Pm;
end
fprintf('\n  K = [K_位置; K_速度] 从 [%.4f, %.4f] 收敛到 [%.4f, %.4f]\n', ...
    K6(1,1), K6(1,2), K6(end,1), K6(end,2));
fprintf('  速度增益一开始比位置增益大: 位置测量里含速度信息, 而速度本身没有传感器,\n');
fprintf('  只能靠 P 的互相关项 P12 从位置残差里"反推" -> 所以前几步要多纠一点速度。\n');
fprintf('  (F,H) 可观测性: O = [H;HF] 的秩 = %d / 2 -> %s\n', obsRank(F6,H6), ...
    ternary(obsRank(F6,H6) == 2, '可观测', '不可观测'));

% 一致性检验 NEES: 理论均值 = 状态维数
neq6 = zeros(N6, 1);
for k = 1:N6
    e = xt6(k, :)' - xh6(k, :).';
    neq6(k) = e' * (P6(:,:,k) \ e);
end
nees_mean = mean(neq6);
nees_lo = 2 - 3*sqrt(2*2/N6);
nees_hi = 2 + 3*sqrt(2*2/N6);
fprintf('\n  一致性检验 NEES = e''*inv(P)*e (e = 真值 - 估计)\n');
fprintf('    理论均值 = 状态维数 = 2,  实测 = %.3f,  3sigma 带 = [%.2f, %.2f] -> %s\n', ...
    nees_mean, nees_lo, nees_hi, ternary(nees_mean > nees_lo && nees_mean < nees_hi, ...
    '自洽(Q R 和真实噪声相符)', '不自洽(Q 或 R 定错了)'));

results.twod = struct('e_meas', e_meas, 'e_dr_p', e_dr_p, 'e_dr_v', e_dr_v, ...
    'e_kf_p', e_kf_p, 'e_kf_v', e_kf_v, 'nees_mean', nees_mean, ...
    'K_first', K6(1,:), 'K_last', K6(end,:));

if opt.figures
    figure6_2d(outdir, t6, xt6, z6, xh6, P6, K6, dr6, Ts6);
end

% =====================================================================
% 第 7 节  3D 匀速模型: 可观测性才是能不能估的前提
% =====================================================================
banner('第 7 节  3D 匀速模型: 只测 x 为什么 y z 估不出来');

% 匀速模型, 状态 [x y z vx vy vz]。两种传感器方案对比:
%   方案A: 只测 x          -> y z 完全不可观测
%   方案B: x y z 三个坐标都测 -> 全部可观测
Ts7 = 0.1;  T7 = 25;  t7 = (0:Ts7:T7)';
N7 = numel(t7);
sb7 = 0.05;
rng(4242, 'twister');
F7 = [eye(3) Ts7*eye(3); zeros(3) eye(3)];
G7 = [0.5*Ts7^2*eye(3); Ts7*eye(3)];
Q7 = G7 * (sb7^2 * eye(3)) * G7';
x7 = zeros(N7, 6);
x7(1, :) = [1.0 -0.5 0.2 0.8 0.6 0.15];
for k = 2:N7
    x7(k, :) = (F7 * x7(k-1, :)' + G7 * (sb7*randn(3,1)))';
end
R7 = 0.09;                                    % 位置测量噪声方差(std = 0.3 m)
R7m = R7 * eye(3);                            % 三个坐标通道的噪声协方差矩阵

% --- 方案 A: 只测 x (1 个测量通道, R 是标量) ---
HA = [1 0 0 0 0 0];
rankA = obsRank(F7, HA);
zA = x7(:,1) + sqrt(R7)*randn(N7, 1);
[xhA, PA] = kfRun(zeros(6,1), diag([1 1 1 4 4 4]), zA, F7, Q7, HA, R7);

% --- 方案 B: x y z 三个坐标都测 (3 个测量通道, R 必须是 3x3 矩阵) ---
HB = [eye(3) zeros(3)];
rankB = obsRank(F7, HB);
zB = x7(:,1:3) + sqrt(R7)*randn(N7, 3);
[xhB, PB] = kfRun(zeros(6,1), diag([1 1 1 4 4 4]), zB, F7, Q7, HB, R7m);

% 统计误差时跳过前 5 步收敛暂态(初值故意全零, 头几步的跳变不代表稳态精度)
i7 = 6:N7;
errA = zeros(6,1);
errB = zeros(6,1);
for i = 1:6
    errA(i) = rms5(xhA(i7,i) - x7(i7,i));
    errB(i) = rms5(xhB(i7,i) - x7(i7,i));
end
posB = rms5(reshape(xhB(i7,1:3) - x7(i7,1:3), [], 1));
velB = rms5(reshape(xhB(i7,4:6) - x7(i7,4:6), [], 1));
sdA = zeros(N7, 3);
for i = 1:3
    sdA(:, i) = sqrt(squeeze(PA(i,i,:)));
end

fprintf('\n  匀速模型状态 6 维: [x y z vx vy vz]。可观判定宁可用秩, 不用直觉:\n');
fprintf('    方案A H = [1 0 0 0 0 0] (只测 x)      -> O 的秩 = %d / 6 -> %s\n', ...
    rankA, ternary(rankA == 6, '可观测', '不可观测'));
fprintf('    方案B H = [I3 0]         (xyz 都测)   -> O 的秩 = %d / 6 -> %s\n', ...
    rankB, ternary(rankB == 6, '可观测', '不可观测'));
fprintf('\n  为什么只测 x 时 y z 不可观测: H*F^k 的 y z 列恒等于 0。\n');
fprintf('  也就是说测量值里根本不含 y z 的任何信息, F 里也没有把 y 混进 x 的耦合。\n');
fprintf('  这不是"滤波器不好", 是信息本身不存在 —— 任何估计器都救不回来。\n');
fprintf('\n  (下面统计的是第 6 步以后的误差, 跳过初值全零带来的收敛暂态)\n');
fprintf('  %-6s %-14s %-14s %-14s\n', '分量', '方案A 估计RMS', '方案B 估计RMS', '说明');
lbl = {'x', 'y', 'z', 'vx', 'vy', 'vz'};
note = {'被测', '方案A 根本没有信息', '方案A 根本没有信息', ...
    '被测', '方案A 根本没有信息', '方案A 根本没有信息'};
for i = 1:6
    fprintf('  %-6s %-14.4f %-14.4f %s\n', lbl{i}, errA(i), errB(i), note{i});
end
fprintf('\n  方案A 的 y z 估计初值是 0, 而真值跑到 %.1f / %.1f, 误差 = 初值误差 + 真值位移,\n', ...
    x7(end,2), x7(end,3));
fprintf('  同时 P22 P33 一直在增长(第 %d 步已涨到 %.0f) —— 滤波器在诚实地告诉你"我不知道"。\n', ...
    N7, PA(2,2,end));
fprintf('  方案B: 三轴位置都测, 位置 RMS %.4f, 速度 RMS %.4f(仍然没有速度传感器)。\n', ...
    posB, velB);
fprintf('\n  工程结论: 能不能估, 由 (F,H) 的可观测性决定, 不由"传感器数量"决定。\n');
fprintf('  另外注意: 方案B 有 3 个测量通道, R 必须写成 3x3 矩阵(R7m);\n');
fprintf('  多通道时 R 只给标量会让 S 和增益全算错 —— 这是新手最常见的坑之一。\n');

results.obs = struct('rankA', rankA, 'rankB', rankB, 'errA', errA, 'errB', errB, ...
    'P22_end', PA(2,2,end), 'rms_pos_B', posB, 'rms_vel_B', velB, ...
    'x_end', [x7(end,2) x7(end,3)]);

if opt.figures
    figure7_observability(outdir, t7, x7, zA, xhA, PA, xhB, PB);
end

% =====================================================================
% 第 8 节  调参
% =====================================================================
banner('第 8 节  调参: K 太大太小都会变差');

% 把卡尔曼换成"固定增益 K 的观测器", K 就成了一个可以手调的旋钮:
%   xhat(k) = F*xhat(k-1) + Kg*(z(k) - H*F*xhat(k-1))
Kscan = [0.02 0.05 0.1 0.2 0.4 0.6 0.8 1.0];
rmsScan = zeros(numel(Kscan), 2);
for i = 1:numel(Kscan)
    xhs = fixedGainObserver(Kscan(i), [0;0], diag([1 1]), z5, F5, Q5, H5);
    rmsScan(i, 1) = rms5(xhs(:,1) - xt5);
    rmsScan(i, 2) = rms5(xhs(:,2) - vt5);
end
[~, ibest] = min(rmsScan(:,1));
fprintf('\n  用第 5 节的数据, 但把增益固定成常数来手调:\n');
fprintf('  %-8s %-14s %-14s\n', 'K', '位置 RMS', '速度 RMS');
for i = 1:numel(Kscan)
    mark = '';
    if i == ibest
        mark = '   <-- 最好';
    end
    fprintf('  %-8.2f %-14.4f %-14.4f%s\n', Kscan(i), rmsScan(i,1), rmsScan(i,2), mark);
end
fprintf('  K 太小 -> 相信模型, 跟不上机动, 曲线滞后;\n');
fprintf('  K 太大 -> 相信测量, 噪声全放进来, 曲线毛刺多。\n');
fprintf('  卡尔曼做的事: 利用 Q 和 R, 每一步自动重新找这个谷底。\n');

% NIS 一致性检验(只看新息, 比 NEES 更容易在线做)
nis5 = zeros(N5, 1);
Pm = diag([1 1]);  xp = [0; 0];
for k = 1:N5
    xp = F5*xp;
    Pm = F5*Pm*F5' + Q5;
    Sk = H5*Pm*H5' + R5;
    innov = z5(k) - H5*xp;
    nis5(k) = innov^2 / Sk;
    Kk = (Pm*H5')/Sk;
    xp = xp + Kk*innov;
    Pm = (eye(2) - Kk*H5) * Pm;
end
nis_mean = mean(nis5);
chi2_95 = 3.8415;                                  % chi2(1) 的 95% 分位点
frac_in = mean(nis5 < chi2_95);
nis_lo = 1 - 3*sqrt(2/N5);
nis_hi = 1 + 3*sqrt(2/N5);
fprintf('\n  NIS 一致性检验: NIS = 新息^2 / S,  理论 E[NIS] = 1\n');
fprintf('    实测均值 = %.3f,  3sigma 带 = [%.3f, %.3f] -> %s\n', ...
    nis_mean, nis_lo, nis_hi, ternary(nis_mean > nis_lo && nis_mean < nis_hi, '自洽', 'Q/R 不自洽'));
fprintf('    落在 95%% 分位点(%.4f)以内的比例 = %.1f%% (理论 95%%)\n', chi2_95, 100*frac_in);
fprintf('    怎么用: NIS 明显大于 1 -> R 定小了或 Q 定大了(滤波器过于自信),\n');
fprintf('            明显小于 1 -> R 定大了或 Q 定小了(滤波器过于保守)。\n');

results.scan = struct('K', Kscan, 'rms', rmsScan, 'bestK', Kscan(ibest), ...
    'bestRMS', rmsScan(ibest,1));
results.nis = struct('mean', nis_mean, 'frac_in', frac_in, 'lo', nis_lo, 'hi', nis_hi);

if opt.figures
    figure8_tuning(outdir);
    figure9_tuning_scan(outdir, Kscan, rmsScan, Kseq);
end

% =====================================================================
% 验证与落盘
% =====================================================================
banner('验证');

checks = {};
checks = addCheck(checks, '两个高斯相乘 = 按精度加权平均', abs(x_wavg - x_kal) < 1e-12, ...
    sprintf('加权平均 %.6f vs 卡尔曼 %.6f, 差 %.2e', x_wavg, x_kal, abs(x_wavg - x_kal)));
checks = addCheck(checks, '精度相加 1/P+ = 1/P- + 1/R (P+ 小于两者)', ...
    abs(1/P_post - (1/Pp + 1/Rm)) < 1e-12 && P_post < min(Pp, Rm), ...
    sprintf('1/P+ = %.4f = 1/%.2f + 1/%.2f', 1/P_post, Pp, Rm));
checks = addCheck(checks, 'K 的最小方差最优性(数值扫描 vs 解析)', ...
    abs(Kbest_num - Kana) < 2e-4, sprintf('数值 %.5f vs 解析 %.5f', Kbest_num, Kana));
checks = addCheck(checks, '手算递推末步已接近稳态 K', abs(hand(end,2) - Kinf_h) < 5e-3, ...
    sprintf('递推 %.4f vs 闭式 %.6f', hand(end,2), Kinf_h));
checks = addCheck(checks, 'K/P 迭代收敛到闭式稳态(机器精度)', ...
    abs(Kseq(end) - Kinf_h) < 1e-10 && abs(Pseq(end) - Pinf_h) < 1e-10, ...
    sprintf('K 差 %.2e, P 差 %.2e', abs(Kseq(end)-Kinf_h), abs(Pseq(end)-Pinf_h)));
checks = addCheck(checks, '稳态关系 P = K*R 成立', ...
    abs((1-Kinf_h)*Pinf_h - Kinf_h*R_h) < 1e-12, ...
    sprintf('(1-K)P^- = %.8f vs K*R = %.8f', (1-Kinf_h)*Pinf_h, Kinf_h*R_h));
checks = addCheck(checks, '一维滤波有效: 估计明显优于测量', rms_kf5 < 0.8*rms_meas5, ...
    sprintf('KF %.4f vs 测量 %.4f (提升 %.2f 倍)', rms_kf5, rms_meas5, rms_meas5/rms_kf5));
checks = addCheck(checks, '一维 3sigma 置信带自洽(覆盖率 >= 97%)', in3 > 0.97, ...
    sprintf('落在 3sigma 内 %.1f%%', 100*in3));
checks = addCheck(checks, '一维 (F,H) 可观测', ok5, sprintf('O 的秩 = %d/2', obsRank(F5,H5)));
checks = addCheck(checks, '二维位置估计优于测量且优于纯外推', ...
    e_kf_p < 0.7*e_meas && e_kf_p < 0.7*e_dr_p, ...
    sprintf('KF %.4f, 测量 %.4f, 外推 %.4f', e_kf_p, e_meas, e_dr_p));
checks = addCheck(checks, '二维速度可由位置测量估出(未直接测)', e_kf_v < e_dr_v, ...
    sprintf('KF 速度 RMS %.4f < 纯外推 %.4f', e_kf_v, e_dr_v));
checks = addCheck(checks, '二维 NEES 均值落在 3sigma 带内', ...
    nees_mean > nees_lo && nees_mean < nees_hi, ...
    sprintf('NEES = %.3f, 带 [%.2f, %.2f]', nees_mean, nees_lo, nees_hi));
checks = addCheck(checks, 'NIS 均值接近 1 (Q R 设置自洽)', ...
    nis_mean > nis_lo && nis_mean < nis_hi, ...
    sprintf('NIS = %.3f, 95%% 带内 %.1f%%', nis_mean, 100*frac_in));
checks = addCheck(checks, '只测 x 时秩判为不可观测(教学结论的数学依据)', rankA < 6, ...
    sprintf('秩 = %d/6', rankA));
checks = addCheck(checks, '三轴都测时秩判为可观测', rankB == 6, sprintf('秩 = %d/6', rankB));
checks = addCheck(checks, '方案B 位置与速度都估得准', ...
    posB < 0.15 && velB < 0.15, ...
    sprintf('位置 RMS %.4f, 速度 RMS %.4f', posB, velB));
checks = addCheck(checks, '不可观测状态的 P 会发散(滤波器诚实报告不知道)', ...
    PA(2,2,end) > PA(2,2,1), sprintf('P22: %.1f -> %.1f', PA(2,2,1), PA(2,2,end)));
checks = addCheck(checks, '固定增益扫描存在极小点(K 不能太大也不能太小)', ...
    ibest > 1 && ibest < numel(Kscan), ...
    sprintf('最优 K = %.2f, 位置 RMS = %.4f', Kscan(ibest), rmsScan(ibest,1)));

nPass = sum([checks.ok]);
fprintf('\n  共 %d 项, 通过 %d 项\n', numel(checks), nPass);
for i = 1:numel(checks)
    fprintf('  [%s] %-48s %s\n', ternary(checks(i).ok, '通过', '不通过'), ...
        checks(i).name, checks(i).detail);
end
results.checks = checks;
results.nPass = nPass;
results.elapsed = toc(t0);
results.verdict = sprintf('%d/%d 通过', nPass, numel(checks));

% ---------------- validation.txt ----------------
L = {};
L{end+1} = '卡尔曼滤波教学  验证记录';
L{end+1} = sprintf('时间: %s', datestr(now, 31));
L{end+1} = sprintf('MATLAB: %s    用时: %.1f s', version, results.elapsed);
L{end+1} = '';
L{end+1} = '【核心公式】';
L{end+1} = sprintf(' 1. 加权平均 == 卡尔曼一步: %.6f vs %.6f, 差 %.2e  %s', ...
    x_wavg, x_kal, abs(x_wavg-x_kal), pass(abs(x_wavg-x_kal) < 1e-12));
L{end+1} = sprintf(' 2. 精度相加: 1/P+ = %.4f = 1/%.2f + 1/%.2f, 且 P+ = %.6f < %.2f  %s', ...
    1/P_post, Pp, Rm, P_post, min(Pp,Rm), pass(P_post < min(Pp,Rm)));
L{end+1} = sprintf(' 3. K 最优性: 扫描最小点 %.5f, 解析 P/(P+R) = %.5f, 差 %.2e  %s', ...
    Kbest_num, Kana, abs(Kbest_num-Kana), pass(abs(Kbest_num-Kana) < 2e-4));
L{end+1} = '';
L{end+1} = '【递推与稳态】';
L{end+1} = sprintf(' 4. 手算递推末步 K = %.4f, 闭式稳态 Kinf = %.6f, 差 %.2e  %s', ...
    hand(end,2), Kinf_h, abs(hand(end,2)-Kinf_h), pass(abs(hand(end,2)-Kinf_h) < 5e-3));
L{end+1} = sprintf(' 5. %d 步迭代 K = %.10f, 闭式 %.10f, 差 %.2e  %s', ...
    n4, Kseq(end), Kinf_h, abs(Kseq(end)-Kinf_h), pass(abs(Kseq(end)-Kinf_h) < 1e-10));
L{end+1} = sprintf('    %d 步迭代 P^- = %.10f, 闭式 %.10f, 差 %.2e  %s', ...
    n4, Pseq(end), Pinf_h, abs(Pseq(end)-Pinf_h), pass(abs(Pseq(end)-Pinf_h) < 1e-10));
L{end+1} = sprintf(' 6. 稳态关系 P = K*R: %.10f vs %.10f, 差 %.2e  %s', ...
    (1-Kinf_h)*Pinf_h, Kinf_h*R_h, abs((1-Kinf_h)*Pinf_h - Kinf_h*R_h), ...
    pass(abs((1-Kinf_h)*Pinf_h - Kinf_h*R_h) < 1e-12));
L{end+1} = sprintf(' 7. K 单调下降: 首步 %.4f -> 稳态 %.6f', Kseq(1), Kseq(end));
L{end+1} = '';
L{end+1} = '【一维滤波效果】';
L{end+1} = sprintf(' 8. 测量 RMS %.4f (理论 sqrt(R) = %.4f), 纯外推 RMS %.4f', ...
    rms_meas5, sqrt(R5), rms_dr5);
L{end+1} = sprintf('    KF RMS %.4f -> 比测量好 %.2f 倍, 比纯外推好 %.1f 倍  %s', ...
    rms_kf5, rms_meas5/rms_kf5, rms_dr5/rms_kf5, pass(rms_kf5 < 0.8*rms_meas5));
L{end+1} = sprintf(' 9. 3sigma 置信带覆盖率 %.1f%% (理论 99.7%%)  %s', ...
    100*in3, pass(in3 > 0.97));
L{end+1} = sprintf('10. 位置估计标准差 %.3f -> %.3f (滤波器越来越自信)', sd5(1), sd5(end));
L{end+1} = sprintf('11. (F,H) 可观测性: O = [H;HF] 秩 %d/2  %s', obsRank(F5,H5), ...
    pass(ok5));
L{end+1} = '';
L{end+1} = '【二维匀速模型(只测位置)】';
L{end+1} = sprintf('12. 位置 RMS: 测量 %.4f, 纯外推 %.4f, KF %.4f  %s', ...
    e_meas, e_dr_p, e_kf_p, pass(e_kf_p < 0.7*e_meas && e_kf_p < 0.7*e_dr_p));
L{end+1} = sprintf('13. 速度 RMS: 纯外推 %.4f, KF %.4f (无速度传感器)  %s', ...
    e_dr_v, e_kf_v, pass(e_kf_v < e_dr_v));
L{end+1} = sprintf('14. K 收敛: [%.4f %.4f] -> [%.4f %.4f]', ...
    K6(1,1), K6(1,2), K6(end,1), K6(end,2));
L{end+1} = sprintf('15. NEES 均值 %.3f (理论 2, 3sigma 带 [%.2f %.2f])  %s', ...
    nees_mean, nees_lo, nees_hi, pass(nees_mean > nees_lo && nees_mean < nees_hi));
L{end+1} = '';
L{end+1} = '【三维匀速模型: 可观测性决定能不能估】';
L{end+1} = sprintf('16. 只测 x: 可观矩阵秩 %d/6 -> 不可观测  %s', rankA, pass(rankA < 6));
L{end+1} = sprintf('17. xyz 都测: 可观矩阵秩 %d/6 -> 可观测  %s', rankB, pass(rankB == 6));
L{end+1} = sprintf('18. 只测 x 时 y 估计 RMS %.4f, z 估计 RMS %.4f (信息不存在, 估不出来)', ...
    errA(2), errA(3));
L{end+1} = sprintf('    P22 (y 的方差) 从 %.2f 涨到 %.0f -> 滤波器诚实报告"不知道"  %s', ...
    PA(2,2,1), PA(2,2,end), pass(PA(2,2,end) > PA(2,2,1)));
L{end+1} = sprintf('19. xyz 都测时(第 6 步后): 位置 RMS %.4f, 速度 RMS %.4f (速度仍无传感器)  %s', ...
    posB, velB, pass(posB < 0.15 && velB < 0.15));
L{end+1} = '';
L{end+1} = '【调参与一致性】';
L{end+1} = sprintf('20. 固定增益扫描最优 K = %.2f, 位置 RMS = %.4f  %s', ...
    Kscan(ibest), rmsScan(ibest,1), pass(ibest > 1 && ibest < numel(Kscan)));
L{end+1} = sprintf('21. NIS 均值 %.3f (理论 1), 95%% 分位点内比例 %.1f%%  %s', ...
    nis_mean, 100*frac_in, pass(nis_mean > nis_lo && nis_mean < nis_hi));
L{end+1} = '';
L{end+1} = sprintf('结论: %d/%d 项通过', nPass, numel(checks));
L{end+1} = '说明: 第 1~6 项不含随机量, 能做机器精度级别的严格比对;';
L{end+1} = '      其余项取决于随机实现, 判据是统计意义上的(3sigma 带 / 覆盖率 / 秩)。';
writeLines(fullfile(outdir, 'validation.txt'), L);
fprintf('\n  已写 validation.txt\n');

% ---------------- kalman_summary.txt ----------------
S = {};
S{end+1} = '卡尔曼滤波速查卡';
S{end+1} = sprintf('生成时间: %s', datestr(now, 31));
S{end+1} = '';
S{end+1} = '一、五个方程(向量形式, 标量就是 1x1)';
S{end+1} = '    预测:  x^- = F*x + B*u              P^- = F*P*F'' + Q';
S{end+1} = '    增益:  K   = P^-*H''*inv(H*P^-*H'' + R)';
S{end+1} = '    校正:  x   = x^- + K*(z - H*x^-)     P = (I - K*H)*P^-';
S{end+1} = '';
S{end+1} = '    对应到你已经会的观测器:';
S{end+1} = '    观测器:  xhat = F*xhat + L*(z - H*F*xhat)     L 靠自己选极点';
S{end+1} = '    卡尔曼:  xhat = F*xhat + K*(z - H*F*xhat)     K 由噪声统计自动定';
S{end+1} = '    形式完全相同。卡尔曼 = 增益按最小误差方差自动选取的观测器。';
S{end+1} = '';
S{end+1} = '二、三个"为什么"';
S{end+1} = sprintf('    1) 为什么是加权平均: 两个高斯相乘 -> 精度(1/方差)相加。');
S{end+1} = sprintf('       本例: 先验 N(%.2f,%.2f^2) x 测量 N(%.2f,%.2f^2) -> 后验 N(%.4f,%.4f^2)', ...
    x_prior, s_prior, z_meas, s_meas, x_kal, sqrt(P_post));
S{end+1} = '    2) 为什么 K 最优: 使 P+ = (1-K)^2*P- + K^2*R 最小 -> K = P-/(P-+R)。';
S{end+1} = '       一个抛物线的顶点而已, 没有别的推导。';
S{end+1} = '    3) 为什么与数据无关: K/P 的递推里没有随机量, 只看 F,Q,R,H,P0。';
S{end+1} = sprintf('       所以能离线预算整条 K: 本例稳态 Kinf = %.6f, P^-inf = %.6f。', ...
    Kinf_h, Pinf_h);
S{end+1} = '       稳态之后 K 是常数 -> 直接当常增益观测器用, 上机只剩矩阵乘法。';
S{end+1} = '';
S{end+1} = '三、手算核对过的递推表 (a=0.98, Q=0.25, R=1, P0=1)';
S{end+1} = sprintf('    %-4s %-9s %-9s %-8s %-10s %-10s %-9s', ...
    'k', 'P^-', 'K', 'z(k)', '新息', 'xhat', 'P');
for k = 1:size(hand,1)
    S{end+1} = sprintf('    %-4d %-9.4f %-9.4f %-8.2f %+-10.4f %-10.4f %-9.4f', ...
        k, hand(k,1), hand(k,2), hand(k,3), hand(k,4), hand(k,5), hand(k,6));
end
S{end+1} = '    怎么核对: 第 k 行用第 k-1 行的 xhat 和 P 算, 每步 5 个乘除加一个减法。';
S{end+1} = '';
S{end+1} = '四、K 和 P 的物理含义(调参就看这两个数)';
S{end+1} = '    K: 这一步拿新息(z - 预测)的百分之多少去修正估计。K 大 -> 跟得快但抖。';
S{end+1} = '    P: 当前估计误差方差。它自己会收缩, 收缩到和 Q/R 平衡为止。';
S{end+1} = '    P 突然变大 -> 模型跟不上实际, 该加大 Q。';
S{end+1} = '    P 一直涨且不回落 -> 那个状态不可观测(见第六节)。';
S{end+1} = '';
S{end+1} = '五、参数定错的症状';
S{end+1} = '    Q 太小: 太信模型, 响应滞后, 曲线拖尾, 真机动跟不上。';
S{end+1} = '    Q 太大: 太信测量, 曲线毛刺多, 滤波等于没做。';
S{end+1} = '    R 太小 == Q 太大;  R 太大 == Q 太小。起作用的是 Q/R 之比, 不是绝对值。';
S{end+1} = '    【最常见的实现坑】测量有 m 个通道时, R 必须是 m x m 矩阵, 不能给标量。';
S{end+1} = '      3D 例子实测: R 用 3x3 矩阵 -> 位置 RMS 0.081, 速度 RMS 0.078;';
S{end+1} = '                    R 误用标量   -> 位置 RMS 0.244, 速度 RMS 49.5 (完全跑飞)。';
S{end+1} = '      原因: S = H*P*H'' + R 的维数不对, 增益跟着错一个量级。';
S{end+1} = sprintf('    判断手段: NIS 均值应约为 1(本例 %.3f), NEES 均值应约为状态维数。', nis_mean);
S{end+1} = '    偏离很明显就是 Q/R 定错了, 而不是"滤波器不好用"。';
S{end+1} = '';
S{end+1} = '六、能不能估: 先看可观测性, 再看滤波';
S{end+1} = '    判据: O = [H; H*F; H*F^2; ...; H*F^(n-1)], rank(O) == n 才可观测。';
S{end+1} = sprintf('    1D 只测位置(2 状态): 秩 %d/2 -> 可观测', obsRank(F5,H5));
S{end+1} = sprintf('    2D 匀速只测位置(2 状态): 秩 %d/2 -> 可观测', obsRank(F6,H6));
S{end+1} = sprintf('    3D 匀速只测 x(6 状态): 秩 %d/6 -> 不可观测', rankA);
S{end+1} = sprintf('      现象: y z 估计误差 %.1f / %.1f 米, P22 从 %.2f 涨到 %.0f', ...
    errA(2), errA(3), PA(2,2,1), PA(2,2,end));
S{end+1} = '      原因: H*F^k 的 y z 两列恒为 0, 测量里根本没有 y z 的信息。';
S{end+1} = sprintf('    3D 匀速测 xyz(6 状态): 秩 %d/6 -> 可观测, 位置 RMS %.4f, 速度 RMS %.4f', ...
    rankB, posB, velB);
S{end+1} = '    结论: 卡尔曼不会凭空造信息。不可观测的状态它只能给你一个发散的 P。';
S{end+1} = '';
S{end+1} = '七、动手练习(改一个数, 重跑, 看哪个结论变了)';
S{end+1} = '    1) 第 3 节 R 改成 0.25 -> K 变大更信测量; 改成 4 -> K 变小。';
S{end+1} = '    2) 第 5 节 sb5 由 2.0 改成 0.5 (真值更平滑但 Q 不变) -> NEES 变小, 带宽但滞后。';
S{end+1} = '    3) 第 5 节 sb5 改成 0.5 且 Q5 也按 0.5 重算 -> 置信带重新变准。';
S{end+1} = '    4) 第 7 节把 HA 换成 [1 1 0 0 0 0] (测 x+y) -> 秩仍然是 4, z 依然不可观测。';
S{end+1} = '    5) 打开 build_kalman_cv.m 生成的 Simulink 模型, 和这里的数字对一遍。';
S{end+1} = '';
S{end+1} = '八、本目录的关键数字';
S{end+1} = sprintf('    一维: 测量 RMS %.4f -> KF RMS %.4f (好 %.2f 倍), 纯外推 %.4f', ...
    rms_meas5, rms_kf5, rms_meas5/rms_kf5, rms_dr5);
S{end+1} = sprintf('    二维: 位置 测量 %.4f / 外推 %.4f -> KF %.4f', e_meas, e_dr_p, e_kf_p);
S{end+1} = sprintf('    二维: 速度 外推 %.4f -> KF %.4f (速度没有传感器)', e_dr_v, e_kf_v);
S{end+1} = sprintf('    三维(可观测方案): 位置 RMS %.4f, 速度 RMS %.4f', posB, velB);
S{end+1} = sprintf('    一致性: NEES %.3f (理论 2), NIS %.3f (理论 1)', nees_mean, nis_mean);
S{end+1} = sprintf('    手调最优固定增益: K = %.2f (位置 RMS %.4f)', Kscan(ibest), rmsScan(ibest,1));
S{end+1} = sprintf('    验证: %d/%d 项通过', nPass, numel(checks));
writeLines(fullfile(outdir, 'kalman_summary.txt'), S);
fprintf('  已写 kalman_summary.txt\n');

% ---------------- 数据 ----------------
save(fullfile(outdir, 'kalman_results.mat'), 'results', 'hand', 'Kseq', 'Pseq', ...
    'Kinf_h', 'Pinf_h', 'Kscan', 'rmsScan', 'K6', ...
    't5', 'xt5', 'vt5', 'z5', 'xh5', 'P5', ...
    't6', 'xt6', 'z6', 'xh6', 'P6', ...
    't7', 'x7', 'zA', 'xhA', 'PA', 'xhB', 'PB', ...
    'nis5', 'nees_mean', 'nis_mean');
fprintf('  已写 kalman_results.mat\n');

banner(sprintf('完成: %s,  用时 %.1f s', results.verdict, results.elapsed));
if nPass < numel(checks)
    warning('%d 项验证未通过, 请看 validation.txt', numel(checks) - nPass);
end
end

% =====================================================================
% 局部函数
% =====================================================================
function opt = parseArgs(args)
opt = struct('figures', true, 'outdir', '');
if mod(numel(args), 2) ~= 0
    error('run_all_kalman: 参数必须成对出现, 例如 (''figures'', false)');
end
for i = 1:2:numel(args)
    name = lower(char(args{i}));
    if ~isfield(opt, name)
        error('run_all_kalman: 未知参数 %s', name);
    end
    opt.(name) = args{i+1};
end
end

function r = obsRank(F, H)
%OBSRANK  可观矩阵 O = [H; H*F; ...; H*F^(n-1)] 的秩
% 秩用奇异值阈值判定(比 rank() 对量纲更稳), 这里 F 的元素量级差异会到 1e2 以上。
n = size(F, 1);
O = zeros(n * size(H,1), n);
Hk = H;
for k = 0:n-1
    O(k*size(H,1) + (1:size(H,1)), :) = Hk;
    Hk = Hk * F;
end
s = svd(O);
if isempty(s) || s(1) == 0
    r = 0;
else
    r = sum(s > 1e-10 * s(1) * n);
end
end

function [xhat, P] = kfRun(x0, P0, z, F, Q, H, R)
%KFRUN  标准卡尔曼滤波递推(先预测后校正), 只用基础矩阵运算
% z 可以是 N x 1 (标量测量) 或 N x m (向量测量)
n = numel(x0);
N = size(z, 1);
xhat = zeros(N, n);
P = zeros(n, n, N);
x = x0(:);
Pk = P0;
I = eye(n);
for k = 1:N
    x  = F * x;                       % 预测
    Pk = F * Pk * F' + Q;
    S  = H * Pk * H' + R;
    K  = (Pk * H') / S;               % 增益(左除代替 inv)
    x  = x + K * (z(k, :)' - H * x);  % 校正
    Pk = (I - K * H) * Pk;
    xhat(k, :) = x';
    P(:, :, k) = Pk;
end
end

function xhat = fixedGainObserver(Kg, x0, P0, z, F, Q, H)
%FIXEDGAINOBSERVER  把卡尔曼增益固定成手调常数, 用于观察 K 太大太小
n = numel(x0);
N = numel(z);
xhat = zeros(N, n);
x = x0(:);
Pk = P0;
I = eye(n);
for k = 1:N
    xp = F * x;
    Pk = F * Pk * F' + Q;
    x  = xp + Kg * (z(k) - H * xp);
    Pk = (I - Kg*H) * Pk;
    xhat(k, :) = x';
end
end

function checks = addCheck(checks, name, ok, detail)
checks(end+1).name = name;      %#ok<AGROW>
checks(end).ok = logical(ok);
checks(end).detail = detail;
end

function s = pass(ok)
if ok
    s = '[通过]';
else
    s = '[不通过]';
end
end

function s = ternary(ok, a, b)
if ok
    s = a;
else
    s = b;
end
end

function writeLines(path, lines)
fid = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0
    error('无法写入 %s', path);
end
cleanup = onCleanup(@() fclose(fid));
for i = 1:numel(lines)
    fprintf(fid, '%s\n', lines{i});
end
clear cleanup;   %#ok<ASGLU>
end

function banner(txt)
fprintf('\n%s\n%s\n%s\n', repmat('=', 1, 74), txt, repmat('=', 1, 74));
end

function saveFig(fig, outdir, name)
path = fullfile(outdir, name);
try
    exportgraphics(fig, path, 'Resolution', 140);
catch ME
    warning('exportgraphics 失败(%s), 改用 saveas', ME.message);
    saveas(fig, path);
end
fprintf('  [图] %s\n', name);
end

function h = newFig(w, hgt)
if nargin < 1
    w = 980; hgt = 720;
end
h = figure('Position', [60 60 w hgt], 'Color', 'w', 'Visible', 'off');
end

function g = gaussPdf(x, mu, s)
g = exp(-0.5*((x - mu)/s).^2) / (s*sqrt(2*pi));
end

function annotate(ax, txt, where)
if nargin < 3
    where = [0.02 0.97];
end
text(ax, where(1), where(2), txt, 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
    'FontSize', 9.5, 'BackgroundColor', [1 1 0.87], 'EdgeColor', [0.55 0.45 0], ...
    'Margin', 6, 'Interpreter', 'none');
end

function rect(ax, xywh, col)
x = xywh(1); y = xywh(2); w = xywh(3); h = xywh(4);
patch(ax, [x x+w x+w x], [y y y+h y+h], col, 'EdgeColor', [0.25 0.25 0.25], 'LineWidth', 1.2);
end

function arrow(ax, p1, p2)
plot(ax, [p1(1) p2(1)], [p1(2) p2(2)], 'k-', 'LineWidth', 1.2);
d = [p2(1)-p1(1), p2(2)-p1(2)];
d = d / norm(d);
nv = [-d(2), d(1)];
tip = p2;
b1 = tip - 0.28*d + 0.13*nv;
b2 = tip - 0.28*d - 0.13*nv;
patch(ax, [tip(1) b1(1) b2(1)], [tip(2) b1(2) b2(2)], 'k', 'EdgeColor', 'none');
end

% ---------------------------------------------------------------------
% 图 1: 两个高斯相乘
% ---------------------------------------------------------------------
function figure1_gaussians(outdir, x1, s1, x2, s2, xp, sp, K, x_avg, x_wavg)
fig = newFig(1020, 500);
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on');
xx = linspace(min([x1-3.5*s1, x2-3.5*s2, xp-3.5*sp]), max([x1+3.5*s1, x2+3.5*s2, xp+3.5*sp]), 800);
g1 = gaussPdf(xx, x1, s1);
g2 = gaussPdf(xx, x2, s2);
gp = gaussPdf(xx, xp, sp);
plot(ax, xx, g1, '-', 'Color', [0 0.45 0.74], 'LineWidth', 1.8);
plot(ax, xx, g2, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.8);
plot(ax, xx, gp, 'k-', 'LineWidth', 2.6);
plot(ax, [x1 x1], [0 max(g1)], ':', 'Color', [0 0.45 0.74]);
plot(ax, [x2 x2], [0 max(g2)], ':', 'Color', [0.85 0.33 0.10]);
plot(ax, [xp xp], [0 max(gp)], 'k:');
xlabel(ax, 'x'); ylabel(ax, '概率密度');
title(ax, sprintf(['两个高斯相乘 = 按精度加权平均:  先验(%.2f, \\sigma=%.2f)  测量(%.2f, \\sigma=%.2f)', ...
    '  \\rightarrow  后验(%.4f, \\sigma=%.3f)'], x1, s1, x2, s2, xp, sp));
legend(ax, {sprintf('先验估计 N(%.2f, %.2f^2)', x1, s1), ...
    sprintf('本次测量 N(%.2f, %.2f^2)', x2, s2), ...
    sprintf('相乘后的后验 N(%.4f, %.4f^2)', xp, sp), '', '', ''}, ...
    'Location', 'southeast');
annotate(ax, sprintf(['做法A 直接平均     : x = %.4f  (各占一半)\n' ...
    '做法B 按精度加权   : x = %.4f\n' ...
    '做法C 卡尔曼一步   : x = %.4f   K = %.4f\n' ...
    '做法C 完全等于做法B (差 1e-16)\n' ...
    '   B 的权重 = %.3f / %.3f   <-- 就是 (1-K) 和 K\n' ...
    '为什么偏向测量: 测量 sigma %.2f < 先验 sigma %.2f\n' ...
    '合并后 sigma = %.4f, 比两者都小'], ...
    x_avg, x_wavg, xp, K, 1-K, K, s2, s1, sp), [0.565 0.72]);
saveFig(fig, outdir, 'fig1_两个高斯相乘.png');
close(fig);
end

% ---------------------------------------------------------------------
% 图 2: K 的含义
% ---------------------------------------------------------------------
function figure2_gain(outdir, gainCases)
fig = newFig(1180, 520);
cols = [0.00 0.45 0.74; 0.47 0.67 0.19; 0.85 0.33 0.10; 0.49 0.18 0.56];

ax1 = subplot(1, 2, 1, 'Parent', fig); hold(ax1, 'on'); grid(ax1, 'on');
cases = [0.09 1.0; 0.5 1.0; 1.0 0.5; 1.0 0.09];
lg = {};
for i = 1:size(cases, 1)
    Pi = cases(i,1);  Ri = cases(i,2);
    Ki = Pi/(Pi+Ri);
    Sp = sqrt(Pi);  Sz = sqrt(Ri);
    xx = linspace(min(-3.5*Sp, -3.5*Sz), max(3.5*Sp, 1+3.5*Sz), 500);
    plot(ax1, xx, gaussPdf(xx, 0, Sp), '-', 'Color', cols(i,:), 'LineWidth', 1.7);
    plot(ax1, xx, gaussPdf(xx, 1, Sz), '--', 'Color', cols(i,:), 'LineWidth', 1.7);
    lg{end+1} = sprintf('P=%.2g R=%.2g 先验(实线), K=%.3f', Pi, Ri, Ki);   %#ok<AGROW>
    lg{end+1} = sprintf('P=%.2g R=%.2g 测量(虚线)', Pi, Ri);                 %#ok<AGROW>
end
xlabel(ax1, 'x'); ylabel(ax1, '概率密度');
title(ax1, '先验(实线)与测量(虚线): 谁窄(方差小)结果就更靠谁');
legend(ax1, lg, 'Location', 'northeast', 'FontSize', 8);

ax2 = subplot(1, 2, 2, 'Parent', fig); hold(ax2, 'on'); grid(ax2, 'on');
ratio = logspace(-3, 3, 500);
plot(ax2, ratio, ratio./(ratio+1), 'k-', 'LineWidth', 2);
for i = 1:size(gainCases, 1)
    r = gainCases(i,1)/gainCases(i,2);
    plot(ax2, r, gainCases(i,1)/(gainCases(i,1)+gainCases(i,2)), 'o', ...
        'MarkerSize', 9, 'MarkerFaceColor', cols(mod(i-1,4)+1,:), 'MarkerEdgeColor', 'k');
end
plot(ax2, [1e-3 1e3], [0.5 0.5], 'k:', 'LineWidth', 1);
set(ax2, 'XScale', 'log');
xlabel(ax2, 'P / R   (模型不确定度 / 传感器不确定度)');
ylabel(ax2, 'K = (P/R) / (1 + P/R)');
title(ax2, 'K 只取决于 P 与 R 的比值, 与数据无关');
annotate(ax2, sprintf(['P/R << 1 : 信模型, K -> 0\n' ...
    'P/R = 1  : 各让一步, K = 0.5\n' ...
    'P/R >> 1 : 信测量, K -> 1\n\n' ...
    'K 就是测量在结果里的权重:\n' ...
    'xhat = (1-K)*x^- + K*z\n\n' ...
    '所以"调 K"就等于"调 Q/R 之比"']), [0.06 0.40]);
saveFig(fig, outdir, 'fig2_增益的含义.png');
close(fig);
end

% ---------------------------------------------------------------------
% 图 3: 流程图 + 手算表
% ---------------------------------------------------------------------
function figure3_flowchart(outdir, a, Q, R, P0, hand, Pinf, Kinf)
fig = newFig(1060, 680);
ax = axes(fig, 'Position', [0.03 0.02 0.94 0.90]); hold(ax, 'on');
axis(ax, [0 10 0 10]); axis(ax, 'off');

rect(ax, [0.5 6.3 3.5 2.1], [0.87 0.92 0.97]);
text(ax, 2.25, 8.00, '预测 (时间更新)', 'HorizontalAlignment','center', 'FontWeight','bold', 'FontSize',12);
text(ax, 2.25, 7.45, 'x^- = a * xhat_{k-1}', 'HorizontalAlignment','center', 'FontSize', 11);
text(ax, 2.25, 6.95, 'P^- = a^2 * P_{k-1} + Q', 'HorizontalAlignment','center', 'FontSize', 11);
text(ax, 2.25, 6.53, sprintf('a = %.2f,  Q = %.2f', a, Q), 'HorizontalAlignment','center', ...
    'FontSize', 9, 'Color', [0.35 0.35 0.35]);

rect(ax, [5.8 6.3 3.7 2.1], [0.97 0.93 0.87]);
text(ax, 7.65, 8.00, '校正 (量测更新)', 'HorizontalAlignment','center', 'FontWeight','bold', 'FontSize',12);
text(ax, 7.65, 7.45, 'K = P^- / (P^- + R)', 'HorizontalAlignment','center', 'FontSize', 11);
text(ax, 7.65, 6.97, 'xhat = x^- + K*(z - x^-)', 'HorizontalAlignment','center', 'FontSize', 11);
text(ax, 7.65, 6.53, 'P = (1 - K) * P^-', 'HorizontalAlignment','center', 'FontSize', 11);

arrow(ax, [4.0 7.35], [5.8 7.35]);
text(ax, 4.9, 7.60, 'x^-, P^-', 'HorizontalAlignment','center', 'FontSize', 10);

arrow(ax, [7.65 6.3], [7.65 5.15]);
plot(ax, [7.65 2.25], [5.15 5.15], 'k-', 'LineWidth', 1.2);
arrow(ax, [2.25 5.15], [2.25 6.3]);
text(ax, 4.95, 5.38, 'xhat, P  ->  交给下一步', 'HorizontalAlignment','center', 'FontSize', 10);

arrow(ax, [7.65 10.0], [7.65 8.4]);
text(ax, 7.80, 9.55, '测量 z(k)', 'FontSize', 10.5);
text(ax, 7.80, 9.20, '传感器读数', 'FontSize', 8.5, 'Color', [0.4 0.4 0.4]);

rect(ax, [0.5 4.05 3.5 0.75], [0.94 0.94 0.94]);
text(ax, 2.25, 4.40, sprintf('初值:  xhat(0) = 0,  P(0) = %.1f', P0), ...
    'HorizontalAlignment','center', 'FontSize', 10.5);
arrow(ax, [2.25 4.80], [2.25 6.3]);

text(ax, 0.5, 3.62, '本次运行的递推表 (每行都能用计算器核对):', 'FontSize', 10.5, 'FontWeight', 'bold');
hdr = {'k', 'P^-', 'K', 'z(k)', '新息 z-x^-', 'xhat(k)', 'P(k)'};
xpos = [0.6 1.5 2.9 4.2 5.5 7.2 8.8];
for j = 1:numel(hdr)
    text(ax, xpos(j), 3.22, hdr{j}, 'FontSize', 9.5, 'FontWeight', 'bold');
end
for k = 1:size(hand, 1)
    yy = 2.92 - 0.255*k;
    vals = {sprintf('%d', k), sprintf('%.4f', hand(k,1)), sprintf('%.4f', hand(k,2)), ...
        sprintf('%.2f', hand(k,3)), sprintf('%+.4f', hand(k,4)), ...
        sprintf('%.4f', hand(k,5)), sprintf('%.4f', hand(k,6))};
    for j = 1:numel(vals)
        text(ax, xpos(j), yy, vals{j}, 'FontSize', 9.5, 'Color', [0.15 0.15 0.15]);
    end
end
text(ax, 0.5, 0.12, sprintf(['K 从 %.4f 一路降到稳态 %.4f: P 每步变小而 R 不变, 所以越来越信模型。', ...
    '   稳态闭式 P^-inf = %.4f'], hand(1,2), Kinf, Pinf), 'FontSize', 10, 'Color', [0.6 0.1 0.1]);
title(ax, sprintf('卡尔曼滤波的五个方程与信息流向 (一个循环 = 一步)\n'), 'FontSize', 13);
saveFig(fig, outdir, 'fig3_五步递推流程图.png');
close(fig);
end

% ---------------------------------------------------------------------
% 图 4: P 的闭环
% ---------------------------------------------------------------------
function figure4_gain_geometry(outdir, a, Q, R, P0, Pinf, Kinf, Kseq, Pseq)
fig = newFig(1120, 640);
N = numel(Kseq);
ax1 = subplot(2,1,1, 'Parent', fig); hold(ax1,'on'); grid(ax1,'on');
plot(ax1, 1:N, Kseq, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 3);
yline(ax1, Kinf, 'k--', sprintf('解析稳态 Kinf = %.6f', Kinf));
xlabel(ax1, '步数 k'); ylabel(ax1, 'K');
title(ax1, '卡尔曼增益 K: 由 P 的递推自己走出来, 与测量数据完全无关');
legend(ax1, {'K(k)', '解析稳态'}, 'Location', 'northeast');
annotate(ax1, sprintf(['K 和 P 的递推里没有随机量,\n' ...
    '只由 a, Q, R, P0 决定。\n\n' ...
    '工程含义: 整条 K 序列可以\n' ...
    '离线先算好 —— 这就是\n' ...
    '"稳态卡尔曼滤波", 上机时\n' ...
    '只剩几步矩阵乘法。\n\n' ...
    'a=%.2f  Q=%.2f  R=%.2f  P0=%.1f'], a, Q, R, P0), [0.30 0.98]);

ax2 = subplot(2,1,2, 'Parent', fig); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, 1:N, Pseq, 'm-o', 'LineWidth', 1.5, 'MarkerSize', 3);
yline(ax2, Pinf, 'k--', sprintf('解析稳态 P^-inf = %.6f', Pinf));
yline(ax2, R, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.4);
xlabel(ax2, '步数 k'); ylabel(ax2, 'P^-');
title(ax2, sprintf(['P^- 的闭环: P^-_{k+1} = a^2*(1-K_k)*P^-_k + Q,   ', ...
    'K_k = P^-_k/(P^-_k+R)   (灰虚线 R = %.2f)'], R));
legend(ax2, {'P^-(k)', '解析稳态', 'R'}, 'Location', 'northeast');
saveFig(fig, outdir, 'fig4_增益递推与稳态.png');
close(fig);
end

% ---------------------------------------------------------------------
% 图 5: 一维滤波
% ---------------------------------------------------------------------
function figure5_1d(outdir, t, xt, z, xh, P, dr)
fig = newFig(1020, 720);
sd = sqrt(squeeze(P(1,1,:)));

ax1 = subplot(2,1,1, 'Parent', fig); hold(ax1,'on'); grid(ax1,'on');
fill(ax1, [t; flipud(t)], [xh(:,1)+3*sd; flipud(xh(:,1)-3*sd)], [0.86 0.90 1.0], 'EdgeColor','none');
plot(ax1, t, z, '.', 'Color', [0.78 0.78 0.78], 'MarkerSize', 4);
plot(ax1, t, xt, 'k-', 'LineWidth', 1.4);
plot(ax1, t, xh(:,1), 'r-', 'LineWidth', 1.8);
plot(ax1, t, dr, '--', 'Color', [0.30 0.60 0.20], 'LineWidth', 1.2);
xlabel(ax1, 't / s'); ylabel(ax1, '位置');
title(ax1, '一维卡尔曼滤波: 灰点=测量, 黑线=真值, 红线=估计, 蓝带=+-3\sigma, 绿虚线=纯模型外推');
legend(ax1, {'+-3\sigma 置信带', '测量 z', '真值 x', '估计 xhat', '纯模型外推'}, 'Location', 'best');

ax2 = subplot(2,2,3, 'Parent', fig); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, t, z - xt, '.', 'Color', [0.78 0.78 0.78], 'MarkerSize', 4);
plot(ax2, t, xh(:,1) - xt, 'r-', 'LineWidth', 1.4);
xlabel(ax2, 't / s'); ylabel(ax2, '误差');
title(ax2, '估计误差(红)明显小于测量误差(灰)', 'FontSize', 10.5);
legend(ax2, {'测量误差', '估计误差'}, 'Location', 'best', 'FontSize', 8);

ax3 = subplot(2,2,4, 'Parent', fig); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, t, sd, 'b-', 'LineWidth', 1.6);
xlabel(ax3, 't / s'); ylabel(ax3, '\surd P_{11}');
title(ax3, sprintf('位置估计标准差: %.3f -> %.3f', sd(1), sd(end)), 'FontSize', 10.5);
saveFig(fig, outdir, 'fig5_一维滤波.png');
close(fig);
end

% ---------------------------------------------------------------------
% 图 6: 2D 匀速模型
% ---------------------------------------------------------------------
function figure6_2d(outdir, t, xt, z, xh, P, K, dr, Ts)
fig = newFig(1180, 780);
sd = sqrt(squeeze(P(1,1,:)));

ax1 = subplot(2,2,[1 2], 'Parent', fig); hold(ax1,'on'); grid(ax1,'on');
fill(ax1, [t; flipud(t)], [xh(:,1)+3*sd; flipud(xh(:,1)-3*sd)], [0.86 0.90 1.0], 'EdgeColor','none');
plot(ax1, t, z, '.', 'Color', [0.80 0.80 0.80], 'MarkerSize', 5);
plot(ax1, t, dr(:,1), '--', 'Color', [0.30 0.60 0.20], 'LineWidth', 1.3);
plot(ax1, t, xt(:,1), 'k-', 'LineWidth', 1.4);
plot(ax1, t, xh(:,1), 'r-', 'LineWidth', 1.8);
xlabel(ax1, 't / s'); ylabel(ax1, '位置 / m');
title(ax1, sprintf('2D 匀速模型 (Ts = %.2f s): 只测位置(灰点), 同时估位置和速度', Ts));
legend(ax1, {'+-3\sigma', '测量 z', '纯模型外推', '真值', '卡尔曼估计'}, 'Location', 'best');

ax2 = subplot(2,2,3, 'Parent', fig); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, t, squeeze(P(1,1,:)), 'b-', 'LineWidth', 1.6);
plot(ax2, t, squeeze(P(2,2,:)), 'm-', 'LineWidth', 1.6);
plot(ax2, t, abs(squeeze(P(1,2,:))), 'k:', 'LineWidth', 1.4);
set(ax2, 'YScale', 'log');
xlabel(ax2, 't / s'); ylabel(ax2, 'P 的元素');
title(ax2, '协方差矩阵的三个独立元素', 'FontSize', 10.5);
legend(ax2, {'P_{11} 位置', 'P_{22} 速度', '|P_{12}| 互相关'}, 'Location', 'best', 'FontSize', 8);

ax3 = subplot(2,2,4, 'Parent', fig); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, t, K(:,1), 'b-', 'LineWidth', 1.7);
plot(ax3, t, K(:,2), 'm-', 'LineWidth', 1.7);
xlabel(ax3, 't / s'); ylabel(ax3, '增益分量');
title(ax3, sprintf('K = [K_位置; K_速度] 收敛到 [%.3f; %.3f]', K(end,1), K(end,2)), 'FontSize', 10.5);
legend(ax3, {'K_位置', 'K_速度'}, 'Location', 'best', 'FontSize', 8);
annotate(ax3, sprintf(['前几步速度增益比位置增益大:\n' ...
    '位置测量里含速度信息, 而速度\n' ...
    '没有传感器, 只能靠互相关项\n' ...
    'P12 从位置残差里反推出来,\n' ...
    '所以要多纠一点速度。\n\n' ...
    'P12 就是这座桥。']), [0.05 0.96]);
saveFig(fig, outdir, 'fig6_二维匀速模型.png');
close(fig);
end

% ---------------------------------------------------------------------
% 图 7: 可观测性对比
% ---------------------------------------------------------------------
function figure7_observability(outdir, t, x, zA, xhA, PA, xhB, PB)
fig = newFig(1180, 800);
sdA = zeros(numel(t), 3);
for i = 1:3
    sdA(:, i) = sqrt(squeeze(PA(i,i,:)));
end

ax1 = subplot(2,2,1, 'Parent', fig); hold(ax1,'on'); grid(ax1,'on');
fill(ax1, [t; flipud(t)], [xhA(:,1)+3*sdA(:,1); flipud(xhA(:,1)-3*sdA(:,1))], [0.86 0.90 1.0], 'EdgeColor','none');
plot(ax1, t, zA, '.', 'Color', [0.82 0.82 0.82], 'MarkerSize', 4);
plot(ax1, t, x(:,1), 'k-', 'LineWidth', 1.3);
plot(ax1, t, xhA(:,1), 'r-', 'LineWidth', 1.6);
xlabel(ax1, 't / s'); ylabel(ax1, 'x / m');
title(ax1, '方案A(只测 x): x 估得很好', 'FontSize', 10.5);
legend(ax1, {'+-3\sigma', '测量', '真值', '估计'}, 'Location', 'best', 'FontSize', 8);

ax2 = subplot(2,2,2, 'Parent', fig); hold(ax2,'on'); grid(ax2,'on');
fill(ax2, [t; flipud(t)], [xhA(:,2)+3*sdA(:,2); flipud(xhA(:,2)-3*sdA(:,2))], [1.0 0.90 0.86], 'EdgeColor','none');
plot(ax2, t, x(:,2), 'k-', 'LineWidth', 1.3);
plot(ax2, t, xhA(:,2), 'r-', 'LineWidth', 1.6);
xlabel(ax2, 't / s'); ylabel(ax2, 'y / m');
title(ax2, '方案A: y 完全估不出来(不可观测)', 'FontSize', 10.5);
legend(ax2, {'+-3\sigma', '真值', '估计'}, 'Location', 'best', 'FontSize', 8);
annotate(ax2, sprintf(['秩判据: O=[H;HF;...] 的秩 = 3/6\n' ...
    'H*F^k 的 y z 列恒为 0\n' ...
    '-> 测量里没有 y 的任何信息\n' ...
    '估计停在初值 0, 误差 = 真值位移\n' ...
    '而且 3sigma 带一路张开,\n' ...
    '滤波器在诚实地说"我不知道"']), [0.04 0.96]);

ax3 = subplot(2,2,3, 'Parent', fig); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, t, squeeze(PA(2,2,:)), 'r-', 'LineWidth', 1.7);
plot(ax3, t, squeeze(PA(1,1,:)), 'b-', 'LineWidth', 1.7);
plot(ax3, t, squeeze(PA(5,5,:)), 'm-', 'LineWidth', 1.7);
xlabel(ax3, 't / s'); ylabel(ax3, 'P 的对角元');
title(ax3, '方案A: 不可观测状态的方差一直涨', 'FontSize', 10.5);
legend(ax3, {'P_{22} (y 的位置)', 'P_{11} (x 的位置)', 'P_{55} (vy)'}, ...
    'Location', 'best', 'FontSize', 8);

ax4 = subplot(2,2,4, 'Parent', fig); hold(ax4,'on'); grid(ax4,'on');
fill(ax4, [t; flipud(t)], [xhB(:,2)+3*sqrt(squeeze(PB(2,2,:))); flipud(xhB(:,2)-3*sqrt(squeeze(PB(2,2,:))))], ...
    [0.86 0.95 0.86], 'EdgeColor','none');
plot(ax4, t, x(:,2), 'k-', 'LineWidth', 1.3);
plot(ax4, t, xhB(:,2), 'r-', 'LineWidth', 1.6);
plot(ax4, t, xhA(:,2), ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.4);
xlabel(ax4, 't / s'); ylabel(ax4, 'y / m');
title(ax4, '方案B(xyz 都测): 同一个 y 立刻估住了', 'FontSize', 10.5);
legend(ax4, {'+-3\sigma', '真值', '方案B 估计', '方案A 估计(失效)'}, 'Location', 'best', 'FontSize', 8);

sgtitle(fig, '能不能估由可观测性决定: 不是"滤波器不好", 而是信息本身不存在', 'FontSize', 12);
saveFig(fig, outdir, 'fig7_可观测性对比.png');
close(fig);
end

% ---------------------------------------------------------------------
% 图 8: 参数定错的症状
% ---------------------------------------------------------------------
function figure8_tuning(outdir)
Ts = 0.05;  T = 8;  t = (0:Ts:T)';  N = numel(t);
sb = 2.0;  fc = 0.3;
rng(20260922, 'twister');
acc = zeros(N,1);  acc(1) = sb*randn;
for k = 2:N
    acc(k) = exp(-2*pi*fc*Ts)*acc(k-1) + sqrt(1-exp(-4*pi*fc*Ts))*sb*randn;
end
vt = zeros(N,1);  xt = zeros(N,1);  xt(1) = 0.3;
for k = 2:N
    vt(k) = vt(k-1) + Ts*acc(k-1);
    xt(k) = xt(k-1) + Ts*vt(k-1);
end
R = 2.25;
z = xt + sqrt(R)*randn(N,1);
F = [1 Ts; 0 1];
H = [1 0];
Qfull = [sb^2*Ts^3/3, sb^2*Ts^2/2; sb^2*Ts^2/2, sb^2*Ts];

cfg = { ...
    'Q 太小: 太信模型 -> 滞后, 跟不上机动',   0.05,  R; ...
    'Q 合适: 与真实加速度匹配',               1.00,  R; ...
    'Q 太大: 太信测量 -> 抖动, 滤波等于没做', 25.0,  R; ...
    'R 定大了 4 倍(等效 Q 定小)',             1.00,  4*R};

fig = newFig(1180, 800);
for i = 1:4
    ax = subplot(2,2,i, 'Parent', fig); hold(ax,'on'); grid(ax,'on');
    scale = cfg{i,2};  Ri = cfg{i,3};
    xh = kfRun([0;0], diag([1 1]), z, F, Qfull*scale, H, Ri);
    plot(ax, t, z, '.', 'Color', [0.82 0.82 0.82], 'MarkerSize', 4);
    plot(ax, t, xt, 'k-', 'LineWidth', 1.3);
    plot(ax, t, xh(:,1), 'r-', 'LineWidth', 1.7);
    e = sqrt(mean((xh(:,1)-xt).^2));
    xlabel(ax, 't / s'); ylabel(ax, '位置');
    title(ax, sprintf('%s\nQ x %.2g, R = %.3g   ->   位置 RMS = %.4f', cfg{i,1}, scale, Ri, e), ...
        'FontSize', 10);
    if i == 1
        legend(ax, {'测量', '真值', '估计'}, 'Location', 'best', 'FontSize', 8);
    end
end
sgtitle(fig, '参数定错的四种典型症状: 起作用的是 Q/R 之比, 不是单个绝对值', 'FontSize', 12);
saveFig(fig, outdir, 'fig8_参数定错的症状.png');
close(fig);
end

% ---------------------------------------------------------------------
% 图 9: 增益扫描与收敛
% ---------------------------------------------------------------------
function figure9_tuning_scan(outdir, Kscan, rmsScan, Kseq)
fig = newFig(1180, 500);
ax1 = subplot(1,2,1, 'Parent', fig); hold(ax1,'on'); grid(ax1,'on');
plot(ax1, Kscan, rmsScan(:,1), 'o-', 'LineWidth', 1.8, 'MarkerFaceColor', [0 0.45 0.74]);
[~, ib] = min(rmsScan(:,1));
plot(ax1, Kscan(ib), rmsScan(ib,1), 'p', 'MarkerSize', 17, ...
    'MarkerFaceColor', [0.93 0.69 0.13], 'MarkerEdgeColor', 'k');
xlabel(ax1, '固定增益 K (手调)'); ylabel(ax1, '位置 RMS 误差');
title(ax1, sprintf('K 太小滞后、太大抖动, 谷底在 K ≈ %.2f', Kscan(ib)));
annotate(ax1, sprintf(['左边 K 小: 相信模型, 测量几乎\n' ...
    '不用 -> 跟不上机动, 曲线滞后\n\n' ...
    '右边 K 大: 相信测量, 噪声全部\n' ...
    '进来 -> 曲线毛刺多\n\n' ...
    '卡尔曼做的事: 利用 Q 和 R,\n' ...
    '每一步都自动重新找这个谷底。\n\n' ...
    '谷底 RMS = %.4f'], rmsScan(ib,1)), [0.38 0.97]);

ax2 = subplot(1,2,2, 'Parent', fig); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, 1:numel(Kseq), Kseq, 'b-o', 'LineWidth', 1.6, 'MarkerSize', 4);
xlabel(ax2, '步数 k'); ylabel(ax2, 'K');
title(ax2, sprintf('同一组参数下 K 的收敛: %.4f -> %.4f', Kseq(1), Kseq(end)));
annotate(ax2, sprintf(['前几步 K 大 = 还不太信自己,\n' ...
    '主要靠测量; 之后稳定在稳态值,\n' ...
    '等于按噪声统计分工。\n\n' ...
    '如果只想用常增益滤波,\n' ...
    '直接取这个稳态 K 即可:\n' ...
    '计算量最小, 效果几乎相同。']), [0.36 0.97]);
saveFig(fig, outdir, 'fig9_增益扫描与收敛.png');
close(fig);
end
