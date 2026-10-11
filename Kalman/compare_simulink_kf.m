function compare_simulink_kf()
%COMPARE_SIMULINK_KF  把 Simulink 模型和 MATLAB 脚本的结果逐点对照
%
% 用法: 在本目录执行 compare_simulink_kf
%   需要 MATLAB + Simulink, 不需要任何工具箱。
%
% 对照思路(这是本例最漂亮的一点):
%   K 和 P 的递推里**不含随机量**, 只由 (F,Q,H,R,P0) 决定。
%   所以只要两边参数一样, K 序列必须逐点相等 —— 可以做到机器精度级别的比对,
%   而不是"看起来差不多"。测量噪声由 Simulink 的随机数模块产生, 与 MATLAB
%   的随机流无关; 但对照时直接把模型输出的 z 喂给 MATLAB 版递推, 于是
%   连状态估计序列也必须完全一致。
%
% 输出:
%   validation_simulink.txt  对照记录
%   model_kalman_simple.png / model_kalman_cv.png  模型结构快照
%   compare_simulink.png     一维模型的三联图

mdl1 = 'kalman_simple';
mdl2 = 'kalman_cv';
folder = fileparts(mfilename('fullpath'));
if isempty(folder), folder = pwd; end
old = pwd;
cleanup = onCleanup(@() cd(old));
cd(folder);

fprintf('\n%s\n', repmat('=', 1, 74));
fprintf('Simulink 模型 与 MATLAB 脚本 对照\n');
fprintf('%s\n', repmat('=', 1, 74));

lines = {};
lines{end+1} = 'Simulink 模型 与 MATLAB 脚本 对照记录';
lines{end+1} = sprintf('时间: %s', datestr(now, 31));
lines{end+1} = sprintf('MATLAB: %s', version);
lines{end+1} = '';

% =====================================================================
% 一维模型
% =====================================================================
if ~exist([mdl1 '.slx'], 'file')
    build_kalman_simple();
end
if ~bdIsLoaded(mdl1)
    load_system(mdl1);
end
out1 = sim(mdl1);
t1 = out1.kf1_x_true.Time;
xt1 = out1.kf1_x_true.Data(:);
z1 = out1.kf1_y_meas.Data(:);
xh1 = out1.kf1_xhat.Data(:);
K1 = out1.kf1_K.Data(:);
P1 = out1.kf1_Pminus.Data(:);
N1 = numel(t1);

a = 0.98; Q = 0.25; R = 1.0;
fa = a^2;
bb = R - fa*R - Q;
Pinf = (-bb + sqrt(bb^2 + 4*Q*R)) / 2;
Kinf = Pinf / (Pinf + R);

% MATLAB 重算(用模型输出的 z)
x = 0; Pp = 1.0;                 % xhat(0)=0, P(0)=1
Kref = zeros(N1,1); Pref = zeros(N1,1); xref = zeros(N1,1);
for k = 1:N1
    xm = a*x;
    Pm = fa*Pp + Q;
    Kk = Pm / (Pm + R);
    x  = xm + Kk*(z1(k) - xm);
    Pp = (1 - Kk)*Pm;
    Kref(k) = Kk; Pref(k) = Pm; xref(k) = x;
end
dK1 = max(abs(K1 - Kref));
dP1 = max(abs(P1 - Pref));
dx1 = max(abs(xh1 - xref));

fprintf('\n[一维 kalman_simple.slx]  %d 步\n', N1);
fprintf('  K  序列最大差 = %.3e  %s\n', dK1, pass(dK1 < 1e-10));
fprintf('  P  序列最大差 = %.3e  %s\n', dP1, pass(dP1 < 1e-10));
fprintf('  估计序列最大差 = %.3e  %s\n', dx1, pass(dx1 < 1e-10));
fprintf('  末步 K = %.6f, 解析稳态 = %.6f, 差 %.2e  %s\n', ...
    K1(end), Kinf, abs(K1(end)-Kinf), pass(abs(K1(end)-Kinf) < 1e-4));
rms1m = sqrt(mean((z1 - xt1).^2));
rms1k = sqrt(mean((xh1 - xt1).^2));
fprintf('  滤波效果: 测量 RMS %.4f -> 估计 RMS %.4f (好 %.2f 倍)\n', ...
    rms1m, rms1k, rms1m/rms1k);

lines{end+1} = '【一维模型 kalman_simple.slx】';
lines{end+1} = sprintf('  步数 %d, 参数 a=%.2f Q=%.2f R=%.2f P0=%.1f', N1, a, Q, R, 1.0);
lines{end+1} = sprintf('  1) K 序列最大差 = %.3e  %s', dK1, pass(dK1 < 1e-10));
lines{end+1} = sprintf('  2) P 序列最大差 = %.3e  %s', dP1, pass(dP1 < 1e-10));
lines{end+1} = sprintf('  3) 估计序列最大差 = %.3e (同一组 z)  %s', dx1, pass(dx1 < 1e-10));
lines{end+1} = sprintf('  4) 末步 K = %.6f, 解析稳态 Kinf = %.6f, 差 %.2e  %s', ...
    K1(end), Kinf, abs(K1(end)-Kinf), pass(abs(K1(end)-Kinf) < 1e-4));
lines{end+1} = sprintf('  5) 滤波效果: 测量 RMS %.4f -> 估计 RMS %.4f (好 %.2f 倍)', ...
    rms1m, rms1k, rms1m/rms1k);
lines{end+1} = '';

% 一维三联图
fig = figure('Position', [60 60 1000 700], 'Color', 'w', 'Visible', 'off');
subplot(3,1,1); plot(t1, z1, '.', 'Color', [0.78 0.78 0.78], 'MarkerSize', 4); hold on; grid on;
plot(t1, xt1, 'k-', 'LineWidth', 1.3); plot(t1, xh1, 'r-', 'LineWidth', 1.6);
xlabel('t / s'); ylabel('x'); title('kalman\_simple.slx: 灰=测量, 黑=真值, 红=估计');
legend('测量', '真值', '估计', 'Location', 'best');
subplot(3,1,2); plot(t1, K1, 'b-', 'LineWidth', 1.5); hold on; grid on;
yline(Kinf, 'k--', sprintf('解析稳态 K = %.4f', Kinf));
xlabel('t / s'); ylabel('K'); title('卡尔曼增益: Simulink 与手算递推逐点一致');
legend('K(k)', '解析稳态', 'Location', 'best');
subplot(3,1,3); plot(t1, P1, 'm-', 'LineWidth', 1.5); hold on; grid on;
yline(Pinf, 'k--', sprintf('解析稳态 P^- = %.4f', Pinf));
xlabel('t / s'); ylabel('P^-'); title('预测误差方差 P^-');
legend('P^-(k)', '解析稳态', 'Location', 'best');
exportgraphics(fig, fullfile(folder, 'compare_simulink.png'), 'Resolution', 140);
close(fig);

% =====================================================================
% 二维模型
% =====================================================================
if ~exist([mdl2 '.slx'], 'file')
    build_kalman_cv();
end
if ~bdIsLoaded(mdl2)
    load_system(mdl2);
end
out2 = sim(mdl2);
xt2 = reshape(out2.kf2_x_true.Data, [], 2);        % 真值 [N 2] = [位置 速度]
z2 = out2.kf2_z_meas.Data(:);
xh2 = permute(out2.kf2_xhat.Data, [3 1 2]);
Ks = permute(out2.kf2_Kvec.Data, [3 1 2]);
Ps = permute(out2.kf2_Pmat.Data, [3 1 2]);
N2 = size(xh2, 1);

Ts = 0.1; F = [1 Ts; 0 1]; H = [1 0]; R2 = 0.25;
Q2 = diag([0.25*Ts^4, Ts^2]) * 0.08^2;
x = [0; 0]; Ppost = [1 0; 0 4]; I2 = eye(2);
Kref2 = zeros(N2,2); Pref2 = zeros(N2,2,2); xref2 = zeros(N2,2);
for k = 1:N2
    xp = F*x;
    Ppred = F*Ppost*F' + Q2;
    S = H*Ppred*H' + R2;
    K = (Ppred*H')/S;
    x = xp + K*(z2(k) - H*xp);
    Ppost = (I2 - K*H)*Ppred;
    Kref2(k,:) = K'; Pref2(k,:,:) = Ppost; xref2(k,:) = x';
end
dK2 = max(max(abs(Ks - Kref2)));
dP2 = max(max(max(abs(Ps - Pref2))));
dx2 = max(max(abs(xh2 - xref2)));
rmsPos = sqrt(mean((z2 - xt2(:,1)).^2));
rmsPosK = sqrt(mean((xh2(:,1) - xt2(:,1)).^2));
rmsVelK = sqrt(mean((xh2(:,2) - xt2(:,2)).^2));

fprintf('\n[二维 kalman_cv.slx]  %d 步\n', N2);
fprintf('  K   序列最大差 = %.3e  %s\n', dK2, pass(dK2 < 1e-10));
fprintf('  P   序列最大差 = %.3e  %s\n', dP2, pass(dP2 < 1e-10));
fprintf('  估计序列最大差 = %.3e  %s\n', dx2, pass(dx2 < 1e-10));
fprintf('  K 从 [%.4f %.4f] 收敛到 [%.5f %.5f]\n', ...
    Ks(1,1), Ks(1,2), Ks(end,1), Ks(end,2));
fprintf('  位置: 测量 RMS %.4f -> 估计 RMS %.4f (好 %.2f 倍)\n', ...
    rmsPos, rmsPosK, rmsPos/rmsPosK);
fprintf('  速度: 估计 RMS %.4f (没有速度传感器)\n', rmsVelK);

lines{end+1} = '【二维模型 kalman_cv.slx】';
lines{end+1} = sprintf('  步数 %d, Ts=%.2g, Q=diag([%.3g %.3g]), R=%.3g, P0=[1 0;0 4]', ...
    N2, Ts, Q2(1,1), Q2(2,2), R2);
lines{end+1} = sprintf('  6) K 序列最大差 = %.3e  %s', dK2, pass(dK2 < 1e-10));
lines{end+1} = sprintf('  7) P 序列最大差 = %.3e  %s', dP2, pass(dP2 < 1e-10));
lines{end+1} = sprintf('  8) 估计序列最大差 = %.3e (同一组 z)  %s', dx2, pass(dx2 < 1e-10));
lines{end+1} = sprintf('  9) K 收敛: [%.4f %.4f] -> [%.5f %.5f]', ...
    Ks(1,1), Ks(1,2), Ks(end,1), Ks(end,2));
lines{end+1} = sprintf(' 10) 位置: 测量 RMS %.4f -> 估计 RMS %.4f (好 %.2f 倍)', ...
    rmsPos, rmsPosK, rmsPos/rmsPosK);
lines{end+1} = sprintf(' 11) 速度估计 RMS %.4f (无速度传感器, 靠 P12 互相关估出)', rmsVelK);
lines{end+1} = '';

allPass = all([dK1 dP1 dx1 dK2 dP2 dx2] < 1e-10) && ...
    abs(K1(end)-Kinf) < 1e-4;
if allPass
    lines{end+1} = '结论: 两个 Simulink 模型与 MATLAB 脚本的递推完全等价(差为 0 量级),';
    lines{end+1} = '      并且一维模型的稳态增益与解析解吻合。';
else
    lines{end+1} = '结论: 存在不通过的对照项, 请检查模型参数是否被改动。';
end

fid = fopen(fullfile(folder, 'validation_simulink.txt'), 'w', 'n', 'UTF-8');
for i = 1:numel(lines)
    fprintf(fid, '%s\n', lines{i});
end
fclose(fid);
fprintf('\n已写 validation_simulink.txt, compare_simulink.png\n');

% 结构快照(按需要额外安装的版本可能不同, 失败不影响对照结论)
try
    print(['-s' mdl1], '-dpng', '-r110', fullfile(folder, 'model_kalman_simple.png'));
    print(['-s' mdl2], '-dpng', '-r110', fullfile(folder, 'model_kalman_cv.png'));
    fprintf('已写 model_kalman_simple.png / model_kalman_cv.png\n');
catch ME
    fprintf('模型快照失败(不影响对照): %s\n', ME.message);
end

fprintf('\n%s\n', repmat('=', 1, 74));
if allPass
    fprintf('对照结论: 全部通过\n');
else
    fprintf('对照结论: 有项目未通过\n');
end
fprintf('%s\n', repmat('=', 1, 74));
end

function s = pass(ok)
if ok
    s = '[通过]';
else
    s = '[不通过]';
end
end
