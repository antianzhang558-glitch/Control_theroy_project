clear; clc;

%% 定义被控对象
s = tf('s');
G = 2 / ((s+2)*(s^2 + 1*s + 1));

%% 设计指标
Kp_req = 10;
PM_req = 45;

%% 计算所需增益
Kp_curr = dcgain(G);
Kc = Kp_req / Kp_curr;
G_gain = Kc * G;

%% 分析增益调整后的相位裕度
S0 = allmargin(G_gain);
if isfield(S0, 'PhaseMargin') && ~isempty(S0.PhaseMargin)
    PM0 = S0.PhaseMargin;
else
    PM0 = 0;
end

%% 自动超前校正设计
if PM0 < PM_req
    phi_m = PM_req - PM0 + 8;
    phi_m = max(phi_m, 5);
    alpha = (1 + sin(deg2rad(phi_m))) / (1 - sin(deg2rad(phi_m)));
    
    [mag, ~, w] = bode(G_gain);
    mag_dB = 20*log10(squeeze(mag));
    target_dB = -10*log10(alpha);
    [~, idx] = min(abs(mag_dB - target_dB));
    wm = w(idx);
    T = 1 / (wm * sqrt(alpha));
    
    G_lead = (alpha*T*s + 1) / (T*s + 1);
else
    G_lead = 1;
end

G_lag = 1;          % 若有滞后需求再扩展
G_c = G_lead * G_lag * Kc;   % <---- 这就是校正环节

%% 输出校正环节 C(s)
fprintf('\n===== 校正环节 C(s) =====\n');
G_c

[num, den] = tfdata(G_c, 'v');
fprintf('分子：'); disp(num);
fprintf('分母：'); disp(den);