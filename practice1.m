%% 完整超前滞后校正设计 + Simulink 部署
clear; clc;

% 1. 对象定义
s = tf('s');
G = 16/(s^2 + 1.6*s + 16);

% 2. 设计指标
Kp_req = 10;
PM_req = 50;

% 3. 增益调整
Kc = Kp_req / dcgain(G);
G_gain = Kc * G;

% 4. 超前部分设计（目标 55° 补偿）
phi_m_lead = 55;
alpha = (1+sin(deg2rad(phi_m_lead))) / (1-sin(deg2rad(phi_m_lead)));
[mag, ~, w] = bode(G_gain);
mag_dB = 20*log10(squeeze(mag));
[~, idx] = min(abs(mag_dB + 10*log10(alpha)));
wm = w(idx); T_lead = 1/(wm*sqrt(alpha));
G_lead = (alpha*T_lead*s + 1)/(T_lead*s + 1);

% 5. 滞后部分设计
G_open_lead = G_lead * G_gain;
[mag2, ~, w2] = bode(G_open_lead);
mag2_dB = 20*log10(squeeze(mag2));
[~, phase2] = bode(G_open_lead, w2);
phase2 = squeeze(phase2);
idx_wc = find(phase2 <= -130, 1);
wc_new = w2(idx_wc);
L_dB = interp1(w2, mag2_dB, wc_new);
beta = 10^(L_dB/20);
T_lag = 10/wc_new;
G_lag = (T_lag*s + 1)/(beta*T_lag*s + 1);

% 6. 总控制器与闭环
G_c = G_lead * G_lag * Kc;
G_open = G_c * G;
G_closed = feedback(G_open, 1);

% 7. 验证
S_f = allmargin(G_open);
fprintf('相位裕度: %.1f°\n', S_f.PhaseMargin);
fprintf('增益裕度: %.1f dB\n', 20*log10(S_f.GainMargin));
stepinfo(G_closed)
step(G_closed); grid on; title('校正后阶跃响应');

% 8. 导出到 Simulink（将 G_c 和 G 写入工作区即可）
% 在 Simulink 中直接使用这两个变量