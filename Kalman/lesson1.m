function lesson1()
%LESSON1  第一讲: 卡尔曼滤波到底在干什么
%
% 用法: 在 Kalman 目录下执行  lesson1
% 想看真正的滤波器长什么样: 打开 kalman_simple.slx 点运行
% 想连着向量形式一起看:     run_all_kalman  (出 9 张图 + 验证记录)
%
% 这一讲只做一件事: 用"两个高斯相乘"把卡尔曼滤波的全部直觉说清楚。

clc;
fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('第一讲  卡尔曼滤波到底在干什么\n');
fprintf('%s\n', repmat('=', 1, 70));

% ---------- 场景 ----------
% 一个传感器在测一个不动的量(真值 2.00), 你已经有一份"上一步的估计"。
x_prior = 1.70;  s_prior = 0.50;      % 先验估计与其标准差
z_meas  = 2.10;  s_meas  = 0.20;      % 本次测量与其标准差
Pp = s_prior^2;  Rm = s_meas^2;       % 对应到卡尔曼术语: P(模型不确定) 与 R(传感器不准)
truth = 2.00;

fprintf('\n场景: 真值 = %.2f\n', truth);
fprintf('      你手里已有的估计(先验) x^- = %.2f, 标准差 %.2f\n', x_prior, s_prior);
fprintf('      这一次的测量(读数)      z   = %.2f, 标准差 %.2f\n', z_meas, s_meas);
fprintf('      问题: 真值取多少?\n');

% ---------- 三种做法 ----------
x_avg  = (x_prior + z_meas)/2;                        % 做法A: 直接平均
invP   = 1/Pp + 1/Rm;                                 % 精度(方差倒数)相加
x_wavg = (x_prior/Pp + z_meas/Rm)/invP;               % 做法B: 按精度加权平均
P_post = 1/invP;                                      % 合并后的方差

K      = Pp/(Pp + Rm);                                % 卡尔曼增益
x_kal  = (1-K)*x_prior + K*z_meas;                    % 做法C: 卡尔曼一步

fprintf('\n【做法A】直接平均                x = %.4f\n', x_avg);
fprintf('【做法B】按精度(1/方差)加权平均  x = %.4f   (权重 %.4f / %.4f)\n', ...
    x_wavg, (1/Pp)/invP, (1/Rm)/invP);
fprintf('【做法C】卡尔曼一步              x = %.4f   (K = %.4f)\n', x_kal, K);

fprintf('\n做法C 减 做法B = %.2e\n', x_kal - x_wavg);
fprintf('=> "卡尔曼滤波"这一步, 就是按精度加权平均。\n');

% ---------- 精度的算术 ----------
fprintf('\n再看合并后的不确定性:\n');
fprintf('  先验方差 P^- = %.4f\n', Pp);
fprintf('  测量方差 R   = %.4f\n', Rm);
fprintf('  合并方差 P+  = %.4f      (1/P+ = 1/P^- + 1/R = %.4f)\n', P_post, invP);
fprintf('  两边标准差: %.3f, %.3f  ->  合并后 %.4f\n', s_prior, s_meas, sqrt(P_post));
fprintf('=> 合并后比两边都更确定。这就是"信息只会越攒越多"。\n');

% ---------- 画图 ----------
xx = linspace(-0.5, 3.5, 800);
g1 = exp(-0.5*((xx-x_prior)/s_prior).^2)/(s_prior*sqrt(2*pi));
g2 = exp(-0.5*((xx-z_meas )/s_meas ).^2)/(s_meas *sqrt(2*pi));
gp = exp(-0.5*((xx-x_kal  )/sqrt(P_post)).^2)/(sqrt(P_post)*sqrt(2*pi));
figure('Position', [80 80 960 460], 'Color', 'w');
plot(xx, g1, '-', 'Color', [0 .45 .74], 'LineWidth', 1.8); hold on; grid on;
plot(xx, g2, '-', 'Color', [.85 .33 .10], 'LineWidth', 1.8);
plot(xx, gp, 'k-', 'LineWidth', 2.6);
plot([truth truth], [0 2.5], 'g--', 'LineWidth', 1.4);
xlabel('x'); ylabel('概率密度');
title(sprintf('两个高斯相乘: 先验×测量 -> 后验 %.4f (\\sigma=%.4f)', x_kal, sqrt(P_post)));
legend({sprintf('先验 N(%.2f, %.2f^2)', x_prior, s_prior), ...
        sprintf('测量 N(%.2f, %.2f^2)', z_meas, s_meas), ...
        sprintf('后验 N(%.4f, %.4f^2)', x_kal, sqrt(P_post)), ...
        sprintf('真值 %.2f', truth)}, 'Location', 'northwest');
fprintf('\n图已弹出: 黑线(后验)比蓝线、橙线都窄。\n');

% ---------- 留给你的题 ----------
fprintf('\n%s\n', repmat('-', 1, 70));
fprintf('【课后题, 改一下数字再跑】\n');
fprintf('  Q1 把 s_meas 从 0.20 改成 0.50 (传感器变烂, 和先验一样准)\n');
fprintf('     -> 猜猜 x 会是多少? K 会是多少? 合并方差会是多少?\n');
fprintf('  Q2 把 s_meas 改成 0.05 (传感器极准)\n');
fprintf('     -> x 会更靠近 1.70 还是更靠近 2.10?\n');
fprintf('  改完直接重跑 lesson1, 对照你猜的数字。\n');
fprintf('%s\n\n', repmat('-', 1, 70));
end
