function out = run_simple_lqr(Q, R)
% RUN_SIMPLE_LQR Simulate and inspect the simplest LQR feedback loop.
% run_simple_lqr       : Q=1, R=1, K=1
% run_simple_lqr(4,1)  : K=2, faster response
% run_simple_lqr(1,4)  : K=0.5, gentler response
if nargin < 1, Q = 1; end
if nargin < 2, R = 1; end
validateattributes(Q, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(R, {'numeric'}, {'scalar','real','finite','positive'});
folder = fileparts(mfilename('fullpath'));
mdl = 'simple_lqr_intro';
if ~isfile(fullfile(folder, [mdl '.slx']))
    build_simple_lqr;
else
    load_system(fullfile(folder, [mdl '.slx']));
end
K = lqr(0, 1, Q, R);
x0 = 10;
ws = get_param(mdl, 'ModelWorkspace');
assignin(ws, 'Q', Q); assignin(ws, 'R', R);
assignin(ws, 'K', K); assignin(ws, 'x0', x0);
out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
x = out.logsout.get('x').Values;
u = out.logsout.get('u').Values;
expected = x0 * exp(-K*x.Time);
maxError = max(abs(x.Data(:) - expected(:)));
assert(abs(K-sqrt(Q/R)) < 1e-10, 'Unexpected LQR gain.');
assert(maxError < 1e-5, 'Simulation disagrees with analytical solution.');
assert(max(abs(u.Data(:) + K*x0*exp(-K*u.Time(:)))) < 1e-5, ...
    'Control signal disagrees with u=-K*x.');
fprintf('Q=%g, R=%g, K=%g; maximum simulation error = %.3g\n', Q,R,K,maxError);

fig = figure('Name', 'First LQR: state and control', 'Color', 'white');
tiledlayout(2,1);
nexttile;
plot(x.Time, x.Data, 'LineWidth', 1.8); hold on;
plot(x.Time, expected, '--', 'LineWidth', 1);
grid on; ylabel('State x');
legend('Simulink', '10 exp(-Kt)', 'Location', 'northeast');
title(sprintf('Q=%g, R=%g, K=%g: dx/dt=u, u=-Kx', Q,R,K));
nexttile;
plot(u.Time, u.Data, 'LineWidth', 1.8);
grid on; ylabel('Control u'); xlabel('Time (s)');
exportgraphics(fig, fullfile(folder, 'simple_lqr_response.png'), 'Resolution', 150);
open_system(mdl);
end
