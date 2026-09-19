function build_simple_lqr
% BUILD_SIMPLE_LQR Create the first LQR teaching model (xdot = u).
% Run once, then use run_simple_lqr(Q,R) to change the LQR weights.
folder = fileparts(mfilename('fullpath'));
mdl = 'simple_lqr_intro';
target = fullfile(folder, [mdl '.slx']);
if isfile(target) || bdIsLoaded(mdl)
    error('Model already exists or is loaded. Use run_simple_lqr instead.');
end
Q = 1; R = 1; x0 = 10;
K = lqr(0, 1, Q, R);
load_system('simulink');
new_system(mdl);
ws = get_param(mdl, 'ModelWorkspace');
assignin(ws, 'Q', Q); assignin(ws, 'R', R);
assignin(ws, 'K', K); assignin(ws, 'x0', x0);
set_param(mdl, 'StopTime', '5', 'Solver', 'ode45', ...
    'RelTol', '1e-8', 'AbsTol', '1e-10', 'MaxStep', '0.02', ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout');

add_block('simulink/Continuous/Integrator', [mdl '/Plant'], ...
    'InitialCondition', 'x0', 'Position', [270 140 320 190]);
add_block('simulink/Math Operations/Gain', [mdl '/LQR feedback'], ...
    'Gain', '-K', 'Orientation', 'left', 'Position', [260 290 330 340]);
add_block('simulink/Signal Routing/Mux', [mdl '/View x and u'], ...
    'Inputs', '2', 'Position', [490 145 495 225]);
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [560 165 610 205]);

h = add_line(mdl, 'Plant/1', 'View x and u/1', 'autorouting', 'on');
set_param(h, 'Name', 'x');
ports = get_param([mdl '/Plant'], 'PortHandles');
set_param(ports.Outport(1), 'DataLogging', 'on');
add_line(mdl, 'Plant/1', 'LQR feedback/1', 'autorouting', 'on');
h = add_line(mdl, 'LQR feedback/1', 'Plant/1', 'autorouting', 'on');
set_param(h, 'Name', 'u');
ports = get_param([mdl '/LQR feedback'], 'PortHandles');
set_param(ports.Outport(1), 'DataLogging', 'on');
add_line(mdl, 'LQR feedback/1', 'View x and u/2', 'autorouting', 'on');
add_line(mdl, 'View x and u/1', 'Scope/1', 'autorouting', 'on');

note = Simulink.Annotation(mdl, sprintf([ ...
    'FIRST LQR: bring x back to zero\n' ...
    'Plant: dx/dt = u, x(0) = 10\n' ...
    'Cost: integral(Q*x^2 + R*u^2) dt\n' ...
    'K = lqr(0,1,Q,R);  u = -K*x\n' ...
    'Default Q=R=1: K=1, x(t)=10*exp(-t)']));
note.Position = [95 20];
set_param(mdl, 'ZoomFactor', 'FitSystem');
save_system(mdl, target);
fprintf('Created: %s\n', target);
open_system(mdl);
end

