%% Simple continuous-time state feedback: u = -K*x
% Run this script to rebuild, simulate, and open the model.
outDir = fileparts(mfilename('fullpath'));
mdl = 'simple_state_feedback';
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
load_system('simulink');
new_system(mdl);
set_param(mdl, 'StopTime', '8', 'Solver', 'ode45', ...
    'RelTol', '1e-7', 'AbsTol', '1e-9', 'ReturnWorkspaceOutputs', 'on');

% A double integrator, with both states available for feedback.
A = [0 1; 0 0]; B = [0; 1]; C = eye(2); D = zeros(2,1);
K = [2 3]; x0 = [1; 0];
% Parameters live in the model so the saved .slx runs independently.
mw = get_param(mdl, 'ModelWorkspace');
assignin(mw, 'A', A); assignin(mw, 'B', B);
assignin(mw, 'C', C); assignin(mw, 'D', D);
assignin(mw, 'K', K); assignin(mw, 'x0', x0);

add_block('simulink/Continuous/State-Space', [mdl '/Plant'], ...
    'A','A','B','B','C','C','D','D','X0','x0', ...
    'Position',[300 145 465 215]);
add_block('simulink/Math Operations/Gain', [mdl '/State feedback'], ...
    'Gain','-K','Multiplication','Matrix(K*u)', ...
    'Orientation','left','Position',[310 315 450 365]);
add_block('simulink/Sinks/Scope', [mdl '/States x1 and x2'], ...
    'Position',[650 150 700 200]);
add_block('simulink/Sinks/Scope', [mdl '/Control u'], ...
    'Position',[90 40 140 90]);
add_block('simulink/Sinks/To Workspace', [mdl '/Save states'], ...
    'VariableName','x_log','SaveFormat','Timeseries', ...
    'Position',[620 235 740 265]);
add_block('simulink/Sinks/To Workspace', [mdl '/Save control'], ...
    'VariableName','u_log','SaveFormat','Timeseries', ...
    'Position',[70 235 190 265]);

ln = add_line(mdl,'Plant/1','States x1 and x2/1','autorouting','on');
set_param(ln,'Name','x = [x1; x2]');
add_line(mdl,'Plant/1','State feedback/1','autorouting','on');
add_line(mdl,'Plant/1','Save states/1','autorouting','on');
ln = add_line(mdl,'State feedback/1','Plant/1','autorouting','on');
set_param(ln,'Name','u = -Kx');
add_line(mdl,'State feedback/1','Control u/1','autorouting','on');
add_line(mdl,'State feedback/1','Save control/1','autorouting','on');

Simulink.Annotation(mdl, sprintf(['STATE FEEDBACK: u = -Kx\n' ...
    'dx1/dt = x2,  dx2/dt = u\n' ...
    'K = [2 3],  x(0) = [1; 0]\n' ...
    'Closed-loop poles: -1, -2; run time: 8 s']));
set_param(mdl,'Location',[100 100 1000 650]);
save_system(mdl,fullfile(outDir,[mdl '.slx']));

%% Verify against the exact closed-loop solution.
result = sim(mdl);
t = result.x_log.Time;
x = result.x_log.Data;
u = result.u_log.Data;
xExact = [2*exp(-t)-exp(-2*t), -2*exp(-t)+2*exp(-2*t)];
maxError = max(abs(x-xExact),[],'all');
assert(maxError < 1e-5, 'State trajectory differs from exact solution.');
assert(norm(x(end,:)) < 0.003, 'States did not converge as expected.');
assert(max(abs(u + x*K.'),[],'all') < 1e-7, 'Feedback law is inconsistent.');
save(fullfile(outDir,'simulation_results.mat'),'t','x','u','A','B','C','D','K','x0','maxError');
fig = figure('Visible','off','Color','w','Position',[100 100 900 650]);
tiledlayout(2,1);
nexttile; plot(t,x,'LineWidth',1.6); grid on;
ylabel('States'); legend('x_1','x_2'); title('State feedback: u = -[2  3]x');
nexttile; plot(t,u,'LineWidth',1.6); grid on;
xlabel('Time (s)'); ylabel('Control u');
exportgraphics(fig,fullfile(outDir,'state_feedback_response.png'),'Resolution',150);
close(fig);
fprintf('Model saved: %s\n',fullfile(outDir,[mdl '.slx']));
fprintf('Closed-loop poles: %g, %g\n',eig(A-B*K));
fprintf('Maximum trajectory error: %.3g\n',maxError);
fprintf('Final states: %.6f, %.6f\n',x(end,1),x(end,2));
if usejava('desktop')
    open_system(mdl);
end
