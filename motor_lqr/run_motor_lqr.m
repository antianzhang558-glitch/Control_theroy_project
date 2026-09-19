function out = run_motor_lqr(speedRef, qSpeed, rVoltage)
% RUN_MOTOR_LQR Two-state DC motor LQR with a constant speed target.
% run_motor_lqr           : 10 rad/s, Q=diag([10 0.1]), Ru=1
% run_motor_lqr(10,40,1)  : penalize speed error more
% run_motor_lqr(10,10,4)  : penalize voltage correction more
% Teaching model: no external load, no voltage limit, full state feedback.
if nargin < 1, speedRef = 10; end
if nargin < 2, qSpeed = 10; end
if nargin < 3, rVoltage = 1; end
validateattributes(speedRef, {'numeric'}, {'scalar','real','finite'});
validateattributes(qSpeed, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(rVoltage, {'numeric'}, {'scalar','real','finite','positive'});
folder = fileparts(mfilename('fullpath'));
mdl = 'motor_lqr_intro';
target = fullfile(folder, [mdl '.slx']);

% SI units: J [kg*m^2], b [N*m*s/rad], Kt [N*m/A], Ke [V*s/rad].
J = 0.01; b = 0.01; Kt = 0.1; Ke = 0.1;
Ra = 1; L = 0.1; % resistance [ohm], inductance [H]
% State order is always [speed (rad/s); current (A)].
A = [-b/J Kt/J; -Ke/L -Ra/L];
B = [0; 1/L];
Q = diag([qSpeed 0.1]); Ru = rVoltage;
[K, P, poles] = lqr(A, B, Q, Ru);
omega_ref = speedRef;
i_ref = b*omega_ref/Kt;
x_ref = [omega_ref; i_ref];
u_ref = Ra*i_ref + Ke*omega_ref;
assert(norm(A*x_ref+B*u_ref) < 1e-9, 'Incorrect equilibrium.');
assert(all(real(poles)<0), 'Unstable closed loop.');
assert(norm(A'*P+P*A-P*B*(B'*P)/Ru+Q,'fro') < 1e-7, ...
    'Riccati equation residual is too large.');
stopTime = max(5, 12/min(-real(poles)));

created = ~isfile(target);
if created
    if bdIsLoaded(mdl)
        error('An unsaved model named %s is already open.', mdl);
    end
    new_system(mdl);
else
    load_system(target);
end
ws = get_param(mdl, 'ModelWorkspace');
params = struct('A',A,'B',B,'Q',Q,'Ru',Ru,'K',K, ...
    'omega_ref',omega_ref,'i_ref',i_ref,'x_ref',x_ref,'u_ref',u_ref);
names = fieldnames(params);
for n=1:numel(names), assignin(ws,names{n},params.(names{n})); end
set_param(mdl, 'StopTime',num2str(stopTime,17), 'Solver','ode45', ...
    'RelTol','1e-8','AbsTol','1e-10','MaxStep','0.01', ...
    'SignalLogging','on','SignalLoggingName','logsout');

if created
    load_system('simulink');
    add_block('simulink/Sources/Constant',[mdl '/Target states'], ...
        'Value','x_ref','Position',[50 180 135 220]);
    add_block('simulink/Math Operations/Sum',[mdl '/Target minus actual'], ...
        'Inputs','+-','Position',[215 180 245 220]);
    add_block('simulink/Math Operations/Gain',[mdl '/LQR gain'], ...
        'Gain','K','Multiplication','Matrix(K*u)', ...
        'Position',[295 178 370 222]);
    add_block('simulink/Sources/Constant',[mdl '/Holding voltage'], ...
        'Value','u_ref','Position',[320 75 400 115]);
    add_block('simulink/Math Operations/Sum',[mdl '/Total voltage'], ...
        'Inputs','++','Position',[445 180 475 220]);
    add_block('simulink/Continuous/State-Space',[mdl '/DC motor'], ...
        'A','A','B','B','C','eye(2)','D','zeros(2,1)','X0','[0;0]', ...
        'Position',[540 175 660 225]);
    add_block('simulink/Signal Routing/Demux',[mdl '/Speed and current'], ...
        'Outputs','2','Position',[735 170 740 230]);
    add_block('simulink/Sources/Constant',[mdl '/Speed target'], ...
        'Value','omega_ref','Position',[780 70 850 100]);
    add_block('simulink/Sinks/Scope',[mdl '/Speed scope'], ...
        'NumInputPorts','2','Position',[945 140 995 190]);
    add_block('simulink/Sinks/Scope',[mdl '/Current scope'], ...
        'Position',[945 240 995 280]);
    add_block('simulink/Sinks/Scope',[mdl '/Voltage scope'], ...
        'Position',[555 380 605 420]);

    add_line(mdl,'Target states/1','Target minus actual/1','autorouting','on');
    h=add_line(mdl,'Target minus actual/1','LQR gain/1','autorouting','on');
    set_param(h,'Name','x_ref - x');
    add_line(mdl,'LQR gain/1','Total voltage/1','autorouting','on');
    add_line(mdl,'Holding voltage/1','Total voltage/2','autorouting','on');
    h=add_line(mdl,'Total voltage/1','DC motor/1','autorouting','on');
    set_param(h,'Name','voltage');
    ports=get_param([mdl '/Total voltage'],'PortHandles');
    set_param(ports.Outport(1),'DataLogging','on');
    h=add_line(mdl,'DC motor/1','Speed and current/1','autorouting','on');
    set_param(h,'Name','states');
    ports=get_param([mdl '/DC motor'],'PortHandles');
    set_param(ports.Outport(1),'DataLogging','on');
    add_line(mdl,'DC motor/1','Target minus actual/2','autorouting','on');
    add_line(mdl,'Speed and current/1','Speed scope/1','autorouting','on');
    add_line(mdl,'Speed target/1','Speed scope/2','autorouting','on');
    add_line(mdl,'Speed and current/2','Current scope/1','autorouting','on');
    add_line(mdl,'Total voltage/1','Voltage scope/1','autorouting','on');
    note=Simulink.Annotation(mdl,sprintf(['DC MOTOR LQR: constant speed tracking\n' ...
        'x = [speed; current], initial x = [0;0]\n' ...
        'u = u_ref + K*(x_ref - x)\n' ...
        'K is computed once before simulation.']));
    note.Position=[40 15];
    note=Simulink.Annotation(mdl,sprintf(['Teaching assumptions: known motor, no external load,\n' ...
        'ideal voltage source, both states available.\n' ...
        'Use run_motor_lqr(...) to change target or weights.']));
    note.Position=[50 390];
    set_param(mdl,'ZoomFactor','FitSystem');
    save_system(mdl,target);
end

out=sim(mdl,'ReturnWorkspaceOutputs','on');
states=out.logsout.get('states').Values;
voltage=out.logsout.get('voltage').Values;
t=states.Time; x=states.Data;
% Independent exact solution of the shifted linear closed loop.
expected=zeros(size(x));
for n=1:numel(t)
    expected(n,:)=(x_ref+expm((A-B*K)*t(n))*(-x_ref)).';
end
maxError=max(abs(x-expected),[],'all');
assert(maxError < 1e-5*max(1,abs(speedRef)), 'Incorrect state trajectory.');
expectedVoltage=zeros(size(voltage.Time));
for n=1:numel(voltage.Time)
    e=expm((A-B*K)*voltage.Time(n))*(-x_ref);
    expectedVoltage(n)=u_ref-K*e;
end
assert(max(abs(voltage.Data(:)-expectedVoltage(:))) < 1e-5*max(1,abs(speedRef)), ...
    'Incorrect feedback or feedforward voltage.');
assert(abs(x(end,1)-omega_ref)<1e-3*max(1,abs(omega_ref)), ...
    'Speed did not reach its target.');
fprintf('K = [%.8f %.8f]\n', K);
fprintf('Target: speed=%g rad/s, current=%g A, holding voltage=%g V\n', ...
    omega_ref,i_ref,u_ref);
fprintf('Final: speed=%.8f, current=%.8f, voltage=%.8f\n', ...
    x(end,1),x(end,2),voltage.Data(end));
fprintf('Maximum state error versus exact solution: %.3g\n',maxError);

fig=figure('Name','DC motor LQR','Color','white','Position',[100 100 900 760]);
tiledlayout(3,1);
nexttile; plot(t,x(:,1),'LineWidth',1.8); hold on;
yline(omega_ref,'--'); grid on; ylabel('Speed (rad/s)');
legend('Actual','Target','Location','best');
title(sprintf('LQR: K=[%.3f %.3f], Q=diag([%g 0.1]), Ru=%g',K,qSpeed,Ru));
nexttile; plot(t,x(:,2),'LineWidth',1.8); hold on;
yline(i_ref,'--'); grid on; ylabel('Current (A)');
legend('Actual','Equilibrium','Location','best');
nexttile; plot(voltage.Time,voltage.Data,'LineWidth',1.8); hold on;
yline(u_ref,'--'); grid on; ylabel('Voltage (V)'); xlabel('Time (s)');
legend('Total voltage','Holding voltage','Location','best');
axesHandles=findall(fig,'Type','axes');
for n=1:numel(axesHandles), axesHandles(n).Toolbar.Visible='off'; end
exportgraphics(fig,fullfile(folder,'motor_lqr_response.png'),'Resolution',150);
open_system(mdl);
end
