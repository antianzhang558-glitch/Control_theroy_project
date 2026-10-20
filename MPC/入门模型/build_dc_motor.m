%% Continuous DC motor: build and verify two open-loop experiments.
% Teaching parameters, not a manufacturer's motor specification.
% Run this file to create a fresh model; existing models are preserved.
outDir = fileparts(mfilename('fullpath'));
mdl = 'dc_motor_open_loop'; n = 1;
while bdIsLoaded(mdl) || exist(fullfile(outDir,[mdl '.slx']),'file')
    mdl = sprintf('dc_motor_open_loop_%02d',n); n = n+1;
end
load_system('simulink'); new_system(mdl);
set_param(mdl,'SolverType','Variable-step','Solver','ode45', ...
    'MaxStep','0.002','RelTol','1e-7','AbsTol','1e-9', ...
    'StopTime','8','ReturnWorkspaceOutputs','on');
ws = get_param(mdl,'ModelWorkspace');
assignin(ws,'R',1); assignin(ws,'L',0.01);
assignin(ws,'Ke',0.1); assignin(ws,'Kt',0.1);
assignin(ws,'J',0.01); assignin(ws,'b',0.001);
assignin(ws,'Udc',12); assignin(ws,'loadTorque',0);

add_block('simulink/Sources/Constant',[mdl '/Voltage_12V'], ...
    'Value','Udc','Position',[35 80 100 115]);
add_block('simulink/Sources/Step',[mdl '/Load_step_at_4s'], ...
    'Time','4','Before','0','After','loadTorque', ...
    'Position',[290 230 365 265]);
add_block('built-in/Subsystem',[mdl '/Electrical'], ...
    'Position',[190 70 340 160]);
e = [mdl '/Electrical'];
add_block('simulink/Ports & Subsystems/In1',[e '/voltage_V'], ...
    'Port','1','Position',[25 45 55 65]);
add_block('simulink/Ports & Subsystems/In1',[e '/speed_rad_s'], ...
    'Port','2','Position',[25 240 55 260]);
add_block('simulink/Math Operations/Sum',[e '/Net_inductor_voltage'], ...
    'Inputs','+--','Position',[220 45 250 135]);
add_block('simulink/Math Operations/Gain',[e '/One_over_L'], ...
    'Gain','1/L','Position',[310 65 370 105]);
add_block('simulink/Continuous/Integrator',[e '/Current_integrator'], ...
    'InitialCondition','0','Position',[425 65 465 105]);
add_block('simulink/Math Operations/Gain',[e '/Resistance_R'], ...
    'Gain','R','Orientation','left','Position',[300 175 365 215]);
add_block('simulink/Math Operations/Gain',[e '/Back_EMF_Ke'], ...
    'Gain','Ke','Position',[105 230 165 270]);
add_block('simulink/Ports & Subsystems/Out1',[e '/current_A'], ...
    'Position',[535 75 565 95]);
add_line(e,'voltage_V/1','Net_inductor_voltage/1','autorouting','on');
add_line(e,'Resistance_R/1','Net_inductor_voltage/2','autorouting','on');
add_line(e,'Back_EMF_Ke/1','Net_inductor_voltage/3','autorouting','on');
add_line(e,'Net_inductor_voltage/1','One_over_L/1','autorouting','on');
add_line(e,'One_over_L/1','Current_integrator/1','autorouting','on');
add_line(e,'Current_integrator/1','current_A/1','autorouting','on');
add_line(e,'Current_integrator/1','Resistance_R/1','autorouting','on');
add_line(e,'speed_rad_s/1','Back_EMF_Ke/1','autorouting','on');
a = Simulink.Annotation(e,sprintf('L * di/dt = u - R*i - Ke*w\nIntegrator 1/s: input di/dt (A/s), output i (A).\nFinite voltage cannot make current jump when L > 0.')); a.Position=[50 320];

add_block('built-in/Subsystem',[mdl '/Mechanical'], ...
    'Position',[450 75 605 170]);
m = [mdl '/Mechanical'];
add_block('simulink/Ports & Subsystems/In1',[m '/current_A'], ...
    'Port','1','Position',[25 55 55 75]);
add_block('simulink/Ports & Subsystems/In1',[m '/load_Nm'], ...
    'Port','2','Position',[25 240 55 260]);
add_block('simulink/Math Operations/Gain',[m '/Torque_Kt'], ...
    'Gain','Kt','Position',[90 45 150 85]);
add_block('simulink/Math Operations/Sum',[m '/Net_torque'], ...
    'Inputs','+--','Position',[220 45 250 135]);
add_block('simulink/Math Operations/Gain',[m '/One_over_J'], ...
    'Gain','1/J','Position',[310 65 370 105]);
add_block('simulink/Continuous/Integrator',[m '/Speed_integrator'], ...
    'InitialCondition','0','Position',[425 65 465 105]);
add_block('simulink/Math Operations/Gain',[m '/Friction_b'], ...
    'Gain','b','Orientation','left','Position',[300 175 365 215]);
add_block('simulink/Ports & Subsystems/Out1',[m '/speed_rad_s'], ...
    'Position',[535 75 565 95]);
add_line(m,'current_A/1','Torque_Kt/1','autorouting','on');
add_line(m,'Torque_Kt/1','Net_torque/1','autorouting','on');
add_line(m,'Friction_b/1','Net_torque/2','autorouting','on');
add_line(m,'load_Nm/1','Net_torque/3','autorouting','on');
add_line(m,'Net_torque/1','One_over_J/1','autorouting','on');
add_line(m,'One_over_J/1','Speed_integrator/1','autorouting','on');
add_line(m,'Speed_integrator/1','speed_rad_s/1','autorouting','on');
add_line(m,'Speed_integrator/1','Friction_b/1','autorouting','on');
a = Simulink.Annotation(m,sprintf('J * dw/dt = Kt*i - b*w - TL\nIntegrator 1/s: input acceleration (rad/s^2), output speed (rad/s).\nIncreasing torque does not imply acceleration until net torque > 0.')); a.Position=[50 320];

add_block('simulink/Signal Routing/Mux',[mdl '/Signals_u_TL_i_w'], ...
    'Inputs','4','Position',[720 70 725 255]);
add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[800 80 855 120]);
add_block('simulink/Sinks/To Workspace',[mdl '/Log'], ...
    'VariableName','motor_trace','SaveFormat','Timeseries','Position',[795 190 885 230]);
add_line(mdl,'Voltage_12V/1','Electrical/1','autorouting','on');
add_line(mdl,'Electrical/1','Mechanical/1','autorouting','on');
add_line(mdl,'Mechanical/1','Electrical/2','autorouting','on');
add_line(mdl,'Load_step_at_4s/1','Mechanical/2','autorouting','on');
add_line(mdl,'Voltage_12V/1','Signals_u_TL_i_w/1','autorouting','on');
add_line(mdl,'Load_step_at_4s/1','Signals_u_TL_i_w/2','autorouting','on');
add_line(mdl,'Electrical/1','Signals_u_TL_i_w/3','autorouting','on');
add_line(mdl,'Mechanical/1','Signals_u_TL_i_w/4','autorouting','on');
add_line(mdl,'Signals_u_TL_i_w/1','Scope/1','autorouting','on');
add_line(mdl,'Signals_u_TL_i_w/1','Log/1','autorouting','on');
a=Simulink.Annotation(mdl,sprintf('CONTINUOUS DC MOTOR - open loop, no MPC yet\nR=1 ohm, L=0.01 H, Ke=Kt=0.1, J=0.01, b=0.001\nDefault: 12 V, zero load, zero initial current and speed.\nFor load experiment: set Load_step_at_4s final value to 0.2 (Nm).\nScope order: voltage (V), load (Nm), current (A), speed (rad/s).')); a.Position=[40 340];
save_system(mdl,fullfile(outDir,[mdl '.slx']));

%% Tests on actual Simulink trajectories and independent analytical solution.
Ac=[-100 -10;10 -0.1]; Bc=[100;0]; Ec=[0;-100];
xs0=-(Ac\(Bc*12));
xsL=-(Ac\(Bc*12+Ec*0.2));
r0=sim(mdl); tr0=r0.get('motor_trace');
set_param([mdl '/Load_step_at_4s'],'After','0.2');
try
    rL=sim(mdl); trL=rL.get('motor_trace');
catch err
    set_param([mdl '/Load_step_at_4s'],'After','loadTorque');
    rethrow(err);
end
set_param([mdl '/Load_step_at_4s'],'After','loadTorque');
save_system(mdl,fullfile(outDir,[mdl '.slx']));
traces={tr0,trL}; errors=zeros(1,2);
for experiment=1:2
    tr=traces{experiment}; t=tr.Time; y=squeeze(tr.Data);
    expected=zeros(numel(t),2);
    x4=xs0+expm(Ac*4)*(-xs0);
    for k=1:numel(t)
        if experiment==2 && t(k)>=4
            x=xsL+expm(Ac*(t(k)-4))*(x4-xsL);
        else
            x=xs0+expm(Ac*t(k))*(-xs0);
        end
        expected(k,:)=x';
    end
    errors(experiment)=max(max(abs(y(:,3:4)-expected)));
    assert(errors(experiment)<1e-3,'Trajectory differs from analytical motor model.');
    assert(max(abs(y(:,1)-12))<1e-10,'Voltage must remain constant.');
    assert(max(abs(y(1,3:4)))<1e-10,'Initial states must be zero.');
end
y0=squeeze(tr0.Data); yL=squeeze(trL.Data);
assert(max(abs(y0(end,3:4)-xs0'))<0.1,'No-load steady state mismatch.');
assert(yL(end,3)>y0(end,3) && yL(end,4)<y0(end,4), ...
    'Added load should increase current and reduce final speed.');
save(fullfile(outDir,[mdl '_results.mat']),'tr0','trL','xs0','xsL','errors');
fig=figure('Visible','off','Color','w','Position',[100 100 1100 780]);
subplot(3,1,1); plot(tr0.Time,y0(:,3),trL.Time,yL(:,3),'LineWidth',1.3);
grid on; ylabel('Current (A)'); legend('No load','0.2 Nm added at 4 s');
title('12 V DC motor: open-loop verification');
subplot(3,1,2); plot(tr0.Time,y0(:,4),trL.Time,yL(:,4),'LineWidth',1.3);
grid on; ylabel('Speed (rad/s)');
subplot(3,1,3); plot(trL.Time,yL(:,2),'LineWidth',1.3);
grid on; ylabel('Load (Nm)'); xlabel('Time (s)');
saveas(fig,fullfile(outDir,[mdl '_response.png'])); close(fig);
open_system(mdl);
fprintf('MODEL: %s\n',fullfile(outDir,[mdl '.slx']));
fprintf('PASS: two simulations agree with analytical trajectories, max errors %.3g / %.3g\n',errors);
fprintf('No-load final: current %.6f A, speed %.6f rad/s\n',y0(end,3:4));
fprintf('Loaded final: current %.6f A, speed %.6f rad/s\n',yL(end,3:4));
fprintf('Theoretical equilibria: no load %.6f A / %.6f rad/s; loaded %.6f A / %.6f rad/s\n',xs0(1),xs0(2),xsL(1),xsL(2));
