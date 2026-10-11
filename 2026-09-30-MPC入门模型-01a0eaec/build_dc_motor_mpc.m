%% Constrained linear MPC for the verified continuous DC motor.
% Requires Simulink, Control System Toolbox, Model Predictive Control Toolbox.
% Both current and speed are measured. Actual load is NOT fed to the MPC.
% An internal constant-load disturbance model and estimator support rejection.
outDir=fileparts(mfilename('fullpath'));
sourceFile=fullfile(outDir,'dc_motor_open_loop.slx');
assert(isfile(sourceFile),'Build the open-loop motor first.');
mdl='dc_motor_mpc'; n=1;
while bdIsLoaded(mdl) || exist(fullfile(outDir,[mdl '.slx']),'file')
    mdl=sprintf('dc_motor_mpc_%02d',n); n=n+1;
end
load_system(sourceFile); load_system('mpclib'); new_system(mdl);
set_param(mdl,'SolverType','Variable-step','Solver','ode45', ...
    'MaxStep','0.001','RelTol','1e-7','AbsTol','1e-9', ...
    'StopTime','8','ReturnWorkspaceOutputs','on');
ws=get_param(mdl,'ModelWorkspace');
assignin(ws,'R',1); assignin(ws,'L',0.01); assignin(ws,'Ke',0.1);
assignin(ws,'Kt',0.1); assignin(ws,'J',0.01); assignin(ws,'b',0.001);

%% Fixed physical model, exact zero-order-hold discretization.
% States [i;w]; outputs [w;i]; inputs [voltage;unmeasured load torque].
Ts=0.02; Np=60; Nc=10;
A=[-100 -10;10 -0.1]; B=[100 0;0 -100]; C=[0 1;1 0];
plant=ss(A,B,C,zeros(2,2));
plant.InputName={'Voltage','LoadTorque'};
plant.OutputName={'Speed','Current'};
plant=setmpcsignals(plant,'MV',1,'UD',2,'MO',[1 2]);
predictionModel=c2d(plant,Ts,'zoh');
controller=mpc(predictionModel,Ts,Np,Nc);
controller.MV.Min=0; controller.MV.Max=12; controller.MV.ScaleFactor=12;
controller.MV.MinECR=0; controller.MV.MaxECR=0;
controller.OV(1).ScaleFactor=60;
controller.OV(2).ScaleFactor=4;
% 0.2 A margin for intersample effects; check actual continuous current too.
controller.OV(2).Min=-3.8; controller.OV(2).Max=3.8;
controller.OV(2).MinECR=0; controller.OV(2).MaxECR=0;
controller.Weights.OutputVariables=[1 0];
controller.Weights.ManipulatedVariables=0;
controller.Weights.ManipulatedVariablesRate=0.1;
setoutdist(controller,'model',tf(zeros(2,1)));
% Random-walk torque estimate. A,B of the motor remain fixed.
setindist(controller,'model',ss(1,0.02,1,0,Ts));
getEstimator(controller); % validate augmented estimator observability now
controllerVar=[mdl '_controller'];
cfg=struct(); cfg.(controllerVar)=controller;
configFile=[mdl '_controller.mat'];
save(fullfile(outDir,configFile),'-struct','cfg');
% Reload controller on every model initialization, including a fresh session.
set_param(mdl,'InitFcn',sprintf("load(fullfile(fileparts(get_param(bdroot,'FileName')),'%s'));",configFile));
assignin('base',controllerVar,controller);

%% Top-level control loop: r -> MPC -> voltage -> continuous motor -> [w;i].
add_block('simulink/Sources/Step',[mdl '/Speed_reference'], ...
    'Time','0.2','Before','0','After','60','Position',[30 145 95 175]);
add_block('simulink/Sources/Constant',[mdl '/Unused_current_reference'], ...
    'Value','0','Position',[35 210 90 240]);
add_block('simulink/Signal Routing/Mux',[mdl '/Reference_w_i'], ...
    'Inputs','2','Position',[140 150 145 235]);
add_block('mpclib/MPC Controller',[mdl '/MPC_Controller'], ...
    'mpcobj',controllerVar,'md_inport','off','return_qpstatus','on', ...
    'Position',[225 95 405 210]);
add_block('dc_motor_open_loop/Electrical',[mdl '/Electrical'], ...
    'Position',[490 90 650 180]);
add_block('dc_motor_open_loop/Mechanical',[mdl '/Mechanical'], ...
    'Position',[730 90 895 185]);
add_block('simulink/Sources/Step',[mdl '/Load_step_at_4s'], ...
    'Time','4','Before','0','After','0.2','Position',[575 260 650 295]);
add_block('simulink/Signal Routing/Mux',[mdl '/Measured_w_i'], ...
    'Inputs','2','Position',[980 265 985 335]);
add_line(mdl,'Speed_reference/1','Reference_w_i/1','autorouting','on');
add_line(mdl,'Unused_current_reference/1','Reference_w_i/2','autorouting','on');
add_line(mdl,'Reference_w_i/1','MPC_Controller/2','autorouting','on');
add_line(mdl,'Measured_w_i/1','MPC_Controller/1','autorouting','on');
add_line(mdl,'MPC_Controller/1','Electrical/1','autorouting','on');
add_line(mdl,'Electrical/1','Mechanical/1','autorouting','on');
add_line(mdl,'Mechanical/1','Electrical/2','autorouting','on');
add_line(mdl,'Load_step_at_4s/1','Mechanical/2','autorouting','on');
add_line(mdl,'Mechanical/1','Measured_w_i/1','autorouting','on');
add_line(mdl,'Electrical/1','Measured_w_i/2','autorouting','on');

% Separate scopes: units and scales should not be confused.
add_block('simulink/Signal Routing/Mux',[mdl '/Target_and_speed'], ...
    'Inputs','2','Position',[1080 45 1085 100]);
add_block('simulink/Sinks/Scope',[mdl '/Speed_scope'], ...
    'Position',[1140 50 1190 90]);
add_block('simulink/Sinks/Scope',[mdl '/Current_scope'], ...
    'Position',[1140 140 1190 180]);
add_block('simulink/Sinks/Scope',[mdl '/Voltage_scope'], ...
    'Position',[1140 220 1190 260]);
add_line(mdl,'Speed_reference/1','Target_and_speed/1','autorouting','on');
add_line(mdl,'Mechanical/1','Target_and_speed/2','autorouting','on');
add_line(mdl,'Target_and_speed/1','Speed_scope/1','autorouting','on');
add_line(mdl,'Electrical/1','Current_scope/1','autorouting','on');
add_line(mdl,'MPC_Controller/1','Voltage_scope/1','autorouting','on');
add_block('simulink/Signal Routing/Mux',[mdl '/Log_signals'], ...
    'Inputs','6','Position',[1080 350 1085 500]);
add_block('simulink/Sinks/To Workspace',[mdl '/Log'], ...
    'VariableName','mpc_motor_trace','SaveFormat','Timeseries', ...
    'Position',[1135 395 1230 435]);
signals={'Speed_reference/1','Mechanical/1','Electrical/1', ...
    'MPC_Controller/1','Load_step_at_4s/1','MPC_Controller/2'};
for k=1:6
    add_line(mdl,signals{k},sprintf('Log_signals/%d',k),'autorouting','on');
end
add_line(mdl,'Log_signals/1','Log/1','autorouting','on');
note=Simulink.Annotation(mdl,sprintf(['DC MOTOR + CONSTRAINED MPC\n' ...
    'Ts=0.02 s | prediction=60 steps (1.2 s) | control=10 steps\n' ...
    'Target 60 rad/s at 0.2 s | unknown load +0.2 Nm at 4 s\n' ...
    'Voltage 0..12 V | predicted current +/-3.8 A | physical check +/-4 A\n' ...
    'Both speed and current measured. Load is NOT connected to MPC.\n' ...
    'Built-in state/disturbance estimator; fixed motor model.\n' ...
    'Current reference has zero weight: current is constrained, not tracked to zero.']));
note.Position=[45 420];
save_system(mdl,fullfile(outDir,[mdl '.slx']));

%% Reopen and remove workspace object: prove InitFcn reloads dependencies.
close_system(mdl,0); evalin('base',['clear ' controllerVar]);
load_system(fullfile(outDir,[mdl '.slx']));
set_param([mdl '/Load_step_at_4s'],'After','0');
r0=sim(mdl); noLoad=r0.get('mpc_motor_trace');
set_param([mdl '/Load_step_at_4s'],'After','0.2');
save_system(mdl,fullfile(outDir,[mdl '.slx']));
r1=sim(mdl); loaded=r1.get('mpc_motor_trace');
traces={noLoad,loaded}; metrics=struct([]);
for caseIndex=1:2
    tr=traces{caseIndex}; t=tr.Time; y=squeeze(tr.Data);
    metrics(caseIndex).peakCurrent=max(abs(y(:,3)));
    metrics(caseIndex).minVoltage=min(y(:,4));
    metrics(caseIndex).maxVoltage=max(y(:,4));
    metrics(caseIndex).finalSpeed=y(end,2);
    metrics(caseIndex).finalCurrent=y(end,3);
    metrics(caseIndex).finalVoltage=y(end,4);
    metrics(caseIndex).finalError=abs(60-y(end,2));
    metrics(caseIndex).minQPStatus=min(y(:,6));
    metrics(caseIndex).overshoot=max(0,max(y(:,2))-60);
    metrics(caseIndex).loadDip=60-min(y(t>=4 & t<=5,2));
    assert(all(isfinite(y(:))),'Nonfinite simulation result.');
    assert(min(y(:,4))>=-1e-5 && max(y(:,4))<=12+1e-5,'Voltage constraint violated.');
    assert(max(abs(y(:,3)))<=4.001,'Continuous current exceeded physical check limit.');
    assert(all(y(:,6)>0),'QP infeasible or iteration limit reached.');
    assert(metrics(caseIndex).finalError<0.2,'Speed tracking did not recover.');
end
save(fullfile(outDir,[mdl '_results.mat']),'noLoad','loaded','metrics','Ts','Np','Nc');
disp(metrics(1)); disp(metrics(2));
fig=figure('Visible','off','Color','w','Position',[100 100 1100 900]);
t0=noLoad.Time; y0=squeeze(noLoad.Data); t=loaded.Time; y=squeeze(loaded.Data);
subplot(4,1,1); plot(t,y(:,1),'k--',t0,y0(:,2),t,y(:,2),'LineWidth',1.2);
grid on; ylabel('Speed (rad/s)'); legend('Target','No load','Load at 4 s','Location','best');
title('DC motor MPC: 60 rad/s, voltage and current constraints');
subplot(4,1,2); plot(t0,y0(:,3),t,y(:,3),'LineWidth',1.2); hold on;
yline(3.8,'k:','Prediction limit'); yline(4,'r--','Physical check');
grid on; ylabel('Current (A)');
subplot(4,1,3); plot(t0,y0(:,4),t,y(:,4),'LineWidth',1.2); hold on;
yline(12,'r--'); grid on; ylabel('Voltage (V)');
subplot(4,1,4); plot(t,y(:,5),'LineWidth',1.2); grid on;
ylabel('Load (Nm)'); xlabel('Time (s)');
saveas(fig,fullfile(outDir,[mdl '_response.png'])); close(fig);
open_system(mdl);
print(['-s' mdl],'-dpng',fullfile(outDir,[mdl '_diagram.png']));
fprintf('PASS: reopened model; tracking; continuous current; voltage; all QP solves.\n');
fprintf('MODEL: %s\n',fullfile(outDir,[mdl '.slx']));
