function model = build_cascade_pid
% Build a self-contained position/speed cascade control teaching model.
folder = fileparts(mfilename('fullpath'));
model = 'cascade_pid_motor';
file = fullfile(folder,[model '.slx']);
if bdIsLoaded(model) || isfile(file)
    error('Model already exists or is open. Preserve your edits; rename it before rebuilding.');
end
load_system('simulink');
new_system(model);
set_param(model,'Solver','ode45','StopTime','16','MaxStep','0.002', ...
    'RelTol','1e-6','AbsTol','1e-8','ReturnWorkspaceOutputs','on', ...
    'Location',[40 60 1450 840]);
ws = get_param(model,'ModelWorkspace');
p = struct('R',1,'L',0.01,'Kt',0.1,'Ke',0.1,'J',0.01,'b',0.001, ...
    'Kp_pos',4,'Ki_pos',3,'Kd_pos',0.05,'N_pos',30, ...
    'Kp_speed',4,'Ki_speed',40,'speed_limit',8,'voltage_limit',24, ...
    'theta_step',pi/2,'theta_change',-pi,'load_torque',0.05);
names = fieldnames(p);
for k=1:numel(names), assignin(ws,names{k},p.(names{k})); end
blk = @(lib,name,pos,varargin) add_block(['simulink/' lib],[model '/' name], ...
    'Position',pos,varargin{:});
blk('Sources/Step','Position step',[40 160 70 190],'Time','0.5','After','theta_step');
blk('Sources/Step','Reverse step',[40 235 70 265],'Time','8','After','theta_change');
blk('Math Operations/Sum','Reference',[120 165 145 200],'Inputs','++');
blk('Math Operations/Sum','Position error',[230 165 255 200],'Inputs','+-');
blk('Continuous/PID Controller','Position PID',[305 145 415 215], ...
    'Controller','PID','P','Kp_pos','I','Ki_pos','D','Kd_pos','N','N_pos', ...
    'LimitOutput','on','UpperSaturationLimit','speed_limit', ...
    'LowerSaturationLimit','-speed_limit','AntiWindupMode','back-calculation','Kb','1');
blk('Math Operations/Sum','Speed error',[475 165 500 200],'Inputs','+-');
blk('Continuous/PID Controller','Speed PI',[555 145 665 215], ...
    'Controller','PI','P','Kp_speed','I','Ki_speed', ...
    'LimitOutput','on','UpperSaturationLimit','voltage_limit', ...
    'LowerSaturationLimit','-voltage_limit','AntiWindupMode','back-calculation','Kb','20');
blk('Ports & Subsystems/Subsystem','DC motor',[745 145 870 235]);
plant = [model '/DC motor'];
Simulink.SubSystem.deleteContents(plant);
add_block('simulink/Sources/In1',[plant '/Voltage'],'Position',[25 68 55 82],'Port','1');
add_block('simulink/Sources/In1',[plant '/Load torque'],'Position',[265 180 295 194],'Port','2');
add_block('simulink/Math Operations/Sum',[plant '/Net voltage'],'Position',[105 60 130 95],'Inputs','+-');
add_block('simulink/Continuous/Transfer Fcn',[plant '/Armature RL'],'Position',[170 50 270 100], ...
    'Numerator','1','Denominator','[L R]');
add_block('simulink/Math Operations/Gain',[plant '/Torque constant'],'Position',[320 55 370 95],'Gain','Kt');
add_block('simulink/Math Operations/Sum',[plant '/Net torque'],'Position',[420 60 445 95],'Inputs','+-');
add_block('simulink/Continuous/Transfer Fcn',[plant '/Rotor mechanics'],'Position',[495 50 605 100], ...
    'Numerator','1','Denominator','[J b]');
add_block('simulink/Math Operations/Gain',[plant '/Back EMF'],'Position',[325 270 385 310], ...
    'Gain','Ke','Orientation','left');
add_block('simulink/Sinks/Out1',[plant '/Speed'],'Position',[665 68 695 82],'Port','1');
add_block('simulink/Sinks/Out1',[plant '/Current'],'Position',[665 188 695 202],'Port','2');
connect(plant,'Voltage/1','Net voltage/1'); connect(plant,'Net voltage/1','Armature RL/1');
connect(plant,'Armature RL/1','Torque constant/1'); connect(plant,'Torque constant/1','Net torque/1');
connect(plant,'Load torque/1','Net torque/2'); connect(plant,'Net torque/1','Rotor mechanics/1');
connect(plant,'Rotor mechanics/1','Speed/1'); connect(plant,'Rotor mechanics/1','Back EMF/1');
connect(plant,'Back EMF/1','Net voltage/2'); connect(plant,'Armature RL/1','Current/1');
note = Simulink.Annotation(plant,sprintf('L di/dt = u - R i - Ke*w\nJ dw/dt = Kt*i - b*w - TL'));
note.Position=[140 345];
blk('Continuous/Integrator','Position',[940 151 975 189]);
blk('Sources/Step','Load on',[555 315 585 345],'Time','4','After','load_torque');
blk('Sources/Step','Load off',[555 390 585 420],'Time','12','After','-load_torque');
blk('Math Operations/Sum','Load torque',[650 320 675 355],'Inputs','++');
blk('Signal Routing/Mux','Position signals',[1070 145 1075 205],'Inputs','2');
blk('Signal Routing/Mux','Speed signals',[1070 280 1075 340],'Inputs','2');
blk('Signal Routing/Mux','Actuator signals',[1070 405 1075 480],'Inputs','3');
groups={'Position','Speed','Actuator'};
variables={'position_log','speed_log','actuator_log'};
ys=[160 295 425];
for k=1:3
    blk('Sinks/Scope',[groups{k} ' scope'],[1140 ys(k) 1175 ys(k)+35]);
    blk('Sinks/To Workspace',[groups{k} ' log'],[1220 ys(k)+40 1330 ys(k)+70], ...
        'VariableName',variables{k},'SaveFormat','Timeseries','MaxDataPoints','inf');
    connect(model,[groups{k} ' signals/1'],[groups{k} ' scope/1']);
    connect(model,[groups{k} ' signals/1'],[groups{k} ' log/1']);
end
connect(model,'Position step/1','Reference/1'); connect(model,'Reverse step/1','Reference/2');
connect(model,'Reference/1','Position error/1','theta ref (rad)');
connect(model,'Position error/1','Position PID/1');
connect(model,'Position PID/1','Speed error/1','speed ref (rad/s)');
connect(model,'Speed error/1','Speed PI/1');
connect(model,'Speed PI/1','DC motor/1','voltage (V)');
connect(model,'Load on/1','Load torque/1'); connect(model,'Load off/1','Load torque/2');
connect(model,'Load torque/1','DC motor/2','load (N m)');
connect(model,'DC motor/1','Position/1','speed (rad/s)');
connect(model,'DC motor/1','Speed error/2');
connect(model,'Position/1','Position error/2','theta (rad)');
connect(model,'Reference/1','Position signals/1'); connect(model,'Position/1','Position signals/2');
connect(model,'Position PID/1','Speed signals/1'); connect(model,'DC motor/1','Speed signals/2');
connect(model,'Speed PI/1','Actuator signals/1'); connect(model,'DC motor/2','Actuator signals/2','current (A)');
connect(model,'Load torque/1','Actuator signals/3');
a=Simulink.Annotation(model,sprintf(['CASCADE PID: MOTOR POSITION + SPEED\n' ...
    'Outer PID -> speed command (+/-8 rad/s) | Inner PI -> voltage (+/-24 V)\n' ...
    'Double-click PID blocks to tune; double-click DC motor to inspect physical dynamics.']));
a.Position=[45 25];
a=Simulink.Annotation(model,sprintf(['EXPERIMENT: 0.5 s: +90 deg | 4 s: add load | 8 s: -90 deg | 12 s: remove load\n' ...
    'Position scope: [reference, actual] rad | Speed scope: [reference, actual] rad/s\n' ...
    'Actuator scope: [voltage V, current A, load N*m]. Parameters are saved in Model Workspace.']));
a.Position=[45 525];
set_param(model,'SimulationCommand','update');
save_system(model,file);
open_system(model);
set_param(model,'ZoomFactor','FitSystem');
print(['-s' model],'-dpng','-r130',fullfile(folder,'model_overview.png'));
fprintf('Created %s\n',file);
end

function connect(system,source,destination,name)
h=add_line(system,source,destination,'autorouting','on');
if nargin==4, set_param(h,'Name',name); end
end
