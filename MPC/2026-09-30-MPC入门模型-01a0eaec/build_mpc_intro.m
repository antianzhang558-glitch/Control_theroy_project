%% One-step scalar MPC: build, open, simulate and check.
% Requires MATLAB and Simulink only. No MPC Toolbox required.
% Plant: x(k+1)=0.8*x(k)+0.2*u(k).
% Cost: (x(k+1)-r)^2 + 0.16*(u(k)-u(k-1))^2.
% Exact scalar constrained optimum: saturate(r-0.8*x+0.8*u_prev).
% Ts=0.1 is the illustrative discrete model's assigned sample time.
% Do not change Ts alone to represent the same continuous physical plant.

outDir = fileparts(mfilename('fullpath'));
mdl = 'mpc_scalar_sat_delay_demo';
% Preserve any existing loaded model or saved result by using a fresh name.
baseName = mdl;
suffix = 1;
while bdIsLoaded(mdl) || exist(fullfile(outDir,[mdl '.slx']),'file')
    mdl = sprintf('%s_%02d',baseName,suffix);
    suffix = suffix+1;
end
load_system('simulink');
new_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','0.1','StopTime','8','ReturnWorkspaceOutputs','on');

add_block('simulink/Sources/Constant',[mdl '/Target_r'], ...
    'Value','1','Position',[35 85 75 115]);
add_block('built-in/Subsystem',[mdl '/MPC_Controller'], ...
    'Position',[190 75 380 180]);
c = [mdl '/MPC_Controller'];
add_block('simulink/Ports & Subsystems/In1',[c '/r'],'Port','1','Position',[25 40 55 60]);
add_block('simulink/Ports & Subsystems/In1',[c '/x'],'Port','2','Position',[25 110 55 130]);
add_block('simulink/Math Operations/Gain',[c '/State_term'], ...
    'Gain','-0.8','Position',[100 100 155 140]);
add_block('simulink/Math Operations/Sum',[c '/Compute_v'], ...
    'Inputs','+++','Position',[230 45 260 125]);
add_block('simulink/Discontinuities/Saturation',[c '/Saturation_0_to_1p2'], ...
    'LowerLimit','0','UpperLimit','1.2','Position',[315 60 415 105]);
add_block('simulink/Discrete/Unit Delay',[c '/Remember_actual_u'], ...
    'SampleTime','0.1','InitialCondition','0','Orientation','left', ...
    'Position',[320 200 385 240]);
add_block('simulink/Math Operations/Gain',[c '/Previous_input_term'], ...
    'Gain','0.8','Orientation','left','Position',[165 200 225 240]);
add_block('simulink/Ports & Subsystems/Out1',[c '/u_actual'], ...
    'Port','1','Position',[485 75 515 95]);
add_block('simulink/Ports & Subsystems/Out1',[c '/v_unconstrained'], ...
    'Port','2','Position',[485 140 515 160]);
add_block('simulink/Ports & Subsystems/Out1',[c '/u_previous'], ...
    'Port','3','Position',[485 275 515 295]);
add_line(c,'r/1','Compute_v/1','autorouting','on');
add_line(c,'x/1','State_term/1','autorouting','on');
add_line(c,'State_term/1','Compute_v/2','autorouting','on');
add_line(c,'Previous_input_term/1','Compute_v/3','autorouting','on');
add_line(c,'Compute_v/1','Saturation_0_to_1p2/1','autorouting','on');
add_line(c,'Compute_v/1','v_unconstrained/1','autorouting','on');
add_line(c,'Saturation_0_to_1p2/1','u_actual/1','autorouting','on');
add_line(c,'Saturation_0_to_1p2/1','Remember_actual_u/1','autorouting','on');
add_line(c,'Remember_actual_u/1','Previous_input_term/1','autorouting','on');
add_line(c,'Remember_actual_u/1','u_previous/1','autorouting','on');
note = Simulink.Annotation(c, ...
    sprintf('Saturation limits CURRENT input.\nUnit Delay stores the APPLIED input for the NEXT sample.\nv = r - 0.8*x + 0.8*u_previous;   u = clip(v,0,1.2)'));
note.Position = [35 335];

add_block('built-in/Subsystem',[mdl '/Discrete_Plant'], ...
    'Position',[490 75 660 145]);
p = [mdl '/Discrete_Plant'];
add_block('simulink/Ports & Subsystems/In1',[p '/u'],'Position',[25 65 55 85]);
add_block('simulink/Math Operations/Gain',[p '/Input_gain'], ...
    'Gain','0.2','Position',[90 55 145 95]);
add_block('simulink/Math Operations/Sum',[p '/Next_state'], ...
    'Inputs','++','Position',[205 60 235 110]);
add_block('simulink/Discrete/Unit Delay',[p '/State_memory'], ...
    'SampleTime','0.1','InitialCondition','0','Position',[295 65 355 105]);
add_block('simulink/Math Operations/Gain',[p '/State_gain'], ...
    'Gain','0.8','Orientation','left','Position',[215 175 275 215]);
add_block('simulink/Ports & Subsystems/Out1',[p '/x'],'Position',[415 75 445 95]);
add_line(p,'u/1','Input_gain/1','autorouting','on');
add_line(p,'Input_gain/1','Next_state/1','autorouting','on');
add_line(p,'State_gain/1','Next_state/2','autorouting','on');
add_line(p,'Next_state/1','State_memory/1','autorouting','on');
add_line(p,'State_memory/1','x/1','autorouting','on');
add_line(p,'State_memory/1','State_gain/1','autorouting','on');

add_block('simulink/Signal Routing/Mux',[mdl '/Signals_r_x_v_u_uprev'], ...
    'Inputs','5','Position',[760 70 765 250]);
add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[835 80 890 125]);
add_block('simulink/Sinks/To Workspace',[mdl '/Log'], ...
    'VariableName','mpc_trace','SaveFormat','Timeseries','Position',[820 190 915 230]);
add_line(mdl,'Target_r/1','MPC_Controller/1','autorouting','on');
add_line(mdl,'Discrete_Plant/1','MPC_Controller/2','autorouting','on');
add_line(mdl,'MPC_Controller/1','Discrete_Plant/1','autorouting','on');
add_line(mdl,'Target_r/1','Signals_r_x_v_u_uprev/1','autorouting','on');
add_line(mdl,'Discrete_Plant/1','Signals_r_x_v_u_uprev/2','autorouting','on');
add_line(mdl,'MPC_Controller/2','Signals_r_x_v_u_uprev/3','autorouting','on');
add_line(mdl,'MPC_Controller/1','Signals_r_x_v_u_uprev/4','autorouting','on');
add_line(mdl,'MPC_Controller/3','Signals_r_x_v_u_uprev/5','autorouting','on');
add_line(mdl,'Signals_r_x_v_u_uprev/1','Scope/1','autorouting','on');
add_line(mdl,'Signals_r_x_v_u_uprev/1','Log/1','autorouting','on');
note = Simulink.Annotation(mdl, ...
    sprintf('One-step MPC | Ts=0.1 s | lambda=0.16 | 0 <= u <= 1.2\nOpen MPC_Controller to inspect saturation and input memory.\nLogged signals: [reference, state, raw command, applied input, previous input].'));
note.Position = [55 320];
save_system(mdl,fullfile(outDir,[mdl '.slx']));
open_system(mdl);
open_system(c);

%% Verify the saved model by simulation; assertions run on actual Simulink data.
result = sim(mdl);
trace = result.get('mpc_trace');
t = trace.Time;
z = squeeze(trace.Data);
assert(size(z,2)==5,'Unexpected logged signal shape.');
assert(max(abs(z(1:3,2)-[0;0.2;0.4]))<1e-9,'State sequence mismatch.');
assert(max(abs(z(1:3,3)-[1;1.64;1.64]))<1e-9,'Raw input sequence mismatch.');
assert(all(z(:,4)>=-1e-10 & z(:,4)<=1.2+1e-10),'Input constraint violated.');
assert(max(abs(z(2:end,5)-z(1:end-1,4)))<1e-9,'Memory must store applied input.');
assert(abs(z(end,2)-1)<0.02,'Tracking did not settle near the target.');
save(fullfile(outDir,[mdl '_results.mat']),'t','z');
fig = figure('Name','MPC: saturation versus memory','Color','w');
subplot(2,1,1);
stairs(t,z(:,1:2),'LineWidth',1.4); grid on;
ylabel('State'); legend('Target r','State x','Location','best');
subplot(2,1,2);
stairs(t,z(:,3:5),'LineWidth',1.4); grid on;
xlabel('Time (s)'); ylabel('Input');
legend('Raw v','Applied u','Previous u','Location','best');
saveas(fig,fullfile(outDir,[mdl '_response.png']));
disp('PASS: state sequence, saturation, applied-input memory and final tracking.');
disp(fullfile(outDir,[mdl '.slx']));
