%% Build a self-contained position/velocity Kalman filter model.
outDir = fileparts(mfilename('fullpath'));
mdl = 'kalman_position_velocity';
load_system('simulink');
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','0.1','StopTime','20','ReturnWorkspaceOutputs','on');

% Plant: x(k+1)=A*x(k)+G*a(k), z(k)=H*x(k)+v(k).
add_block('simulink/Sources/Random Number',[mdl '/Acceleration noise'], ...
    'Mean','0','Variance','0.04','Seed','1234','SampleTime','0.1', ...
    'Position',[30 135 120 175]);
add_block('simulink/Discrete/Discrete State-Space',[mdl '/True motion'], ...
    'A','[1 0.1;0 1]','B','[0.005;0.1]','C','eye(2)','D','zeros(2,1)', ...
    'InitialCondition','[0;1]','SampleTime','0.1', ...
    'Position',[175 130 300 185]);
add_block('simulink/Signal Routing/Demux',[mdl '/True states'], ...
    'Outputs','2','Position',[345 120 350 195]);
add_block('simulink/Sources/Random Number',[mdl '/Sensor noise'], ...
    'Mean','0','Variance','0.25','Seed','5678','SampleTime','0.1', ...
    'Position',[200 255 300 295]);
add_block('simulink/Math Operations/Sum',[mdl '/Measured position'], ...
    'Inputs','++','Position',[410 125 435 160]);

% The delays hold the prior for the CURRENT sample; no algebraic loop.
sub = [mdl '/Kalman Filter'];
add_block('built-in/Subsystem',sub,'Position',[505 120 670 210]);
add_block('simulink/Sources/In1',[sub '/z'],'Position',[25 105 55 125]);
add_block('simulink/User-Defined Functions/MATLAB Function',[sub '/Correct and predict'], ...
    'Position',[210 85 370 190]);
rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','Path',[sub '/Correct and predict']);
chart.Script = fileread(fullfile(outDir,'kf_step.m'));
add_block('simulink/Discrete/Unit Delay',[sub '/Prior state'], ...
    'InitialCondition','[0;0]','SampleTime','0.1', ...
    'Position',[95 240 165 275],'Orientation','left');
add_block('simulink/Discrete/Unit Delay',[sub '/Prior covariance'], ...
    'InitialCondition','eye(2)','SampleTime','0.1', ...
    'Position',[250 310 335 345],'Orientation','left');
add_block('simulink/Sinks/Out1',[sub '/xhat'],'Port','1','Position',[450 90 480 110]);
add_block('simulink/Sinks/Out1',[sub '/Gain K'],'Port','2','Position',[450 190 480 210]);
add_line(sub,'z/1','Correct and predict/1','autorouting','on');
add_line(sub,'Prior state/1','Correct and predict/2','autorouting','on');
add_line(sub,'Prior covariance/1','Correct and predict/3','autorouting','on');
add_line(sub,'Correct and predict/1','xhat/1','autorouting','on');
add_line(sub,'Correct and predict/2','Prior state/1','autorouting','on');
add_line(sub,'Correct and predict/3','Prior covariance/1','autorouting','on');
add_line(sub,'Correct and predict/4','Gain K/1','autorouting','on');
note = Simulink.Annotation(sub,sprintf('Measurement update -> time prediction\nDouble-click Correct and predict to inspect equations.\nPrior: xhat = [0;0], P = I'));
note.Position = [30 20 470 65];

add_block('simulink/Signal Routing/Demux',[mdl '/Estimated states'], ...
    'Outputs','2','Position',[720 120 725 195]);
add_block('simulink/Sinks/Scope',[mdl '/Position - true measured estimated'], ...
    'NumInputPorts','3','Position',[850 65 900 145]);
add_block('simulink/Sinks/Scope',[mdl '/Velocity - true estimated'], ...
    'NumInputPorts','2','Position',[850 200 900 260]);
add_block('simulink/Sinks/Scope',[mdl '/Kalman gain'], ...
    'Position',[720 295 770 335]);

sources = {'True motion/1','Measured position/1','Kalman Filter/1','Kalman Filter/2'};
names = {'true_log','measurement_log','estimate_log','gain_log'};
for i=1:4
    add_block('simulink/Sinks/To Workspace',[mdl '/' names{i}], ...
        'VariableName',names{i},'SaveFormat','Timeseries', ...
        'Position',[180+(i-1)*175 400 310+(i-1)*175 430]);
end
edges = {
    'Acceleration noise/1','True motion/1';
    'True motion/1','True states/1';
    'True states/1','Measured position/1';
    'Sensor noise/1','Measured position/2';
    'Measured position/1','Kalman Filter/1';
    'Kalman Filter/1','Estimated states/1';
    'True states/1','Position - true measured estimated/1';
    'Measured position/1','Position - true measured estimated/2';
    'Estimated states/1','Position - true measured estimated/3';
    'True states/2','Velocity - true estimated/1';
    'Estimated states/2','Velocity - true estimated/2';
    'Kalman Filter/2','Kalman gain/1'};
for i=1:size(edges,1)
    add_line(mdl,edges{i,1},edges{i,2},'autorouting','on');
end
for i=1:4
    add_line(mdl,sources{i},[names{i} '/1'],'autorouting','on');
end
note = Simulink.Annotation(mdl,sprintf(['POSITION / VELOCITY KALMAN FILTER\n' ...
    'Ts = 0.1 s; acceleration variance = 0.04; sensor variance R = 0.25\n' ...
    'Only noisy POSITION enters the filter. Initial true velocity = 1 m/s.']));
note.Position = [30 5 790 55];
set_param(mdl,'Location',[50 50 1100 650]);
save_system(mdl,fullfile(outDir,[mdl '.slx']));

%% Simulate and independently verify the discrete recursion.
result = sim(mdl);
t = result.true_log.Time;
truth = squeeze(result.true_log.Data);
estimate = squeeze(result.estimate_log.Data);
z = result.measurement_log.Data;
gain = squeeze(result.gain_log.Data);
if size(truth,1)~=numel(t), truth=truth'; end
if size(estimate,1)~=numel(t), estimate=estimate'; end
A=[1 .1;0 1]; H=[1 0]; G=[.005;.1]; Q=.04*(G*G'); R=.25;
xp=zeros(2,1); P=eye(2); reference=zeros(numel(t),2);
for i=1:numel(t)
    Kg=P*H'/(H*P*H'+R);
    xc=xp+Kg*(z(i)-H*xp);
    reference(i,:)=xc';
    J=eye(2)-Kg*H;
    Pc=J*P*J'+Kg*R*Kg';
    xp=A*xc; P=A*Pc*A'+Q;
    assert(min(eig(P))>0,'Covariance must stay positive definite.');
end
recursionError=max(abs(reference-estimate),[],'all');
assert(recursionError<1e-9,'Simulink result differs from reference recursion.');
keep=t>=2;
measurementRMSE=sqrt(mean((z(keep)-truth(keep,1)).^2));
positionRMSE=sqrt(mean((estimate(keep,1)-truth(keep,1)).^2));
velocityRMSE=sqrt(mean((estimate(keep,2)-truth(keep,2)).^2));
assert(positionRMSE<measurementRMSE,'Filter should improve position RMSE.');
save(fullfile(outDir,'kalman_results.mat'),'t','truth','z','estimate','gain', ...
    'measurementRMSE','positionRMSE','velocityRMSE','recursionError');
f=figure('Visible','off','Color','w','Position',[50 50 1050 850]);
tiledlayout(3,1);
nexttile; plot(t,z,'.','Color',[.7 .7 .7]); hold on;
plot(t,truth(:,1),'k','LineWidth',1.5); plot(t,estimate(:,1),'r','LineWidth',1.3);
grid on; ylabel('Position (m)'); legend('Measurement','True','Kalman','Location','northwest');
title('Kalman filter: estimate position and velocity from noisy position');
nexttile; plot(t,truth(:,2),'k',t,estimate(:,2),'r','LineWidth',1.3);
grid on; ylabel('Velocity (m/s)'); legend('True','Kalman');
nexttile; plot(t,z-truth(:,1),'Color',[.7 .7 .7]); hold on;
plot(t,estimate(:,1)-truth(:,1),'r','LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Position error (m)'); legend('Measurement error','Kalman error');
exportgraphics(f,fullfile(outDir,'kalman_response.png'),'Resolution',150); close(f);
fprintf('Measurement position RMSE (t>=2s): %.6f m\n',measurementRMSE);
fprintf('Kalman position RMSE (t>=2s): %.6f m\n',positionRMSE);
fprintf('Kalman velocity RMSE (t>=2s): %.6f m/s\n',velocityRMSE);
fprintf('Maximum reference recursion error: %.3g\n',recursionError);
fprintf('Model saved: %s\n',fullfile(outDir,[mdl '.slx']));
close_system(mdl,0);
