function build_beginner_kalman
% Beginner scalar Kalman filter built entirely from elementary blocks.
folder=fileparts(mfilename('fullpath'));
mdl='kalman_beginner_temperature';
load_system('simulink');
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','0.1','StopTime','30','ReturnWorkspaceOutputs','on');

% True temperature and sensor (the only subsystem).
s=[mdl '/真实温度与传感器'];
add_block('built-in/Subsystem',s,'Position',[50 75 235 145]);
add_block('simulink/Discrete/Unit Delay',[s '/当前真实温度'], ...
    'InitialCondition','20','SampleTime','0.1','Position',[70 65 160 105]);
add_block('simulink/Sources/Random Number',[s '/温度自然变化w'], ...
    'Mean','0','Variance','0.01','Seed','1234','SampleTime','0.1', ...
    'Position',[60 180 160 220]);
add_block('simulink/Math Operations/Sum',[s '/下一步温度'], ...
    'Inputs','++','Position',[225 100 250 150]);
add_block('simulink/Sources/Random Number',[s '/测量噪声v'], ...
    'Mean','0','Variance','1','Seed','5678','SampleTime','0.1', ...
    'Position',[220 250 320 290]);
add_block('simulink/Math Operations/Sum',[s '/带噪测量'], ...
    'Inputs','++','Position',[375 160 400 210]);
add_block('simulink/Sinks/Out1',[s '/真实值'],'Port','1','Position',[450 70 480 90]);
add_block('simulink/Sinks/Out1',[s '/测量值'],'Port','2','Position',[450 175 480 195]);
ln(s,'当前真实温度/1','下一步温度/1'); ln(s,'温度自然变化w/1','下一步温度/2');
ln(s,'下一步温度/1','当前真实温度/1');
ln(s,'当前真实温度/1','真实值/1'); ln(s,'当前真实温度/1','带噪测量/1');
ln(s,'测量噪声v/1','带噪测量/2'); ln(s,'带噪测量/1','测量值/1');
note(s,'x(k+1) = x(k) + w(k),   z(k) = x(k) + v(k)',[40 10 500 40]);

% Upper row: estimate the temperature.
blk('Discrete/Unit Delay','上一拍估计',[50 270 160 310], ...
    'InitialCondition','18','SampleTime','0.1');
blk('Math Operations/Gain','预测温度',[265 270 355 310],'Gain','1');
blk('Math Operations/Sum','测量减预测',[480 260 510 320],'Inputs','|+-');
blk('Math Operations/Product','增益乘偏差',[655 265 710 315],'Inputs','**');
blk('Math Operations/Sum','校正温度',[845 260 875 320],'Inputs','|++');
blk('Sinks/Scope','温度对比_真实_测量_估计',[1080 75 1140 165],'NumInputPorts','3');

% Lower row: calculate how much to trust the measurement.
blk('Discrete/Unit Delay','上一拍不确定度P',[50 520 160 560], ...
    'InitialCondition','4','SampleTime','0.1');
blk('Sources/Constant','过程方差Q',[170 610 240 640],'Value','0.01');
blk('Math Operations/Sum','预测不确定度',[285 510 315 570],'Inputs','|++');
blk('Sources/Constant','测量方差R',[365 610 435 640],'Value','1');
blk('Math Operations/Sum','预测方差加R',[480 510 510 570],'Inputs','|++');
blk('Math Operations/Divide','计算卡尔曼增益K',[650 510 705 570],'Inputs','*/');
blk('Sources/Constant','常数1',[750 610 810 640],'Value','1');
blk('Math Operations/Sum','一减K',[845 510 875 570],'Inputs','|+-');
blk('Math Operations/Product','校正不确定度',[990 510 1045 570],'Inputs','**');
blk('Sinks/Scope','增益K与不确定度P',[1130 480 1190 560],'NumInputPorts','2');

ln(mdl,'真实温度与传感器/1','温度对比_真实_测量_估计/1');
ln(mdl,'真实温度与传感器/2','温度对比_真实_测量_估计/2');
ln(mdl,'真实温度与传感器/2','测量减预测/1','测量z');
ln(mdl,'上一拍估计/1','预测温度/1');
ln(mdl,'预测温度/1','测量减预测/2','预测值x_minus');
ln(mdl,'预测温度/1','校正温度/1');
ln(mdl,'测量减预测/1','增益乘偏差/1','偏差z_minus_x');
ln(mdl,'计算卡尔曼增益K/1','增益乘偏差/2','增益K');
ln(mdl,'增益乘偏差/1','校正温度/2');
ln(mdl,'校正温度/1','温度对比_真实_测量_估计/3','估计x_hat');
ln(mdl,'校正温度/1','上一拍估计/1');
ln(mdl,'上一拍不确定度P/1','预测不确定度/1');
ln(mdl,'过程方差Q/1','预测不确定度/2');
ln(mdl,'预测不确定度/1','预测方差加R/1','预测方差P_minus');
ln(mdl,'测量方差R/1','预测方差加R/2');
ln(mdl,'预测不确定度/1','计算卡尔曼增益K/1');
ln(mdl,'预测方差加R/1','计算卡尔曼增益K/2');
ln(mdl,'常数1/1','一减K/1'); ln(mdl,'计算卡尔曼增益K/1','一减K/2');
ln(mdl,'一减K/1','校正不确定度/1');
ln(mdl,'预测不确定度/1','校正不确定度/2');
ln(mdl,'校正不确定度/1','上一拍不确定度P/1','校正方差P');
ln(mdl,'计算卡尔曼增益K/1','增益K与不确定度P/1');
ln(mdl,'校正不确定度/1','增益K与不确定度P/2');

note(mdl,'入门：用有噪声的温度测量，估计真实温度（所有滤波公式均用基础模块实现）',[50 10 1130 40]);
note(mdl,'① 预测：温度先沿用上一拍估计     x_minus = x_hat_prev',[50 205 450 235]);
note(mdl,'③ 校正：预测值 + K × (测量值 - 预测值)',[640 205 1100 235]);
note(mdl,'② 算信任程度：P_minus = P_prev + Q；K = P_minus / (P_minus + R)',[50 435 820 465]);
note(mdl,'④ 更新不确定度：P = (1-K) × P_minus',[830 435 1220 465]);
note(mdl,'Unit Delay 保存上一拍结果。Q、R 是方差，不是标准差。Ts=0.1 s。',[50 710 1050 740]);

sources={'真实温度与传感器/1','真实温度与传感器/2','校正温度/1','计算卡尔曼增益K/1','校正不确定度/1'};
vars={'truth_log','z_log','estimate_log','K_log','P_log'};
for j=1:numel(vars)
    blk('Sinks/To Workspace',vars{j},[1260 80+100*j 1370 110+100*j], ...
        'VariableName',vars{j},'SaveFormat','Timeseries');
    ln(mdl,sources{j},[vars{j} '/1']);
end
set_param(mdl,'Location',[30 40 1530 900]);
save_system(mdl,fullfile(folder,[mdl '.slx']));
result=sim(mdl);
t=result.truth_log.Time; truth=result.truth_log.Data; z=result.z_log.Data;
estimate=result.estimate_log.Data; K=result.K_log.Data; P=result.P_log.Data;
% Independent scalar verification, including both feedback delays.
xprev=18; Pprev=4; ref=zeros(size(z)); refK=ref; refP=ref;
for j=1:numel(z)
    xp=xprev; Pp=Pprev+.01; kg=Pp/(Pp+1);
    xprev=xp+kg*(z(j)-xp); Pprev=(1-kg)*Pp;
    ref(j)=xprev; refK(j)=kg; refP(j)=Pprev;
end
maxError=max(abs(ref-estimate));
assert(maxError<1e-10 && max(abs(refK-K))<1e-10 && max(abs(refP-P))<1e-10);
assert(all(P>0) && all(K>0 & K<1));
keep=t>=2;
measurementRMSE=sqrt(mean((z(keep)-truth(keep)).^2));
filterRMSE=sqrt(mean((estimate(keep)-truth(keep)).^2));
assert(filterRMSE<measurementRMSE);
save(fullfile(folder,'beginner_results.mat'),'t','truth','z','estimate','K','P', ...
    'measurementRMSE','filterRMSE','maxError');
f=figure('Visible','off','Color','w','Position',[50 50 1050 750]);
tiledlayout(3,1);
nexttile; plot(t,z,'Color',[.75 .75 .75]); hold on;
plot(t,truth,'k',t,estimate,'r','LineWidth',1.5); grid on;
ylabel('Temperature'); legend('Measurement','True','Estimate');
title('Beginner scalar Kalman filter');
nexttile; plot(t,z-truth,'Color',[.75 .75 .75]); hold on;
plot(t,estimate-truth,'r','LineWidth',1.3); grid on;
ylabel('Error'); legend('Measurement error','Estimate error');
nexttile; plot(t,K,'b',t,P,'--r','LineWidth',1.3); grid on;
xlabel('Time (s)'); legend('Kalman gain K','Posterior variance P');
exportgraphics(f,fullfile(folder,'beginner_response.png'),'Resolution',150); close(f);
print(['-s' mdl],'-dpng','-r100',fullfile(folder,'beginner_model.png'));
fprintf('Measurement RMSE (t>=2s): %.6f\nFilter RMSE (t>=2s): %.6f\n',measurementRMSE,filterRMSE);
fprintf('Independent recursion maximum error: %.3g\n',maxError);
fprintf('First update: Pminus=4.01, K=%.6f, z=%.6f, xhat=%.6f\n',K(1),z(1),estimate(1));
fprintf('Model: %s\n',fullfile(folder,[mdl '.slx']));
close_system(mdl,0);

    function blk(lib,name,pos,varargin)
        add_block(['simulink/' lib],[mdl '/' name],'Position',pos,varargin{:});
    end
end
function ln(sys,src,dst,label)
h=add_line(sys,src,dst,'autorouting','on');
if nargin==4, set_param(h,'Name',label); end
end
function note(sys,txt,pos)
n=Simulink.Annotation(sys,txt); n.Position=pos;
end
