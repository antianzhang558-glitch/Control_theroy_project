function build_beginner_kalman_2d
% Two states, one measured quantity; every equation uses elementary blocks.
folder=fileparts(mfilename('fullpath')); mdl='kalman_beginner_2d';
load_system('simulink');
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','0.1','StopTime','30','ReturnWorkspaceOutputs','on');
A=[1 .1;0 1]; H=[1 0]; Q=diag([.001 .0001]); R=.25;
% All matrices are stored with the model, so opening the SLX is sufficient.
mw=get_param(mdl,'ModelWorkspace');
assignin(mw,'A',A); assignin(mw,'H',H); assignin(mw,'Q',Q); assignin(mw,'R',R);

s=[mdl '/真实温度与传感器']; sub(s,[40 70 225 150]);
b(s,'Sources/Random Number','真实过程噪声',[30 180 140 220], ...
    'Mean','[0;0]','Variance','[0.001;0.0001]','Seed','[1234;4321]','SampleTime','0.1');
b(s,'Discrete/Discrete State-Space','温度与变化速度',[190 75 320 135], ...
    'A','A','B','eye(2)','C','eye(2)','D','zeros(2)', ...
    'InitialCondition','[20;0.1]','SampleTime','0.1');
gain(s,'只测温度H',[365 150 445 190],'H','Matrix(K*u)');
b(s,'Sources/Random Number','温度测量噪声',[340 260 450 300], ...
    'Mean','0','Variance','R','Seed','5678','SampleTime','0.1');
b(s,'Math Operations/Sum','测量温度z',[500 150 530 210],'Inputs','|++');
port(s,'Out1','真实状态',1,[580 75 610 95]); port(s,'Out1','测量z',2,[580 165 610 185]);
L(s,'真实过程噪声/1','温度与变化速度/1'); L(s,'温度与变化速度/1','真实状态/1');
L(s,'温度与变化速度/1','只测温度H/1'); L(s,'只测温度H/1','测量温度z/1');
L(s,'温度测量噪声/1','测量温度z/2'); L(s,'测量温度z/1','测量z/1');
note(s,'x=[温度;变化速度]，x_next=A*x+w；z=H*x+v；H=[1 0]',[30 10 660 40]);

s=[mdl '/1 预测']; sub(s,[280 270 460 365]);
port(s,'In1','上拍估计',1,[30 75 60 95]); port(s,'In1','上拍P',2,[30 225 60 245]);
gain(s,'预测状态A',[135 65 245 105],'A','Matrix(K*u)');
gain(s,'左乘A',[120 215 210 255],'A','Matrix(K*u)');
gain(s,'右乘A转置',[270 215 390 255],"A'",'Matrix(u*K)');
b(s,'Sources/Constant','过程协方差Q',[300 320 390 350],'Value','Q');
b(s,'Math Operations/Sum','预测协方差',[450 220 480 280],'Inputs','|++');
port(s,'Out1','预测状态',1,[540 75 570 95]); port(s,'Out1','预测P',2,[540 235 570 255]);
L(s,'上拍估计/1','预测状态A/1'); L(s,'预测状态A/1','预测状态/1');
L(s,'上拍P/1','左乘A/1'); L(s,'左乘A/1','右乘A转置/1');
L(s,'右乘A转置/1','预测协方差/1'); L(s,'过程协方差Q/1','预测协方差/2');
L(s,'预测协方差/1','预测P/1');
note(s,"x_minus=A*x_prev；P_minus=A*P_prev*A'+Q",[30 10 610 40]);

s=[mdl '/2 计算增益K']; sub(s,[530 345 700 420]);
port(s,'In1','预测P',1,[30 100 60 120]);
gain(s,'右乘H转置',[120 90 230 130],"H'",'Matrix(u*K)');
gain(s,'左乘H',[300 185 390 225],'H','Matrix(K*u)');
b(s,'Sources/Constant','测量方差R',[300 285 390 315],'Value','R');
b(s,'Math Operations/Sum','新息方差S',[450 190 480 250],'Inputs','|++');
b(s,'Math Operations/Divide','除以标量S',[530 95 590 155],'Inputs','*/');
port(s,'Out1','增益K',1,[650 110 680 130]);
L(s,'预测P/1','右乘H转置/1'); L(s,'右乘H转置/1','除以标量S/1');
L(s,'右乘H转置/1','左乘H/1'); L(s,'左乘H/1','新息方差S/1');
L(s,'测量方差R/1','新息方差S/2'); L(s,'新息方差S/1','除以标量S/2');
L(s,'除以标量S/1','增益K/1');
note(s,"K=P_minus*H'/(H*P_minus*H'+R)；K为2×1，分母S为标量",[30 10 730 40]);

s=[mdl '/3 测量校正']; sub(s,[790 240 990 390]);
port(s,'In1','测量z',1,[30 80 60 100]); port(s,'In1','预测状态',2,[30 165 60 185]);
port(s,'In1','预测P',3,[30 380 60 400]); port(s,'In1','增益K',4,[30 285 60 305]);
gain(s,'预测测量H',[130 155 230 195],'H','Matrix(K*u)');
b(s,'Math Operations/Sum','测量减预测',[290 75 320 135],'Inputs','|+-');
b(s,'Math Operations/Product','K乘测量偏差',[395 90 455 150],'Inputs','**','Multiplication','Matrix(*)');
b(s,'Math Operations/Sum','校正状态',[545 90 575 150],'Inputs','|++');
gain(s,'K乘H',[150 275 250 315],'H','Matrix(u*K)');
b(s,'Sources/Constant','单位矩阵I',[160 440 250 470],'Value','eye(2)');
b(s,'Math Operations/Sum','I减KH',[330 290 360 350],'Inputs','|+-');
b(s,'Math Operations/Product','校正协方差',[460 310 520 370],'Inputs','**','Multiplication','Matrix(*)');
port(s,'Out1','估计状态',1,[660 105 690 125]); port(s,'Out1','校正P',2,[660 325 690 345]);
L(s,'测量z/1','测量减预测/1'); L(s,'预测状态/1','预测测量H/1');
L(s,'预测测量H/1','测量减预测/2'); L(s,'增益K/1','K乘测量偏差/1');
L(s,'测量减预测/1','K乘测量偏差/2'); L(s,'预测状态/1','校正状态/1');
L(s,'K乘测量偏差/1','校正状态/2'); L(s,'校正状态/1','估计状态/1');
L(s,'增益K/1','K乘H/1'); L(s,'单位矩阵I/1','I减KH/1');
L(s,'K乘H/1','I减KH/2'); L(s,'I减KH/1','校正协方差/1');
L(s,'预测P/1','校正协方差/2'); L(s,'校正协方差/1','校正P/1');
note(s,'x_hat=x_minus+K*(z-H*x_minus)；P=(I-K*H)*P_minus',[30 10 730 40]);
note(s,'本教学模型使用标准协方差更新；Product采用矩阵乘法。',[30 510 730 540]);

% Top-level memory and comparison scopes.
b(mdl,'Discrete/Unit Delay','上一拍估计',[55 275 190 315], ...
    'InitialCondition','[18;0]','SampleTime','0.1');
b(mdl,'Discrete/Unit Delay','上一拍P',[55 420 190 460], ...
    'InitialCondition','diag([4 1])','SampleTime','0.1');
b(mdl,'Signal Routing/Demux','真实状态拆开',[1040 65 1045 145],'Outputs','2');
b(mdl,'Signal Routing/Demux','估计状态拆开',[1040 250 1045 330],'Outputs','2');
b(mdl,'Sinks/Scope','温度_真实_测量_估计',[1185 60 1240 145],'NumInputPorts','3');
b(mdl,'Sinks/Scope','变化速度_真实_估计',[1185 240 1240 310],'NumInputPorts','2');
b(mdl,'Sinks/Scope','两个卡尔曼增益',[815 490 875 535]);
edges={
    '真实温度与传感器/1','真实状态拆开/1';
    '真实温度与传感器/2','3 测量校正/1';
    '真实温度与传感器/2','温度_真实_测量_估计/2';
    '上一拍估计/1','1 预测/1'; '上一拍P/1','1 预测/2';
    '1 预测/1','3 测量校正/2'; '1 预测/2','2 计算增益K/1';
    '1 预测/2','3 测量校正/3'; '2 计算增益K/1','3 测量校正/4';
    '2 计算增益K/1','两个卡尔曼增益/1';
    '3 测量校正/1','上一拍估计/1'; '3 测量校正/2','上一拍P/1';
    '3 测量校正/1','估计状态拆开/1';
    '真实状态拆开/1','温度_真实_测量_估计/1';
    '估计状态拆开/1','温度_真实_测量_估计/3';
    '真实状态拆开/2','变化速度_真实_估计/1';
    '估计状态拆开/2','变化速度_真实_估计/2'};
for j=1:size(edges,1), L(mdl,edges{j,1},edges{j,2}); end
vars={'truth_log','z_log','estimate_log','K_log','P_log'};
src={'真实温度与传感器/1','真实温度与传感器/2','3 测量校正/1','2 计算增益K/1','3 测量校正/2'};
for j=1:numel(vars)
    b(mdl,'Sinks/To Workspace',vars{j},[1340 70+j*90 1450 100+j*90], ...
        'VariableName',vars{j},'SaveFormat','Timeseries');
    L(mdl,src{j},[vars{j} '/1']);
end
note(mdl,'二维入门：x=[温度;温度变化速度]，传感器只测温度。双击1、2、3查看基础模块。',[40 5 1270 40]);
note(mdl,'A=[1 0.1;0 1]，H=[1 0]，Q=diag([0.001 0.0001])，R=0.25，Ts=0.1 s',[40 585 1270 620]);
note(mdl,'上拍估计是2×1，P是2×2，K是2×1；两条延迟反馈保存上一拍的结果。',[40 635 1270 665]);
set_param(mdl,'Location',[20 30 1530 850]);
save_system(mdl,fullfile(folder,[mdl '.slx']));
result=sim(mdl);
t=result.truth_log.Time; truth=squeeze(result.truth_log.Data); z=result.z_log.Data;
estimate=squeeze(result.estimate_log.Data); K=squeeze(result.K_log.Data); P=squeeze(result.P_log.Data);
if size(truth,1)~=numel(t), truth=truth'; end
if size(estimate,1)~=numel(t), estimate=estimate'; end
xprev=[18;0]; Pprev=diag([4 1]); ref=zeros(numel(t),2); covarianceError=0;
for j=1:numel(t)
    xp=A*xprev; Pp=A*Pprev*A'+Q;
    kg=Pp*H'/(H*Pp*H'+R);
    xprev=xp+kg*(z(j)-H*xp); Pprev=(eye(2)-kg*H)*Pp;
    ref(j,:)=xprev';
    covarianceError=max(covarianceError,max(abs(P(:,:,j)-Pprev),[],'all'));
    assert(norm(Pprev-Pprev','fro')<1e-10 && min(eig(Pprev))>0);
end
maxError=max(abs(ref-estimate),[],'all'); assert(maxError<1e-9 && covarianceError<1e-9);
assert(rank([H;H*A])==2,'Temperature rate must be observable from temperature samples.');
keep=t>=2; measurementRMSE=sqrt(mean((z(keep)-truth(keep,1)).^2));
temperatureRMSE=sqrt(mean((estimate(keep,1)-truth(keep,1)).^2));
rateRMSE=sqrt(mean((estimate(keep,2)-truth(keep,2)).^2));
assert(temperatureRMSE<measurementRMSE);
save(fullfile(folder,'beginner_2d_results.mat'),'t','truth','z','estimate','K','P', ...
    'measurementRMSE','temperatureRMSE','rateRMSE','maxError','covarianceError');
f=figure('Visible','off','Color','w','Position',[50 50 1050 850]); tiledlayout(3,1);
nexttile; plot(t,z,'.','Color',[.7 .7 .7]); hold on;
plot(t,truth(:,1),'k',t,estimate(:,1),'r','LineWidth',1.4); grid on;
ylabel('Temperature'); legend('Measurement','True','Estimate'); title('Two-state beginner Kalman filter');
nexttile; plot(t,truth(:,2),'k',t,estimate(:,2),'r','LineWidth',1.4); grid on;
ylabel('Temperature rate / s'); legend('True','Estimate');
nexttile; plot(t,z-truth(:,1),'Color',[.7 .7 .7]); hold on;
plot(t,estimate(:,1)-truth(:,1),'r','LineWidth',1.3); grid on;
xlabel('Time (s)'); ylabel('Temperature error'); legend('Measurement error','Estimate error');
exportgraphics(f,fullfile(folder,'beginner_2d_response.png'),'Resolution',150); close(f);
print(['-s' mdl],'-dpng','-r100',fullfile(folder,'beginner_2d_model.png'));
for j=1:3
    names={'1 预测','2 计算增益K','3 测量校正'};
    print(['-s' mdl '/' names{j}],'-dpng','-r100',fullfile(folder,sprintf('beginner_2d_step%d.png',j)));
end
fprintf('Measurement temperature RMSE (t>=2s): %.6f\n',measurementRMSE);
fprintf('Filtered temperature RMSE (t>=2s): %.6f\n',temperatureRMSE);
fprintf('Temperature rate RMSE (t>=2s): %.6f\n',rateRMSE);
fprintf('Reference state/covariance errors: %.3g / %.3g\n',maxError,covarianceError);
fprintf('Model saved: %s\n',fullfile(folder,[mdl '.slx'])); close_system(mdl,0);
end
function b(s,lib,name,pos,varargin)
add_block(['simulink/' lib],[s '/' name],'Position',pos,varargin{:});
end
function gain(s,name,pos,value,mult)
b(s,'Math Operations/Gain',name,pos,'Gain',value,'Multiplication',mult);
end
function port(s,type,name,n,pos)
if strcmp(type,'In1'), cat='Sources/'; else, cat='Sinks/'; end
b(s,[cat type],name,pos,'Port',num2str(n));
end
function sub(s,pos)
add_block('built-in/Subsystem',s,'Position',pos);
end
function L(s,src,dst)
add_line(s,src,dst,'autorouting','on');
end
function note(s,txt,pos)
n=Simulink.Annotation(s,txt); n.Position=pos;
end
