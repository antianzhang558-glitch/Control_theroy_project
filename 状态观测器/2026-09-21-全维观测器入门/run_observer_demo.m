function run_observer_demo
% 全维观测器入门：只测位置，同时估计位置与速度。
% MATLAB + Simulink；不需要 Control System Toolbox。
folder = fileparts(mfilename('fullpath'));
A = [0 1; -2 -3]; B = [0;1]; C = [1 0];
K = [4 2]; L = [8;4]; N = 6;
x0 = [0.5;0]; xhat0 = [0;0];
assert(rank([B A*B]) == 2 && rank([C;C*A]) == 2);
assert(max(abs(sort(eig(A-B*K))-[-3;-2])) < 1e-10);
assert(max(abs(sort(eig(A-L*C))-[-6;-5])) < 1e-10);
model = 'full_order_observer_intro';
modelFile = fullfile(folder,[model '.slx']);
if ~isfile(modelFile)
    build_model(model,modelFile);
end
load_system(modelFile);
out = sim(model,'ReturnWorkspaceOutputs','on');
s = out.get('states'); t = s.time;
z = s.signals.values; x = z(:,1:2); xhat = z(:,3:4);
e = x-xhat; u = N-xhat*K';
% 独立验证：用增广闭环矩阵的精确解检查 Simulink 接线。
M = [A -B*K; L*C A-B*K-L*C];
g = [B*N;B*N]; zss = -M\g; z0 = [x0;xhat0];
exact = zeros(size(z)); expectedError = zeros(size(e));
for k = 1:numel(t)
    exact(k,:) = (zss+expm(M*t(k))*(z0-zss))';
    expectedError(k,:) = (expm((A-L*C)*t(k))*(x0-xhat0))';
end
stateError = max(abs(z-exact),[],'all');
errorCheck = max(abs(e-expectedError),[],'all');
assert(stateError < 1e-5,'模型与默认设计的精确解不一致。');
assert(errorCheck < 1e-5,'观测误差动态不符合 e_dot=(A-LC)e。');
assert(norm(e(end,:)) < 1e-5 && abs(x(end,1)-1) < 1e-4);
f = figure('Name','全维观测器：位置测量与估计状态反馈','Color','w','Position',[80 80 1100 720]);
tiledlayout(2,2);
nexttile; plot(t,x(:,1),'LineWidth',1.6); hold on;
plot(t,xhat(:,1),'--','LineWidth',1.6); yline(1,':'); grid on;
title('位置：真实值与估计值'); xlabel('时间 / s'); ylabel('位置 / m'); legend('x_1','估计 x_1','目标','Location','best');
nexttile; plot(t,x(:,2),'LineWidth',1.6); hold on;
plot(t,xhat(:,2),'--','LineWidth',1.6); grid on;
title('速度：无需速度传感器'); xlabel('时间 / s'); ylabel('速度 / (m/s)'); legend('x_2','估计 x_2','Location','best');
nexttile; plot(t,e,'LineWidth',1.6); grid on;
title('初值不一致，估计误差仍收敛'); xlabel('时间 / s'); ylabel('各状态的估计误差'); legend('e_1 / m','e_2 / (m/s)');
nexttile; plot(t,u,'LineWidth',1.6); grid on;
title('控制输入 u = 6r - [4 2] xhat'); xlabel('时间 / s'); ylabel('力 / N');
exportgraphics(f,fullfile(folder,'observer_response.png'),'Resolution',160);
save(fullfile(folder,'observer_results.mat'),'t','x','xhat','e','u','A','B','C','K','L','N');
fid = fopen(fullfile(folder,'validation.txt'),'w');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'MATLAB %s\nController poles: -2, -3\nObserver poles: -5, -6\n',version);
fprintf(fid,'Max state error vs exact solution: %.3g\nMax observer error residual: %.3g\nFinal position: %.9f\nFinal estimation error norm: %.3g\nPASS\n',stateError,errorCheck,x(end,1),norm(e(end,:)));
fprintf('PASS: final position %.9f, estimation error %.3g\n',x(end,1),norm(e(end,:)));
if usejava('desktop'), open_system(model); end
end

function build_model(m,modelFile)
% 数值全部保存在模块中，直接打开 slx 即可运行。
new_system(m);
set_param(m,'StopTime','8','Solver','ode45','RelTol','1e-8','AbsTol','1e-10','MaxStep','0.01');
add_block('simulink/Sources/Constant',[m '/Reference'],'Value','1','Position',[25 60 55 90]);
add_block('simulink/Math Operations/Gain',[m '/N'],'Gain','6','Position',[90 60 130 90]);
add_block('simulink/Math Operations/Sum',[m '/ControlSum'],'Inputs','+-','Position',[180 60 205 95]);
add_block('simulink/Continuous/State-Space',[m '/Plant'],'A','[0 1;-2 -3]','B','[0;1]','C','eye(2)','D','zeros(2,1)','X0','[0.5;0]','Position',[280 50 405 105]);
add_block('simulink/Math Operations/Gain',[m '/PositionSensor'],'Gain','[1 0]','Multiplication','Matrix(K*u)','Position',[450 55 540 100]);
add_block('simulink/Math Operations/Sum',[m '/Innovation'],'Inputs','+-','Position',[590 180 620 215]);
add_block('simulink/Math Operations/Gain',[m '/L'],'Gain','[8;4]','Multiplication','Matrix(K*u)','Position',[660 180 730 215]);
add_block('simulink/Math Operations/Gain',[m '/B'],'Gain','[0;1]','Multiplication','Matrix(K*u)','Position',[285 160 355 190]);
add_block('simulink/Math Operations/Sum',[m '/ObserverSum'],'Inputs','+++','Position',[785 260 815 310]);
add_block('simulink/Continuous/Integrator',[m '/EstimatedState'],'InitialCondition','[0;0]','Position',[855 260 900 305]);
add_block('simulink/Math Operations/Gain',[m '/A'],'Gain','[0 1;-2 -3]','Multiplication','Matrix(K*u)','Position',[640 350 735 390]);
add_block('simulink/Math Operations/Gain',[m '/EstimatedPosition'],'Gain','[1 0]','Multiplication','Matrix(K*u)','Position',[450 250 540 290]);
add_block('simulink/Math Operations/Gain',[m '/K'],'Gain','[4 2]','Multiplication','Matrix(K*u)','Position',[90 280 155 320]);
add_block('simulink/Signal Routing/Mux',[m '/StateMux'],'Inputs','2','Position',[990 55 995 135]);
add_block('simulink/Sinks/Scope',[m '/CompareStates'],'Position',[1050 35 1110 85]);
add_block('simulink/Sinks/To Workspace',[m '/LogStates'],'VariableName','states','SaveFormat','Structure With Time','Position',[1040 120 1130 160]);
wires = {'Reference/1','N/1'; 'N/1','ControlSum/1'; 'ControlSum/1','Plant/1';
'Plant/1','PositionSensor/1'; 'PositionSensor/1','Innovation/1';
'ControlSum/1','B/1'; 'B/1','ObserverSum/1'; 'Innovation/1','L/1';
'L/1','ObserverSum/2'; 'A/1','ObserverSum/3'; 'ObserverSum/1','EstimatedState/1';
'EstimatedState/1','A/1'; 'EstimatedState/1','EstimatedPosition/1';
'EstimatedPosition/1','Innovation/2'; 'EstimatedState/1','K/1'; 'K/1','ControlSum/2';
'Plant/1','StateMux/1'; 'EstimatedState/1','StateMux/2';
'StateMux/1','CompareStates/1'; 'StateMux/1','LogStates/1'};
for k=1:size(wires,1), add_line(m,wires{k,1},wires{k,2},'autorouting','on'); end
Simulink.Annotation(m,sprintf('全维状态观测器入门 | 只能测量位置 y=x1\n对象：x_dot=Ax+Bu     控制：u=6r-K*xhat\n观测器：xhat_dot=A*xhat+Bu+L*(y-C*xhat)\nK=[4 2]，L=[8;4]；反馈只使用估计状态。'));
set_param(m,'SimulationCommand','update');
save_system(m,modelFile);
end
