function build_observer_simple
% 简洁版：目标 -> 控制器 -> 对象，观测器估计状态并反馈。
folder=fileparts(mfilename('fullpath'));
m='observer_simple'; file=fullfile(folder,[m '.slx']);
if isfile(file), error('简洁模型已存在，请直接打开，避免覆盖修改。'); end
new_system(m);
set_param(m,'StopTime','8','Solver','ode45','MaxStep','0.01','RelTol','1e-8','AbsTol','1e-10','SignalLogging','on');
add_block('simulink/Sources/Constant',[m '/目标位置'],'Value','1','Position',[30 85 65 115]);
ctrl=[m '/控制器']; add_block('built-in/Subsystem',ctrl,'Position',[150 65 275 135]);
add_block('simulink/Ports & Subsystems/In1',[ctrl '/r'],'Port','1','Position',[30 35 60 55]);
add_block('simulink/Ports & Subsystems/In1',[ctrl '/xhat'],'Port','2','Position',[30 95 60 115]);
add_block('simulink/Signal Routing/Mux',[ctrl '/合并'],'Inputs','2','Position',[110 40 115 110]);
add_block('simulink/Math Operations/Gain',[ctrl '/反馈'],'Gain','[6 -4 -2]','Multiplication','Matrix(K*u)','Position',[160 50 250 90]);
add_block('simulink/Ports & Subsystems/Out1',[ctrl '/u'],'Position',[300 60 330 80]);
add_line(ctrl,'r/1','合并/1'); add_line(ctrl,'xhat/1','合并/2'); add_line(ctrl,'合并/1','反馈/1'); add_line(ctrl,'反馈/1','u/1');
add_block('simulink/Continuous/State-Space',[m '/对象'],'A','[0 1;-2 -3]','B','[0;1]','C','[1 0]','D','0','X0','[0.5;0]','Position',[365 65 485 135]);
obs=[m '/全维观测器']; add_block('built-in/Subsystem',obs,'Position',[345 235 500 315]);
add_block('simulink/Ports & Subsystems/In1',[obs '/u'],'Port','1','Position',[25 35 55 55]);
add_block('simulink/Ports & Subsystems/In1',[obs '/y'],'Port','2','Position',[25 105 55 125]);
add_block('simulink/Signal Routing/Mux',[obs '/合并'],'Inputs','2','Position',[100 40 105 115]);
add_block('simulink/Continuous/State-Space',[obs '/观测器方程'],'A','[-8 1;-6 -3]','B','[0 8;1 4]','C','eye(2)','D','zeros(2,2)','X0','[0;0]','Position',[155 45 295 110]);
add_block('simulink/Ports & Subsystems/Out1',[obs '/xhat'],'Position',[350 65 380 85]);
add_line(obs,'u/1','合并/1'); add_line(obs,'y/1','合并/2'); add_line(obs,'合并/1','观测器方程/1'); add_line(obs,'观测器方程/1','xhat/1');
a=Simulink.Annotation(obs,'xhat_dot = (A-LC)*xhat + [B L]*[u;y]'); a.Position=[110 155];
add_block('simulink/Sinks/Scope',[m '/看结果'],'NumInputPorts','2','Position',[630 75 680 135]);
w={'目标位置/1','控制器/1';'控制器/1','对象/1';'控制器/1','全维观测器/1';'对象/1','全维观测器/2';'全维观测器/1','控制器/2';'对象/1','看结果/1';'全维观测器/1','看结果/2'};
for k=1:size(w,1), add_line(m,w{k,1},w{k,2},'autorouting','on'); end
p=get_param([m '/对象'],'PortHandles'); line=get_param(p.Outport,'Line'); set_param(line,'Name','y'); set_param(p.Outport,'DataLogging','on');
p=get_param(obs,'PortHandles'); line=get_param(p.Outport,'Line'); set_param(line,'Name','xhat'); set_param(p.Outport,'DataLogging','on');
p=get_param(ctrl,'PortHandles'); set_param(get_param(p.Outport,'Line'),'Name','u');
a=Simulink.Annotation(m,sprintf('先看三个模块：控制器 → 对象 → 观测器\n对象只测位置；观测器估计位置和速度，再反馈给控制器。')); a.Position=[100 360];
set_param(m,'SimulationCommand','update'); save_system(m,file);
out=sim(m,'ReturnWorkspaceOutputs','on');
y=out.logsout.get('y').Values; xh=out.logsout.get('xhat').Values;
% 独立精确解检查简化后的接口、反馈符号和观测器输入顺序。
A=[0 1;-2 -3];B=[0;1];C=[1 0];K=[4 2];L=[8;4];
M=[A -B*K; L*C A-B*K-L*C]; g=[B*6;B*6]; zs=-M\g;
z0=[0.5;0;0;0]; ey=zeros(size(y.Data)); eh=zeros(size(xh.Data));
for k=1:numel(y.Time), z=zs+expm(M*y.Time(k))*(z0-zs); ey(k)=z(1); end
for k=1:numel(xh.Time), z=zs+expm(M*xh.Time(k))*(z0-zs); eh(k,:)=z(3:4)'; end
err=max([max(abs(y.Data-ey),[],'all'),max(abs(xh.Data-eh),[],'all')]);
assert(err<1e-5); assert(abs(y.Data(end)-1)<1e-4);
fid=fopen(fullfile(folder,'simple_validation.txt'),'w');
fprintf(fid,'PASS\nMax error vs exact solution: %.3g\nFinal position: %.9f\n',err,y.Data(end)); fclose(fid);
fprintf('PASS: simplified model; max error %.3g, final position %.9f\n',err,y.Data(end));
if usejava('desktop'),open_system(m);end
end

