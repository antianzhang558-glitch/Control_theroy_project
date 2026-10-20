function build_simple_tank
% Minimal cascade PI teaching example. All signals are normalized.
folder=fileparts(mfilename('fullpath'));
model='simple_cascade_tank';
file=fullfile(folder,[model '.slx']);
if isfile(file) || bdIsLoaded(model)
    error('Model exists. Open the saved model instead of rebuilding.');
end
load_system('simulink'); new_system(model);
set_param(model,'Solver','ode45','StopTime','20','MaxStep','0.01', ...
    'ReturnWorkspaceOutputs','on','Location',[70 80 1420 730]);
b=@(lib,name,pos,varargin) add_block(['simulink/' lib],[model '/' name], ...
    'Position',pos,'FontSize','12',varargin{:});
b('Sources/Step','目标水位',[35 150 65 180],'Time','1','After','1');
b('Math Operations/Sum','水位误差',[160 145 190 185],'Inputs','+-');
b('Continuous/PID Controller','外环：水位PI',[250 130 355 200], ...
    'Controller','PI','P','2','I','0.4','BackgroundColor','lightBlue');
b('Math Operations/Sum','流量误差',[445 145 475 185],'Inputs','+-');
b('Continuous/PID Controller','内环：流量PI',[535 130 640 200], ...
    'Controller','PI','P','2','I','10','BackgroundColor','green');
b('Continuous/Transfer Fcn','水泵',[705 140 805 190], ...
    'Numerator','1','Denominator','[0.2 1]','BackgroundColor','green');
b('Continuous/Transfer Fcn','水箱',[905 140 1005 190], ...
    'Numerator','1','Denominator','[5 1]','BackgroundColor','lightBlue');
b('Sinks/Scope','看水位变化',[1130 137 1175 193],'NumInputPorts','2');
b('Sinks/To Workspace','保存水位',[1120 370 1230 405], ...
    'VariableName','height_log','SaveFormat','Timeseries');
wire(model,'目标水位/1','水位误差/1');
wire(model,'水位误差/1','外环：水位PI/1');
wire(model,'外环：水位PI/1','流量误差/1','目标流量');
wire(model,'流量误差/1','内环：流量PI/1');
wire(model,'内环：流量PI/1','水泵/1','水泵指令');
wire(model,'水泵/1','水箱/1','实际流量');
wire(model,'水箱/1','看水位变化/2','实际水位');
wire(model,'目标水位/1','看水位变化/1');
wire(model,'水箱/1','保存水位/1');
% Explicit feedback routes make the two loops easy to distinguish.
add_line(model,[855 165;855 270;460 270;460 185]);
add_line(model,[1055 165;1055 325;175 325;175 185]);
a=Simulink.Annotation(model,sprintf('最简单的串级控制：水位外环 + 流量内环\n外环：水位低了，需要多少水？    内环：让水泵提供这些水。'));
a.Position=[35 25]; a.FontSize=15;
a=Simulink.Annotation(model,'内环反馈：实际进水流量'); a.Position=[510 280]; a.FontSize=12;
a=Simulink.Annotation(model,'外环反馈：实际水位'); a.Position=[370 337]; a.FontSize=12;
a=Simulink.Annotation(model,sprintf('使用：点击运行，再双击“看水位变化”。目标水位在 1 秒时从 0 变为 1。\n水箱带自然排水，因此保持水位也需要持续进水。信号采用归一化数值。\n先理解两条反馈线即可；PI 是微分项为零的 PID。本模型暂不加入扰动、限幅等内容。'));
a.Position=[35 445]; a.FontSize=12;
set_param(model,'SimulationCommand','update');
save_system(model,file);
open_system(model); set_param(model,'ZoomFactor','FitSystem');
out=sim(model); h=out.get('height_log');
assert(all(isfinite(h.Data(:))),'Nonfinite response');
assert(abs(h.Data(end)-1)<0.005,'Water level did not converge');
assert(min(h.Data)>-1e-6 && max(h.Data)<1.02,'Unexpected response');
f=figure('Visible','off','Color','w','Position',[100 100 950 430]);
plot(h.Time,double(h.Time>=1),'--',h.Time,h.Data,'LineWidth',1.8);
grid on; xlabel('时间 / 秒'); ylabel('归一化水位');
legend('目标水位','实际水位','Location','southeast'); title('简单串级 PI：水位逐渐到达目标');
ax=gca; ax.Toolbar.Visible='off';
exportgraphics(f,fullfile(folder,'simple_tank_response.png'),'Resolution',140); close(f);
print(['-s' model],'-dpng','-r120',fullfile(folder,'simple_tank_diagram.png'));
fid=fopen(fullfile(folder,'simple_tank_validation.txt'),'w');
fprintf(fid,'MATLAB %s\nPASS: compiled and simulated, finite and bounded response.\nFinal height: %.8f; target: 1; final error: %.8f\n',version,h.Data(end),1-h.Data(end)); fclose(fid);
open_system([model '/看水位变化']);
fprintf('SIMPLE_TANK_READY\n');
end
function wire(m,s,d,label)
h=add_line(m,s,d,'autorouting','on');
if nargin==4, set_param(h,'Name',label); end
end
