function build_models
% Rebuild the two small, independent teaching diagrams.
root = fileparts(mfilename('fullpath'));
addpath(root);
load_system('simulink');
names = {'mpct_simple','rg_pi_simple'};
for k = 1:numel(names)
    name = names{k};
    if bdIsLoaded(name), close_system(name,0); end
    new_system(name);
    set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
        'FixedStep','1','StartTime','0','StopTime','240', ...
        'ReturnWorkspaceOutputs','on','SaveTime','on','TimeSaveName','tout', ...
        'SaveOutput','off','SignalLogging','off','ScreenColor','white');
    set_param(name,'PostLoadFcn','addpath(fileparts(get_param(bdroot,''FileName'')));', ...
        'InitFcn','addpath(fileparts(get_param(bdroot,''FileName''))); clear simple_mpct_step simple_rg_step;');
    mw = get_param(name,'ModelWorkspace');
    reference_data.time = [0;10;80;160;240];
    reference_data.signals.values = [0;0.7;1.2;0.3;0.3];
    reference_data.signals.dimensions = 1;
    assignin(mw,'reference_data',reference_data);
    add_block('simulink/Sources/From Workspace',[name '/User_reference_r'], ...
        'VariableName','reference_data','Interpolate','off','SampleTime','1', ...
        'OutputAfterFinalValue','Holding final value', ...
        'Position',[40 155 155 195]);
    add_plant(name,[650 160 790 215]);
    if k==1
        controller = [name '/Joint_MPC'];
        add_block('simulink/User-Defined Functions/MATLAB Function',controller, ...
            'Position',[290 135 470 225]);
        script = sprintf(['function [u,ya] = fcn(r,x)\n' ...
            '%% Joint optimization of artificial equilibrium and future inputs.\n' ...
            'coder.extrinsic(''simple_mpct_step'');\n' ...
            'a=zeros(2,1);\na=simple_mpct_step([r;x]);\nu=a(1);\nya=a(2);\nend\n']);
        set_chart(controller,script);
        wire(name,'User_reference_r/1','Joint_MPC/1','r');
        wire(name,'Plant/1','Joint_MPC/2','x');
        wire(name,'Joint_MPC/1','Plant/1','u');
        target = 'Joint_MPC/2'; input = 'Joint_MPC/1';
        title = ['人工参考 MPC：人工稳态和未来输入一起优化' newline ...
            '观察 r / ya / y：执行最优输入序列的第一项，再重新计算。'];
    else
        controller = [name '/Reference_governor'];
        add_block('simulink/User-Defined Functions/MATLAB Function',controller, ...
            'Position',[230 110 410 250]);
        script = sprintf(['function [v,kappa] = fcn(r,x,I,previous_v)\n' ...
            '%% Predict the PI+plant closed loop, then maximize admissible kappa.\n' ...
            'coder.extrinsic(''simple_rg_step'');\n' ...
            'a=zeros(2,1);\na=simple_rg_step([r;x;I;previous_v]);\nv=a(1);\nkappa=a(2);\nend\n']);
        set_chart(controller,script);
        add_pi(name,[475 150 580 220]);
        add_block('simulink/Discrete/Unit Delay',[name '/Previous_reference'], ...
            'SampleTime','1','InitialCondition','0','Position',[260 310 350 345]);
        wire(name,'User_reference_r/1','Reference_governor/1','r');
        wire(name,'Plant/1','Reference_governor/2','x');
        wire(name,'PI/2','Reference_governor/3','I');
        wire(name,'Previous_reference/1','Reference_governor/4','previous_v');
        wire(name,'Reference_governor/1','Previous_reference/1','v');
        wire(name,'Reference_governor/1','PI/1','v');
        wire(name,'Plant/1','PI/2','x');
        wire(name,'PI/1','Plant/1','u');
        target = 'Reference_governor/1'; input = 'PI/1';
        add_sink(name,'Kappa_scope','simulink/Sinks/Scope',[840 380 930 425]);
        add_log(name,'kappa_log',[1010 385 1110 415]);
        wire(name,'Reference_governor/2','Kappa_scope/1','kappa');
        wire(name,'Reference_governor/2','kappa_log/1','kappa');
        title = ['RG + PI：RG 选参考，PI 算输入' newline ...
            '观察 r / v / y 和 kappa：保持参考，不等于输入和输出停止变化。'];
    end
    add_block('simulink/Signal Routing/Mux',[name '/Reference_target_output'], ...
        'Inputs','3','Position',[835 110 840 200]);
    add_sink(name,'Tracking_scope','simulink/Sinks/Scope',[905 125 1010 180]);
    add_log(name,'tracking_log',[1045 200 1150 230]);
    wire(name,'User_reference_r/1','Reference_target_output/1','r');
    wire(name,target,'Reference_target_output/2','target');
    wire(name,'Plant/1','Reference_target_output/3','y');
    wire(name,'Reference_target_output/1','Tracking_scope/1','r_target_y');
    wire(name,'Reference_target_output/1','tracking_log/1','r_target_y');
    add_sink(name,'Input_scope','simulink/Sinks/Scope',[840 275 930 320]);
    add_log(name,'input_log',[1010 280 1110 310]);
    wire(name,input,'Input_scope/1','u');
    wire(name,input,'input_log/1','u');
    note = Simulink.Annotation(name,title);
    note.Position = [40 25]; note.FontSize = 14;
    note2 = Simulink.Annotation(name, ...
        ['教学对象：x(k+1)=0.9x(k)+0.1u(k)，y=x，Ts=1' newline ...
        '约束：0<=x,u<=1；人工稳态/参考上限0.98。无单位，不是斯特林模型。' newline ...
        'r：0 → 0.7（10s）→ 1.2（80s，不可达）→ 0.3（160s）' newline ...
        '双击 Plant / PI 查看结构；Tracking_scope 三条线依次为 r、target、y。']);
    note2.Position = [40 445]; note2.FontSize = 11;
    set_param(name,'Location',[40 40 1350 780],'ZoomFactor','FitSystem');
    save_system(name,fullfile(root,[name '.slx']));
    set_param(name,'SimulationCommand','update');
    print(['-s' name],'-dpng','-r120',fullfile(root,[name '_diagram.png']));
    close_system(name,0);
end
fprintf('Built both models in %s\n',root);
end

function set_chart(path,script)
rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','Path',path);
chart.Script = script;
end

function add_plant(name,pos)
path = [name '/Plant'];
add_block('simulink/Ports & Subsystems/Subsystem',path,'Position',pos);
Simulink.SubSystem.deleteContents(path);
add_block('simulink/Ports & Subsystems/In1',[path '/u'],'Position',[25 70 55 90]);
add_block('simulink/Math Operations/Gain',[path '/input_gain'], ...
    'Gain','0.1','Position',[95 60 150 100]);
add_block('simulink/Math Operations/Sum',[path '/sum'], ...
    'Inputs','++','Position',[200 65 220 95]);
add_block('simulink/Discrete/Unit Delay',[path '/state_x'], ...
    'SampleTime','1','InitialCondition','0','Position',[275 60 330 100]);
add_block('simulink/Math Operations/Gain',[path '/memory_gain'], ...
    'Gain','0.9','Position',[180 170 235 210]);
add_block('simulink/Ports & Subsystems/Out1',[path '/y'], ...
    'Position',[385 70 415 90]);
wire(path,'u/1','input_gain/1',''); wire(path,'input_gain/1','sum/1','');
wire(path,'sum/1','state_x/1',''); wire(path,'state_x/1','y/1','x');
wire(path,'state_x/1','memory_gain/1',''); wire(path,'memory_gain/1','sum/2','');
end

function add_pi(name,pos)
path = [name '/PI'];
add_block('simulink/Ports & Subsystems/Subsystem',path,'Position',pos);
Simulink.SubSystem.deleteContents(path);
add_block('simulink/Ports & Subsystems/In1',[path '/v'], ...
    'Port','1','Position',[25 55 55 75]);
add_block('simulink/Ports & Subsystems/In1',[path '/x'], ...
    'Port','2','Position',[25 120 55 140]);
add_block('simulink/Math Operations/Sum',[path '/error'], ...
    'Inputs','+-','Position',[105 60 125 90]);
add_block('simulink/Math Operations/Gain',[path '/Kp'], ...
    'Gain','2','Position',[175 45 220 85]);
add_block('simulink/Math Operations/Gain',[path '/Ki'], ...
    'Gain','0.2','Position',[175 150 220 190]);
add_block('simulink/Math Operations/Sum',[path '/integral_update'], ...
    'Inputs','++','Position',[260 155 280 185]);
add_block('simulink/Discrete/Unit Delay',[path '/integral_state'], ...
    'SampleTime','1','InitialCondition','0','Position',[325 150 375 190]);
add_block('simulink/Math Operations/Sum',[path '/control'], ...
    'Inputs','++','Position',[325 50 345 80]);
add_block('simulink/Ports & Subsystems/Out1',[path '/u'], ...
    'Port','1','Position',[455 55 485 75]);
add_block('simulink/Ports & Subsystems/Out1',[path '/I'], ...
    'Port','2','Position',[455 155 485 175]);
wire(path,'v/1','error/1',''); wire(path,'x/1','error/2','');
wire(path,'error/1','Kp/1','e'); wire(path,'error/1','Ki/1','');
wire(path,'Kp/1','control/1',''); wire(path,'Ki/1','integral_update/1','');
wire(path,'integral_update/1','integral_state/1','');
wire(path,'integral_state/1','integral_update/2','');
wire(path,'integral_state/1','control/2','');
wire(path,'control/1','u/1',''); wire(path,'integral_state/1','I/1','I');
end

function wire(model,src,dst,label)
h=add_line(model,src,dst,'autorouting','on');
if ~isempty(label), set_param(h,'Name',label); end
end
function add_sink(name,label,lib,pos)
add_block(lib,[name '/' label],'Position',pos);
end
function add_log(name,label,pos)
add_block('simulink/Sinks/To Workspace',[name '/' label], ...
    'VariableName',label,'SaveFormat','Array','MaxDataPoints','inf','Position',pos);
end
