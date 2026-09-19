function build_fuzzy_pid
% Simple Mamdani gain-scheduled PID. All artifacts stay beside this script.
outdir = fileparts(mfilename('fullpath'));
cd(outdir);
mdl = 'simple_fuzzy_pid';
fis = mamfis('Name','Triangle_PID','AndMethod','min','OrMethod','max', ...
    'ImplicationMethod','min','AggregationMethod','max','DefuzzificationMethod','centroid');
labels = {'NB','NM','NS','ZO','PS','PM','PB'};
tri = [-3 -3 -2; -3 -2 -1; -2 -1 0; -1 0 1; 0 1 2; 1 2 3; 2 3 3];
for v = {'e','ec'}
    fis = addInput(fis,[-3 3],'Name',v{1});
    for j=1:7, fis=addMF(fis,v{1},'trimf',tri(j,:),'Name',labels{j}); end
end
for v = {'dKp','dKi','dKd'}
    fis = addOutput(fis,[-3 3],'Name',v{1});
    for j=1:7, fis=addMF(fis,v{1},'trimf',tri(j,:),'Name',labels{j}); end
end
% Rows: e NB..PB. Columns: ec NB..PB. Entries: output MF index.
% Symmetric magnitude scheduling: large |e| -> stronger P, weaker I;
% large |ec| -> stronger D. This is a teaching rule base, not plant optimization.
pRule = [7 7 7 7 7 7 7;6 6 6 6 6 6 6;5 5 4 4 4 5 5;5 4 4 4 4 4 5;5 5 4 4 4 5 5;6 6 6 6 6 6 6;7 7 7 7 7 7 7];
iRule = [1 1 1 1 1 1 1;2 2 2 2 2 2 2;3 4 5 5 5 4 3;3 4 5 6 5 4 3;3 4 5 5 5 4 3;2 2 2 2 2 2 2;1 1 1 1 1 1 1];
dRule = [7 6 5 4 5 6 7;7 6 5 4 5 6 7;7 6 5 4 5 6 7;7 6 5 4 5 6 7;7 6 5 4 5 6 7;7 6 5 4 5 6 7;7 6 5 4 5 6 7];
rules=zeros(49,7); n=0;
for i=1:7
    for j=1:7
        n=n+1; rules(n,:)=[i j pRule(i,j) iRule(i,j) dRule(i,j) 1 1];
    end
end
fis=addRule(fis,rules);
writeFIS(fis,fullfile(outdir,'triangle_pid.fis'));
% Verify coverage, including domain edges and the origin.
x=linspace(-3,3,61)'; mu=zeros(numel(x),7);
for j=1:7, mu(:,j)=trimf(x,tri(j,:)); end
assert(all(sum(mu,2)>0));
[E,EC]=ndgrid(-3:0.3:3); vals=evalfis(fis,[E(:) EC(:)]);
assert(all(isfinite(vals),'all'));
if bdIsLoaded(mdl), close_system(mdl,0); end
load_system('simulink'); load_system('fuzblock'); new_system(mdl);
set_param(mdl,'Solver','ode4','SolverType','Fixed-step','FixedStep','0.01', ...
    'StopTime','20','ReturnWorkspaceOutputs','on');
% Load companion FIS relative to the model, independent of current folder.

init = ['fp=fullfile(fileparts(get_param(bdroot,''FileName'')),''triangle_pid.fis''); ' ...
    'simple_fuzzy_pid_fis=readfis(fp); ' ...
    'clear fp;'];
set_param(mdl,'InitFcn',init);
assignin('base','simple_fuzzy_pid_fis',fis);
B=@(lib,name,pos,varargin)add_block(['simulink/' lib],[mdl '/' name],'Position',pos,varargin{:});
B('Sources/Step','Reference',[30 160 60 190],'Time','1','Before','0','After','1');
B('Math Operations/Sum','Error',[115 155 140 195],'Inputs','+-');
B('Ports & Subsystems/Subsystem','Fuzzy gain scheduler',[225 35 415 115]);
B('Ports & Subsystems/Subsystem','Variable gain PID',[470 140 650 245]);
B('Continuous/Transfer Fcn','Plant',[735 165 855 210],'Numerator','1','Denominator','[1 2 1]');
B('Signal Routing/Mux','Response mux',[920 100 925 155],'Inputs','2');
B('Sinks/Scope','Reference and output',[975 105 1025 150]);
B('Sinks/To Workspace','Response data',[975 185 1085 215],'VariableName','response','SaveFormat','Timeseries');
B('Sinks/To Workspace','Gain data',[465 25 570 55],'VariableName','gains','SaveFormat','Timeseries');
sub=[mdl '/Fuzzy gain scheduler']; Simulink.SubSystem.deleteContents(sub);
S=@(lib,name,pos,varargin)add_block(['simulink/' lib],[sub '/' name],'Position',pos,varargin{:});
S('Ports & Subsystems/In1','e',[20 60 50 80]);
S('Math Operations/Gain','Ke',[100 45 145 80],'Gain','3');
S('Continuous/Transfer Fcn','Filtered error rate',[90 155 180 195],'Numerator','[20 0]','Denominator','[1 20]');
S('Math Operations/Gain','Kec',[210 155 255 195],'Gain','0.6');
S('Discontinuities/Saturation','E range',[290 45 345 80],'LowerLimit','-3','UpperLimit','3');
S('Discontinuities/Saturation','EC range',[290 155 345 190],'LowerLimit','-3','UpperLimit','3');
S('Signal Routing/Mux','e ec',[390 60 395 175],'Inputs','2');
libs=find_system('fuzblock','SearchDepth',1,'Type','Block');
lib=libs{find(cellfun(@(s)strcmp(regexprep(get_param(s,'Name'),'\s',''),'FuzzyLogicController'),libs),1)};
add_block(lib,[sub '/Triangle Mamdani FIS'],'Position',[445 85 600 145],'FIS','simple_fuzzy_pid_fis');
S('Math Operations/Gain','Adjustment scales',[645 90 725 140],'Gain','[0.4;0.12;0.08]','Multiplication','Element-wise(K.*u)');
S('Sources/Constant','Base PID',[650 210 725 245],'Value','[3;1;0.6]');
S('Math Operations/Sum','Scheduled gains',[780 110 810 150],'Inputs','++');
S('Ports & Subsystems/Out1','Kp Ki Kd',[865 120 895 140]);
wire(sub,{'e/1','Ke/1';'e/1','Filtered error rate/1';'Ke/1','E range/1';'Filtered error rate/1','Kec/1';'Kec/1','EC range/1';'E range/1','e ec/1';'EC range/1','e ec/2';'e ec/1','Triangle Mamdani FIS/1';'Triangle Mamdani FIS/1','Adjustment scales/1';'Adjustment scales/1','Scheduled gains/1';'Base PID/1','Scheduled gains/2';'Scheduled gains/1','Kp Ki Kd/1'});
sub=[mdl '/Variable gain PID']; Simulink.SubSystem.deleteContents(sub);
S=@(lib,name,pos,varargin)add_block(['simulink/' lib],[sub '/' name],'Position',pos,varargin{:});
S('Ports & Subsystems/In1','e',[20 120 50 140],'Port','1');
S('Ports & Subsystems/In1','Kp Ki Kd',[20 25 50 45],'Port','2');
S('Signal Routing/Demux','Gains',[105 20 110 90],'Outputs','3');
S('Continuous/Integrator','Integral e',[180 155 215 190]);
S('Continuous/Transfer Fcn','Filtered derivative',[160 245 250 280],'Numerator','[20 0]','Denominator','[1 20]');
for k=1:3
    names={'P','I','D'}; S('Math Operations/Product',names{k},[320 75+(k-1)*90 350 105+(k-1)*90],'Inputs','2');
end
S('Math Operations/Sum','PID sum',[440 150 470 205],'Inputs','+++');
S('Ports & Subsystems/Out1','u',[530 170 560 190]);
wire(sub,{'Kp Ki Kd/1','Gains/1';'e/1','P/1';'e/1','Integral e/1';'e/1','Filtered derivative/1';'Integral e/1','I/1';'Filtered derivative/1','D/1';'Gains/1','P/2';'Gains/2','I/2';'Gains/3','D/2';'P/1','PID sum/1';'I/1','PID sum/2';'D/1','PID sum/3';'PID sum/1','u/1'});
wire(mdl,{'Reference/1','Error/1';'Error/1','Fuzzy gain scheduler/1';'Error/1','Variable gain PID/1';'Fuzzy gain scheduler/1','Variable gain PID/2';'Fuzzy gain scheduler/1','Gain data/1';'Variable gain PID/1','Plant/1';'Plant/1','Error/2';'Reference/1','Response mux/1';'Plant/1','Response mux/2';'Response mux/1','Reference and output/1';'Response mux/1','Response data/1'});
a=Simulink.Annotation(mdl,sprintf('TRIANGULAR FUZZY PID | 2 inputs, 3 outputs, 49 rules\nPlant: 1/(s+1)^2 | Step at 1 s | Stop: 20 s\nDouble-click scheduler to inspect normalization and FIS.'));
a.Position=[230 300];
set_param(mdl,'ZoomFactor','FitSystem');
save_system(mdl,fullfile(outdir,[mdl '.slx']));
simout=sim(mdl); ts=simout.response; gs=simout.gains;
y=ts.Data(:,2); assert(all(isfinite(y))); assert(abs(y(end)-1)<0.02);
metrics=struct('FinalOutput',y(end),'FinalError',1-y(end),'OvershootPercent',max(0,(max(y)-1)*100),'PeakAbsOutput',max(abs(y)));
disp(metrics); save(fullfile(outdir,'simulation_results.mat'),'ts','gs','metrics');
f=figure('Visible','off','Color','w'); tiledlayout(2,1);
nexttile; plot(ts.Time,ts.Data,'LineWidth',1.4); grid on; legend('Reference','Output'); xlabel('Time (s)'); ylabel('Response'); title('Triangular fuzzy PID: unit step');
nexttile; plot(gs.Time,gs.Data,'LineWidth',1.2); grid on; legend('Kp','Ki','Kd'); xlabel('Time (s)'); ylabel('Scheduled gains');
exportgraphics(f,fullfile(outdir,'step_response.png'),'Resolution',150); close(f);
f=figure('Visible','off','Color','w','Position',[100 100 1000 750]); tiledlayout(3,2);
for k=1:2, nexttile; plotmf(fis,'input',k); grid on; end
for k=1:3, nexttile; plotmf(fis,'output',k); grid on; end
exportgraphics(f,fullfile(outdir,'triangular_memberships.png'),'Resolution',150); close(f);
print(['-s' mdl],'-dpng',fullfile(outdir,'model_diagram.png'));
% Reload saved SLX without a pre-existing FIS variable and verify initialization.
close_system(mdl,0); evalin('base','clear simple_fuzzy_pid_fis');
load_system(fullfile(outdir,[mdl '.slx'])); check=sim(mdl);
assert(abs(check.response.Data(end,2)-y(end))<1e-9);
fid=fopen(fullfile(outdir,'validation.txt'),'w');
fprintf(fid,'MATLAB %s\n49 rules; triangular membership coverage verified at 61 points.\n441 input pairs: finite inference outputs.\nSaved-model reload simulation passed.\nFinal output: %.8f\nFinal error: %.8f\nOvershoot: %.4f percent\n',version,metrics.FinalOutput,metrics.FinalError,metrics.OvershootPercent); fclose(fid);
disp('BUILD_AND_VALIDATION_OK');
end
function wire(sys,pairs)
for k=1:size(pairs,1), add_line(sys,pairs{k,1},pairs{k,2},'autorouting','on'); end
end
