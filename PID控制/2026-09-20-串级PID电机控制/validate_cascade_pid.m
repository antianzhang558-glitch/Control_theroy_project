function validate_cascade_pid
% Meaningful nominal, perturbation, and saturation checks.
folder=fileparts(mfilename('fullpath'));
load_system(fullfile(folder,'cascade_pid_motor.slx'));
model='cascade_pid_motor';
fid=fopen(fullfile(folder,'validation.txt'),'w');
cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'MATLAB %s\n',version);
scenarios={'nominal','inertia_150_percent','double_load','large_command'};
for k=1:numel(scenarios)
    input=Simulink.SimulationInput(model);
    if k==2, input=input.setVariable('J',0.015,'Workspace',model); end
    if k==3, input=input.setVariable('load_torque',0.1,'Workspace',model); end
    if k==4
        input=input.setVariable('theta_step',4*pi,'Workspace',model);
        input=input.setVariable('theta_change',-8*pi,'Workspace',model);
    end
    out=sim(input);
    p=out.get('position_log'); s=out.get('speed_log'); a=out.get('actuator_log');
    err=p.Data(:,1)-p.Data(:,2);
    assert(all(isfinite(p.Data(:))) && all(isfinite(s.Data(:))) && all(isfinite(a.Data(:))));
    assert(max(abs(a.Data(:,1)))<=24+1e-6,'Voltage limit failed');
    assert(max(abs(s.Data(:,1)))<=8+1e-6,'Speed command limit failed');
    assert(abs(err(end))<0.025,'Final error exceeds 0.025 rad');
    assert(max(abs(err(p.Time>15)))<0.03,'Late response has not settled');
    disturbance=p.Time>=4 & p.Time<8;
    fprintf(fid,'%s: PASS | final error=%.6f rad | peak voltage=%.3f V | peak current=%.3f A | load interval max error=%.6f rad\n', ...
        scenarios{k},err(end),max(abs(a.Data(:,1))),max(abs(a.Data(:,2))),max(abs(err(disturbance))));
    if k==1
        ix=p.Time>=0.5 & p.Time<4;
        overshoot=max(0,(max(p.Data(ix,2))-pi/2)/(pi/2)*100);
        fprintf(fid,'Nominal first-step overshoot=%.3f percent\n',overshoot);
    end
end
fprintf('All four simulation checks passed.\n');
end
