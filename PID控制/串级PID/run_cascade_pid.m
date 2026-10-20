function results = run_cascade_pid
% Open the saved model, simulate it, and export a response figure.
folder=fileparts(mfilename('fullpath'));
file=fullfile(folder,'cascade_pid_motor.slx');
if ~isfile(file), build_cascade_pid; end
load_system(file);
model='cascade_pid_motor';
open_system(model);
results=sim(model);
p=results.get('position_log'); s=results.get('speed_log'); a=results.get('actuator_log');
fig=figure('Name','Cascade PID motor control','Color','w','Position',[100 60 1100 820]);
tiledlayout(4,1,'TileSpacing','compact');
nexttile; plot(p.Time,p.Data(:,1)*180/pi,'--',p.Time,p.Data(:,2)*180/pi,'LineWidth',1.5);
ylabel('Position (deg)'); legend('Reference','Actual','Location','best'); grid on;
title('Cascade PID: position outer loop / speed inner loop');
nexttile; plot(s.Time,s.Data,'LineWidth',1.2); ylabel('Speed (rad/s)');
legend('Reference','Actual','Location','best'); grid on;
nexttile; plot(a.Time,a.Data(:,1),'LineWidth',1.2); ylabel('Voltage (V)'); grid on;
nexttile; yyaxis left; plot(a.Time,a.Data(:,2),'LineWidth',1.2); ylabel('Current (A)');
yyaxis right; plot(a.Time,a.Data(:,3),'LineWidth',1.2); ylabel('Load (N m)'); grid on; xlabel('Time (s)');
axesList=findall(fig,'Type','axes'); for k=1:numel(axesList), axesList(k).Toolbar.Visible='off'; end
exportgraphics(fig,fullfile(folder,'response.png'),'Resolution',160);
save(fullfile(folder,'simulation_results.mat'),'results');
fprintf('Simulation finished. Figures and data saved in %s\n',folder);
end
