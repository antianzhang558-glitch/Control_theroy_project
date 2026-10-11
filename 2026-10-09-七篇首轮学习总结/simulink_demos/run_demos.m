function results = run_demos
% Simulate both SLX files, validate behavior, and save comparison plots.
root=fileparts(mfilename('fullpath')); addpath(root);
results=struct;
names={'mpct_simple','rg_pi_simple'};
f=figure('Visible','off','Color','w','Position',[100 100 1100 800]);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
report=fopen(fullfile(root,'verification.txt'),'w');
cleanup=onCleanup(@()fclose(report)); %#ok<NASGU>
fprintf(report,'Actual Simulink runs; normalized teaching plant, not Stirling validation.\n');
fprintf(report,'Plant: x+=0.9*x+0.1*u; x,u in [0,1]; artificial equilibrium/reference in [0,0.98].\n');
for k=1:2
    name=names{k}; load_system(fullfile(root,[name '.slx']));
    out=sim(name,'SimulationMode','normal');
    t=out.tout; tr=out.tracking_log; u=out.input_log;
    assert(all(isfinite([tr(:);u(:)])),'Nonfinite simulation result');
    assert(min(tr(:,3))>=-2e-7 && max(tr(:,3))<=1+2e-7,'State bound violated');
    assert(min(u)>=-2e-7 && max(u)<=1+2e-7,'Input bound violated');
    assert(min(tr(:,2))>=-2e-7 && max(tr(:,2))<=0.98+2e-7,'Target bound violated');
    assert(max(abs(tr(1:10,3)))<1e-7,'Initial zero condition incorrect');
    assert(abs(tr(t==79,3)-0.7)<0.015,'Reachable reference tracking failed');
    assert(abs(tr(t==159,3)-0.98)<0.015,'Unreachable target response failed');
    assert(abs(tr(end,3)-0.3)<0.015,'Downward reference tracking failed');
    assert(max(abs(tr(2:end,3)-(0.9*tr(1:end-1,3)+0.1*u(1:end-1))))<1e-8, ...
        'Logged dynamics/time alignment mismatch');
    results.(name)=struct('time',t,'tracking',tr,'input',u);
    fprintf(report,'\n%s: PASS\n',name);
    fprintf(report,'output range [%.9f, %.9f], input range [%.9f, %.9f]\n',min(tr(:,3)),max(tr(:,3)),min(u),max(u));
    fprintf(report,'y at 79,159,240 s = %.9f, %.9f, %.9f\n',tr(t==79,3),tr(t==159,3),tr(end,3));
    nexttile(k); plot(t,tr(:,1),'k--','LineWidth',1.2); hold on;
    plot(t,tr(:,2),'Color',[0.8 .35 .1],'LineWidth',1.2);
    plot(t,tr(:,3),'Color',[0 .35 .65],'LineWidth',1.5); grid on;
    yline(1,':','physical bound'); ylim([-.05 1.3]);
    legend('requested r','artificial target / RG v','actual y','Location','best');
    title(strrep(name,'_',' ')); ylabel('normalized output');
    nexttile(k+2); stairs(t,u,'LineWidth',1.3); grid on;
    yline(1,':'); yline(0,':'); ylim([-.05 1.1]); ylabel('actual input u');
    nexttile(k+4);
    if k==2
        kap=out.kappa_log; results.(name).kappa=kap;
        assert(min(kap)>=-1e-9 && max(kap)<=1+1e-9);
        assert(max(abs(tr(:,2)-([0;tr(1:end-1,2)]+kap.*(tr(:,1)-[0;tr(1:end-1,2)]))))<1e-8);
        % Verify the RG tail construction against long constant-v predictions.
        I=0; worst=-inf;
        for n=1:numel(t)
            X=[tr(n,3);I]; v=tr(n,2);
            for j=1:1200
                uj=2*(v-X(1))+X(2);
                worst=max([worst;X(1)-1;-X(1);uj-1;-uj]);
                X=[0.9*X(1)+0.1*uj;X(2)+0.2*(v-X(1))];
            end
            I=I+0.2*(v-tr(n,3));
        end
        assert(worst<3e-7,'Constant-reference RG rollout violates bounds');
        fprintf(report,'RG 1200-step constant-reference checks at every sample: PASS; max residual %.3g\n',worst);
        stairs(t,kap,'LineWidth',1.2); ylim([-.05 1.05]); ylabel('RG kappa'); grid on;
    else
        plot(t,tr(:,2)-tr(:,3),'LineWidth',1.2); ylabel('artificial target - actual y'); grid on;
    end
    xlabel('time / sample (Ts=1)');
    close_system(name,0);
end
save(fullfile(root,'demo_results.mat'),'results');
exportgraphics(f,fullfile(root,'comparison.png'),'Resolution',150);
savefig(f,fullfile(root,'comparison.fig')); close(f);
% A second run must restart the controller state, not reuse a previous run.
for k=1:2
    name=names{k}; load_system(fullfile(root,[name '.slx']));
    out=sim(name,'StopTime','20','SimulationMode','normal');
    ref=results.(name).tracking(1:21,:);
    assert(max(abs(out.tracking_log(:)-ref(:)))<1e-8,'Restart is not deterministic');
    close_system(name,0);
end
fprintf(report,'\nRepeat-run initialization: PASS for both models.\n');
fprintf('Both teaching models passed actual Simulink simulations.\n');
end
