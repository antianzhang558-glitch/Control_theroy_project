function out = simple_mpct_step(in)
% Teaching MPCT: jointly optimize U and artificial equilibrium s.
% Plant x+ = 0.9*x + 0.1*u, hence equilibrium ua = xa = s.
% Normal-mode Simulink simulation; Optimization Toolbox required.
persistent H F G phi Aeq lb ub options
N = 10; Q = 1; R = 0.1; S = 20;
if isempty(H)
    phi = (0.9.^(1:N))';
    G = zeros(N);
    for i = 1:N
        for j = 1:i
            G(i,j) = 0.1*0.9^(i-j);
        end
    end
    % State cost uses x_0,...,x_(N-1); input cost uses u_0,...,u_(N-1).
    F = [zeros(1,N); G(1:N-1,:)];
    F = [F, -ones(N,1)];
    Fu = [eye(N), -ones(N,1)];
    es = [zeros(1,N), 1];
    H = 2*(Q*(F'*F) + R*(Fu'*Fu) + S*(es'*es));
    H = (H+H')/2;
    Aeq = [G(end,:), -1]; % x_N = s (terminal equality)
    lb = zeros(N+1,1);
    ub = [ones(N,1); 0.98]; % artificial equilibrium has upper margin
    options = optimoptions('quadprog','Display','off','Algorithm','active-set', ...
        'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9);
end
r = in(1); x = in(2);
f = 2*Q*F'*([1;phi(1:N-1)]*x);
f(end) = f(end)-2*S*r;
Aineq = [G,zeros(N,1); -G,zeros(N,1)];
bineq = [ones(N,1)-phi*x; phi*x];
warm = [x*ones(N,1);x]; % feasible constant equilibrium for the default run
[z,~,flag] = quadprog(H,f,Aineq,bineq,Aeq,-phi(end)*x,lb,ub,warm,options);
if flag <= 0
    error('TeachingMPCT:Infeasible','QP failed: exitflag=%d, x=%.6g, r=%.6g.',flag,x,r);
end
% Expose failures instead of silently clipping the optimizer output.
if max([Aineq*z-bineq; abs(Aeq*z+phi(end)*x); lb-z; z-ub]) > 2e-7
    error('TeachingMPCT:Residual','QP constraint residual exceeds tolerance.');
end
out = [z(1); z(end)]; % first actual input and selected artificial output
end
