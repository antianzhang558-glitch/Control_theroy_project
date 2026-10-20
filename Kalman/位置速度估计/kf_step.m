function [xhat,xnext,Pnext,K] = kf_step(z,xprior,Pprior)
%#codegen
% Correct the current prior, then predict the next sample.
Ts = 0.1;
A = [1 Ts; 0 1];
H = [1 0];
G = [Ts^2/2; Ts];
Q = 0.04*(G*G');
R = 0.25;
K = Pprior*H'/(H*Pprior*H'+R);
xhat = xprior + K*(z-H*xprior);
J = eye(2)-K*H;
Ppost = J*Pprior*J' + K*R*K';
xnext = A*xhat;
Pnext = A*Ppost*A'+Q;
Pnext = (Pnext+Pnext')/2;
end
