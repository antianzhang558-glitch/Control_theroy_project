function out = simple_rg_step(in)
% Scalar RG around a fixed PI; maximize kappa in [0,1].
% in = [requested reference; plant state; PI integral state; previous v].
% The admissible set checks a finite prefix AND a certified infinite tail.
persistent L q b
if isempty(L)
    % Plant x+ = .9*x+.1*u; PI u=2*(v-x)+I, I+=.2*(v-x).
    A = [0.7,0.1; -0.2,1]; B = [0.2;0.2];
    C = [1,0; -2,1]; D = [0;2]; E = [1;1];
    assert(norm(A*E+B-E)<1e-12);
    L = zeros(0,2); q = zeros(0,1); b = zeros(0,1);
    Aj = eye(2); horizon = 40;
    for j = 0:horizon-1
        P = C*Aj; g = C*(eye(2)-Aj)*E+D;
        L = [L;P;-P]; q = [q;g;-g]; b = [b;ones(2,1);zeros(2,1)]; %#ok<AGROW>
        Aj = Aj*A;
    end
    % A has real eigenvalues .9 and .8. For every remaining time ell>=0:
    % |C*A^ell*e_H| <= abs(C*V)*abs(V\e_H).
    % Enumerating the four signs makes this absolute-value bound linear.
    [V,T] = eig(A);
    assert(isreal(V) && max(abs(diag(T)))<1);
    signs = [1,1;1,-1;-1,1;-1,-1];
    for row = 1:2
        W = (signs.*abs(C(row,:)*V))*(V\Aj);
        L = [L;W;W]; %#ok<AGROW>
        q = [q;ones(4,1)-W*E;-ones(4,1)-W*E]; %#ok<AGROW>
        b = [b;ones(4,1);zeros(4,1)]; %#ok<AGROW>
    end
    % Restrict constant reference to the same equilibrium range as MPCT.
    L = [L;zeros(2,2)]; q = [q;1;-1]; b = [b;0.98;0];
end
r = in(1); X = in(2:3); oldv = in(4);
slack = b-L*X-q*oldv;
if min(slack)<-2e-8
    error('TeachingRG:NoBackup','Old reference is outside the certified admissible set.');
end
direction = q*(r-oldv);
positive = direction>1e-13;
kappa = 1;
if any(positive)
    kappa = min(1,min(max(slack(positive),0)./direction(positive)));
end
kappa = max(0,kappa);
if kappa<1
    kappa = max(0,kappa-1e-12); % roundoff margin, not an input saturation
end
v = oldv+kappa*(r-oldv);
if max(L*X+q*v-b)>3e-8
    error('TeachingRG:Residual','Updated reference violates the admissible set.');
end
out = [v;kappa];
end
