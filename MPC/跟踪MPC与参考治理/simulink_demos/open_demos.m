function open_demos
% Entry point: keep this folder intact and open the two diagrams.
root=fileparts(mfilename('fullpath'));
addpath(root);
open_system(fullfile(root,'mpct_simple.slx'));
open_system(fullfile(root,'rg_pi_simple.slx'));
fprintf('Press Run, then open Tracking_scope / Input_scope. RG also has Kappa_scope.\n');
end
