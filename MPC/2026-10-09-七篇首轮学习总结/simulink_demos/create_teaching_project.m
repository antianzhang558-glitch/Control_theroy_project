function create_teaching_project
root=fileparts(mfilename('fullpath'));
project=matlab.project.createProject('Folder',root,'Name','MPC_RG_Teaching');
addPath(project,root);
files={'mpct_simple.slx','rg_pi_simple.slx','simple_mpct_step.m','simple_rg_step.m', ...
    'build_models.m','run_demos.m','open_demos.m','start_here.m','simulate_both.m', ...
    'README.md','create_teaching_project.m'};
for n=1:numel(files), addFile(project,fullfile(root,files{n})); end
addStartupFile(project,fullfile(root,'start_here.m'));
addShortcut(project,fullfile(root,'simulate_both.m'));
close(project);
fprintf('Created project entry in %s\n',root);
end
