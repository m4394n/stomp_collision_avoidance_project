% Load robot
robot_name = 'kukaIiwa7';
robot = loadrobot(robot_name, 'DataFormat', 'column');
nJoints = numel(robot.homeConfiguration);
N_runs = 1000; 

% Random joint configuration 
theta = rand(nJoints,1) * pi/2;
tolerance = 1e-6;

% Validation Check
fprintf('--- 1. Validation Check: Comparing All Joint Transformations ---\n');

% Run PoE implementation  
[~, T_poe] = updateJointsWorldPosition(robot, theta);

% Calculate MATLAB built-in transforms for all joints
T_builtin = cell(1, nJoints);
for k = 1:nJoints
    % Use robot.BodyNames{k} to get the transformation up to the k-th body
    T_builtin{k} = getTransform(robot, theta, robot.BodyNames{k});
end

all_match = true;
for k = 1:nJoints
    diff = T_poe{k} - T_builtin{k};
    max_diff = max(abs(diff(:)));
    if max_diff > tolerance
        fprintf('Joint %d (%s) differs! Max abs difference = %.2e\n', k, robot.BodyNames{k}, max_diff);
        all_match = false;
    end
end

if all_match
    fprintf('All %d joint transformations match MATLAB built-in (Diff < %.1e).\n', nJoints, tolerance);
else
    fprintf('Validation failed for one or more joints.\n');
end

% Performance Comparison

fprintf('\n--- 2. Performance Comparison (%d runs) ---\n', N_runs);

% Time PoE implementation 
tic;
for i = 1:N_runs
    [~, ~] = updateJointsWorldPosition(robot, theta); 
end
time_poe = toc / N_runs;
fprintf('PoE Implementation (All Joints): %.6f seconds (per run)\n', time_poe);


% Time MATLAB built-in
tic;
for i = 1:N_runs
    for k = 1:nJoints
        getTransform(robot, theta, robot.BodyNames{k});
    end
end
time_builtin = toc / N_runs;
fprintf('MATLAB getTransform (All Joints): %.6f seconds (per run)\n', time_builtin);

% Conclusion
fprintf('\n--- Conclusion ---\n');
if time_poe < time_builtin
    speedup_percent = (time_builtin - time_poe) / time_builtin * 100;
    fprintf('PoE implementation is %.2f%% faster than sequential getTransform calls.\n', speedup_percent);
else
    speedup_percent = (time_poe - time_builtin) / time_poe * 100;
    fprintf('MATLAB built-in (sequential calls) is %.2f%% faster than PoE.\n', speedup_percent);
end