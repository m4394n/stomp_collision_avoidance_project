%% Test PoE FK vs MATLAB getTransform

clc; clear; close all;

%% Load robot
robot_name = 'kinovaGen3';
robot = loadrobot(robot_name, 'DataFormat', 'column');

nJoints = numel(robot.homeConfiguration);

%% Random joint configuration
theta = rand(nJoints,1) * pi/2;  % angles in [0, pi/2]

%% --- 1. FK using your PoE implementation ---
[X_poe, T_poe] = updateJointsWorldPosition(robot, theta);

%% --- 2. FK using MATLAB built-in getTransform ---
T_builtin = cell(1, nJoints);
for k = 1:nJoints
    T_builtin{k} = getTransform(robot, theta, robot.BodyNames{k});
end

%% --- 3. Compare each joint ---
tolerance = 1e-6;
fprintf('Comparing PoE FK vs getTransform for each joint:\n');
for k = 1:nJoints
    diff = T_poe{k} - T_builtin{k};
    if max(abs(diff(:))) > tolerance
        fprintf('Joint %d differs! Max abs difference = %.2e\n', k, max(abs(diff(:))));
    else
        fprintf('Joint %d matches.\n', k);
    end
end

%% --- 4. Compare end-effector position ---
ee_poe = X_poe(end,1:3)';
ee_builtin = T_builtin{end}(1:3,4);
fprintf('\nEnd-effector position difference: [%.3e, %.3e, %.3e]\n', ee_poe - ee_builtin);

%% --- 5. Visualization ---
figure;
show(robot, theta); hold on;
plot3(X_poe(:,1), X_poe(:,2), X_poe(:,3), 'ro-', 'LineWidth', 2, 'MarkerSize',6);
title('PoE FK (red) vs MATLAB robot visualization (blue)');
grid on; axis equal;
