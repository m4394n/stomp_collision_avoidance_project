% Visualize the obstacles and robot manipulator
% ----------------------------------------------------------

positions = x0(1:numJoints)';

% === Create a visible, movable figure ===
CAPTURE_WIDTH  = 875;
CAPTURE_HEIGHT = 526;
screenSize = get(0, 'ScreenSize');  % [left bottom width height] of display
leftPos   = (screenSize(3) - CAPTURE_WIDTH) / 2;  % center horizontally
bottomPos = max(100, screenSize(4) - CAPTURE_HEIGHT - 200); % not off-screen

hgif = figure('Units', 'pixels', ...
              'Position', [leftPos bottomPos CAPTURE_WIDTH CAPTURE_HEIGHT], ...
              'Resize', 'on', ...      
              'Visible', 'on', ...
              'Name', 'Kinova Capture', ...
              'NumberTitle', 'off');
ax1 = axes('Parent', hgif);

% === Plot robot and environment ===
show(robot, positions(:,1), 'PreservePlot', false, 'Frames', 'off', 'Parent', ax1);
view(ax1, 150, 29);
hold(ax1, 'on');

% Lock axis limits and aspect ratio (for stable camera view)
axis(ax1, [-0.8 0.8 -0.6 0.7 -0.2 0.7]);
axis(ax1, 'vis3d');
daspect(ax1, [1 1 1]);

% Plot the final position
plot3(ax1, poseFinal(1), poseFinal(2), poseFinal(3), 'r.', 'MarkerSize', 20);

% Visualize collision world specified in helperCreateObstaclesKINOVA.m
for i = 1:numel(world)
    [~, pObj] = show(world{i}, 'Parent', ax1);
    pObj.LineStyle = 'none';
end

% ----------------------------------------------------------
% Keep the handle for video capture use
assignin('base', 'hgif', hgif);
assignin('base', 'ax1', ax1);
% ----------------------------------------------------------
