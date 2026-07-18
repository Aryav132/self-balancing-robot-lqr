% 1. Extract data safely from Simulink 'out' object
raw_data = out.sim_data;
% Handle if Simulink saved it as a 'timeseries' object
if isa(raw_data, 'timeseries')
    raw_data = raw_data.Data;
end
% 2. Crush the 3D array down to a normal 2D table using 'squeeze'
states = squeeze(raw_data);
% Ensure it is formatted as Rows = Time, Columns = States
if size(states, 2) ~= 4
    states = states'; % Transpose it if it is sideways
end
% 3. Extract position and angle
x_pos = states(:, 1);
theta = states(:, 3);
% Physical dimensions for the drawing
R = 0.05;  % Wheel radius (m)
L = 0.3;   % Chassis length (m)
% Create the animation figure window
figure('Name', 'Two-Wheeled Robot Balancing Animation', 'Color', 'white');
% Loop through the live Simulink data frame by frame
for i = 1:5:length(x_pos)
    clf; % Clear the current frame
    hold on;
    grid on;
    % Get current state
    curr_x = x_pos(i);
    curr_theta = theta(i);
    % Draw the Ground
    plot([-2, 2], [0, 0], 'k', 'LineWidth', 2);
    % Draw the Wheel
    rectangle('Position', [curr_x - R, 0, 2*R, 2*R], 'Curvature', [1 1], 'FaceColor', [0.2 0.2 0.2]);
    % Draw the Chassis
    top_x = curr_x - L * sin(curr_theta);
    top_y = R + L * cos(curr_theta);
    plot([curr_x, top_x], [R, top_y], 'b', 'LineWidth', 6); % Main body
    plot(top_x, top_y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Top weight
    % Keep the camera framed nicely around the robot
    axis([-1, 1, -0.1, 0.5]);
    axis equal;
    title(sprintf('Time: %.2f seconds | Angle: %.2f deg', i*0.01, rad2deg(curr_theta)));
    % Render the graphics instantly
    drawnow;
end
