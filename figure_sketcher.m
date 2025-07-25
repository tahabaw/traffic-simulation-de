% ---------- IDM DATA ----------
% Load the CSV file (ensure it's in your MATLAB path or working dir)
data = readtable("your_recorded_data_1.csv");

% Convert to numeric arrays
time = data{:, "Time"};
speed = data{:, "Speed"};
position = data{:, "ContinuousPosition"};

% Compute acceleration (numerical derivative of speed)
acceleration = [0; diff(speed) ./ diff(time)];

% Find distance at t = 5000
t_target = 5000;
idx = find(time >= t_target, 1);
if isempty(idx)
    distance_5000 = NaN;
else
    distance_5000 = position(idx);
end

% --- Plot Position, Speed, Acceleration ---
figure('Name', 'IDM', 'NumberTitle', 'off');
sgtitle('RL-MODEL');

subplot(3,1,1);
plot(time, position, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Position of car\_1 Over Time');
grid on;

subplot(3,1,2);
plot(time, speed, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Speed of car\_1 Over Time');
grid on;

subplot(3,1,3);
plot(time, acceleration, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Acceleration of car\_1 Over Time');
grid on;

% Add distance text below plots
annotation('textbox', [0.1, 0.01, 0.8, 0.05], ...
    'String', sprintf('Distance traveled at t = 5000s: %.2f meters', distance_5000), ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');


% ---------- RL-Model DATA ----------
% Load the CSV file
data = readtable("your_recorded_data_2.csv");

% Convert to numeric arrays
time = data{:, "Time"};
speed = data{:, "Speed"};
position = data{:, "ContinuousPosition"};

% Compute acceleration
acceleration = [0; diff(speed) ./ diff(time)];

% Find distance at t = 5000
idx = find(time >= t_target, 1);
if isempty(idx)
    distance_5000 = NaN;
else
    distance_5000 = position(idx);
end

% --- Plot Position, Speed, Acceleration ---
figure('Name', 'RL-Model', 'NumberTitle', 'off');
sgtitle('IDM');

subplot(3,1,1);
plot(time, position, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Position (m)');
title('Position of car\_1 Over Time');
grid on;

subplot(3,1,2);
plot(time, speed, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('Speed of car\_1 Over Time');
grid on;

subplot(3,1,3);
plot(time, acceleration, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Acceleration of car\_1 Over Time');
grid on;

% Add distance text below plots
annotation('textbox', [0.1, 0.01, 0.8, 0.05], ...
    'String', sprintf('Distance traveled at t = 5000s: %.2f meters', distance_5000), ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
