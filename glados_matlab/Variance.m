% motor_variance_unloaded.csv was recorded with a gain of 220, which is off
% by a factor of 10, and the new gain satisfying a least-squares curve fit
% is calculated.
data = csvread('motor_variance_unloaded.csv', 1); % 1 skips the header
reference = data(:,1);
left_speed = data(:,2);
left_var = data(:,3);
right_speed = data(:,4);
right_var = data(:,5);
avg_speed = data(:,6);
avg_var = data(:,7);

% Do a linear least squares curve fit through the origin
% The slope should be 1
gain = 21.2487; % Insert the gain here
m = sum(reference .* avg_speed) / sum(reference.^2);
new_gain = gain / m

% Plot the variance as a function of speed, and indicate the "mean"
% variance
plot(avg_speed, avg_var, avg_speed, mean(avg_var));
