%% Chirp Filtering

clc;
clear;
close all;

%%

% construct a series of chirps with the desired properties
F0 = 20000;
F1 = 27000;
robot_speed = 5.7; % m/s
speed_of_sound = 343.2; % m/s
doppler = F0 * robot_speed / speed_of_sound;
number_of_chirps = 1;
adc_sps = 64000;
dt = 1/adc_sps;
signal_T = 0.005;
noise_T = 0.1;
noise_aplitude = 10;

t = 0:dt:signal_T-dt;
t_total = 0:dt:(signal_T+2*noise_T)*number_of_chirps-dt;
t_size = size(t,2);
noise_t_size = round(noise_T/dt);

unshifted_chirp = chirp(t, F0, t(end), F1, 'linear', -90);
noisy_unshifted_chirp = unshifted_chirp + rand(1,t_size)* noise_aplitude - noise_aplitude/2;
shifted_chirp = chirp(t, F0 + doppler, t(end), F1 + doppler, 'linear', -90);
noisy_shifted_chirp = shifted_chirp + rand(1,t_size)* noise_aplitude - noise_aplitude/2;
noise = rand(1,noise_t_size)*noise_aplitude - noise_aplitude/2;
padded_chirp = [noise noisy_shifted_chirp noise];
unshifted_padded_chirp = [noise noisy_unshifted_chirp noise];
signal = repmat(padded_chirp, 1, number_of_chirps);
unshifted_signal = repmat(unshifted_padded_chirp, 1, number_of_chirps);

filter = chirp(t, F0, t(end), F1, 'linear', -90);
flen = size(filter,2);

shifted_y = zeros(1, size(signal,2) - flen);
unshifted_y = zeros(1, size(signal,2) - flen);
for i = (1:size(signal,2) - flen)
    window = signal(i:i+flen-1)';
    unshifted_window = unshifted_signal(i:i+flen-1)';
    shifted_y(1, i) = (filter * window) / 1000;
    unshifted_y(1, i) = (filter * unshifted_window) / 1000;
end

[unshifted_max_val, unshifted_detection] = max(unshifted_y);
[shifted_max_val, shifted_detection] = max(shifted_y);
unshifted_detection = unshifted_detection * dt;
shifted_detection = shifted_detection * dt;

fprintf("Chirping from %fHz to %fHz\n", F0, F1);
fprintf("Robot moving at %fm/s will cause shift of %fHz\n", robot_speed, doppler);
fprintf("Shifted Chirp will be from %fHz to %fHz\n", F0 + doppler, F1 + doppler);

disp("Start of chirp detected at");
disp(unshifted_detection);
disp("Start of doppler-shifted chirp detected at");
disp(shifted_detection);
disp("Error (seconds) cause by doppler shift");
disp(unshifted_detection - shifted_detection);
disp("Error (meters) cause by doppler shift");
disp((unshifted_detection - shifted_detection) * speed_of_sound);

figure;
subplot(3, 1, 1);
plot(signal);
plot(t_total, signal);
xlim([t_total(1), t_total(end)]);
title("signal");

subplot(3, 1, 2);
plot(t, filter, 'Color', 'r');
xlim([t_total(1), t_total(end)]);
title("filter");

subplot(3, 1, 3);
hold on;
plot(t_total(1:end-flen), unshifted_y, 'Color', 'g', 'DisplayName', 'Detection');
plot(t_total(1:end-flen), shifted_y, 'DisplayName', 'Detection with doppler shift');
plot(t_total(1:end-flen), (unshifted_y >= unshifted_max_val)*unshifted_max_val, 'DisplayName', 'Binary Detection');
plot(t_total(1:end-flen), (shifted_y >= shifted_max_val)*unshifted_max_val, 'DisplayName', 'Binary detection with doppler shift');
xlim([t_total(1), t_total(end)]);
legend('show');
title("detected signal");

