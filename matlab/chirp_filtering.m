%% Chirp Filtering

clc;
clear;
close all;

%%

% construct a series of chirps with the desired properties
F0 = 20000;
F1 = 27000;
robot_speed = 3; % m/s
speed_of_sound = 343.2; % m/s
start_doppler = F0 * robot_speed / speed_of_sound;
end_doppler = F1 * robot_speed / speed_of_sound;
number_of_chirps = 1;
adc_sps = 96000; % ensure we're safely above Nyquist's
dt = 1/adc_sps;
signal_T = 0.005;
padding_T = 0.1;
noise = 10;
signal_scale = 1;

t = 0:dt:signal_T-dt;
t_total = 0:dt:signal_T + 2 * padding_T - dt;
padding_t_size = round(padding_T/dt);

% Original Chirp
unshifted_chirp = chirp(t, F0, t(end), F1, 'linear', -90) * signal_scale;
% Doppler Shifted Chirp
shifted_chirp = chirp(t, F0 + start_doppler, t(end), F1 + end_doppler, 'linear', -90) * signal_scale;

% Add padding
padding = zeros(1, padding_t_size);
shifted_padded = [padding shifted_chirp padding];
unshifted_padded = [padding unshifted_chirp padding];

% Add noise
noisey_shifted_signal = shifted_padded + rand(size(shifted_padded)) * noise - noise/2;
noisey_unshifted_signal = unshifted_padded + rand(size(unshifted_padded)) * noise - noise/2;

repeated_shifted_signal = repmat(noisey_shifted_signal, 1, number_of_chirps);
repeated_unshifted_signal = repmat(noisey_unshifted_signal, 1, number_of_chirps);

filter = chirp(t, F0, t(end), F1, 'linear', -90);
flen = size(filter,2);

shifted_matched = zeros(1, size(repeated_shifted_signal,2) - flen);
unshifted_matched = zeros(1, size(repeated_shifted_signal,2) - flen);
for i = (1:size(repeated_shifted_signal,2) - flen)
    window = repeated_shifted_signal(i:i+flen-1)';
    unshifted_window = repeated_unshifted_signal(i:i+flen-1)';
    shifted_matched(1, i) = (filter * window) / 1000;
    unshifted_matched(1, i) = (filter * unshifted_window) / 1000;
end

[unshifted_max_val, unshifted_detection] = max(unshifted_matched);
[shifted_max_val, shifted_detection] = max(shifted_matched);
unshifted_detection = unshifted_detection * dt;
shifted_detection = shifted_detection * dt;

fprintf("Chirping from %fHz to %fHz\n", F0, F1);
fprintf("Robot moving at %fm/s will cause shift of ~%fHz\n", robot_speed, start_doppler);
fprintf("Shifted Chirp will be from %fHz to %fHz\n", F0 + start_doppler, F1 + end_doppler);

disp("Start of chirp detected at");
disp(unshifted_detection);
disp("Start of doppler-shifted chirp detected at");
disp(shifted_detection);
disp("Error with no shift (seconds)");
disp(unshifted_detection - padding_T);
disp("Error with no shift (meters)");
disp((unshifted_detection - padding_T) * speed_of_sound);
disp("Error with dpppler shift (seconds)");
disp(shifted_detection - padding_T);
disp("Error with dpppler  shift (meters)");
disp((shifted_detection - padding_T) * speed_of_sound);

figure;
plot(shifted_padded);
title("Unshifted, No-Noise, Chirp Signal");

figure;
plot(unshifted_chirp(1:20));
title("Unshifted, No-Noise, Chirp Signal (close up)");

figure;
plot(filter);
title("Filter used for matching");

figure;
hold on;
plot(t_total, repeated_unshifted_signal, 'DisplayName', 'Final Signal');
plot(t_total, repeated_shifted_signal, 'DisplayName', 'Finall Shifted Signal');
xlim([t_total(1), t_total(end)]);
legend('show');
title("Final Signal");

figure;
hold on;
plot(t_total(1:end-flen), unshifted_matched, 'DisplayName', 'Pattern-Matched');
plot(t_total(1:end-flen), shifted_matched, 'DisplayName', 'Pattern-Matched on Shifted Signal');
xlim([t_total(1), t_total(end)]);
legend('show');
title("Signal Convolved with Pattern");

