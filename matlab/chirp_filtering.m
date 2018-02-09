%% Chirp Filtering

clc;
clear;
close all;

%%

% construct a series of chirps with the desired properties
F0 = 500;
F1 = 2000;
doppler = 0.01*F0;
number_of_chirps = 1;
dt = 0.000001;
T = 0.02;
noise_T = 0.02;
t = 0:dt:T;
t_total = 0:dt:T+2*noise_T;
unshifted_chirp = chirp(t, F0, t(end), F1, 'linear', -90);
c = chirp(t, F0, t(end), F1 + doppler, 'linear', -90);
noise = rand(1,noise_T/dt)*0.05;
padded_chirp = [noise c noise];
unshifted_padded_chirp = [noise unshifted_chirp noise];
signal = repmat(padded_chirp, 1, number_of_chirps);
unshifted_signal = repmat(unshifted_padded_chirp, 1, number_of_chirps);

filter = chirp(t, F0, t(end), F1, 'linear', -90);
flen = size(filter,2);

y = zeros(1, size(signal,2) - flen);
unshifted_y = zeros(1, size(signal,2) - flen);
for i = (1:size(signal,2) - flen)
    window = signal(i:i+flen-1)';
    unshifted_window = unshifted_signal(i:i+flen-1)';
    y(1, i) = (filter * window) / 1000;
    unshifted_y(1, i) = (filter * unshifted_window) / 1000;
end

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
plot(t_total(1:end-flen), y);
plot(t_total(1:end-flen), unshifted_y, 'Color', 'g');
xlim([t_total(1), t_total(end)]);

