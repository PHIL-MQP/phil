%% Chirp Filtering

clc;
clear;
close all;

%%

% construct a series of chirps with the desired properties
doppler = 0;
F0 = 1000;
F1 = 1000;
number_of_chirps = 1;
T = 0:0.000001:0.01;
c = chirp(T, F0 + doppler, T(end), F1 + doppler, 'linear', -90);
noise = rand(1,5000)*0.05;
padded_chirp = [noise c noise];
signal = repmat(padded_chirp, 1, number_of_chirps);

filter = chirp(T, F0, T(end), F1, 'linear', -90);
flen = size(filter,2);

y = zeros(size(signal,2) - flen);
for i = (1:10:size(signal,2) - flen)
    window = signal(i:i+flen-1)';
    y(i) = (filter * window) / 1000;
end

figure;
hold on;
plot(signal);
% plot(filter);
plot(y);
