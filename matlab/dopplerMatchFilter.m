clear all;
close all;
clc;

Fs = 96000;
f_tx = 2000;
f_sweep = 200;
t_sweep = 0.1;
f_offset = 10;
t=0:1/Fs:1;
t_chirp = 0:1/Fs:t_sweep;
sig = vco((sawtooth(2*pi*(1/t_sweep)*t,.5)),[f_tx f_tx+f_sweep],Fs);
sig_up = vco((sawtooth(2*pi*(1/t_sweep)*t,.5)),[f_offset+f_tx f_tx+f_sweep+f_offset],Fs);

%figure;
%spectrogram(sig);

chirp = sig(1:length(t_chirp));

%figure;
%plot(t_chirp,chirp);

fil_out = filter(chirp,1,sig);
fil_out_up = filter(chirp,1,sig_up);

figure;
plot(t,fil_out,t,fil_out_up)