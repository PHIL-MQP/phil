% %%%%%%%%%%%%%%%%%%%%% Function: acnv.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Title: Animation of Graphical Convolution
%
%  To run this script just type 'acnv' on the MatLab prompt: > acnv
%
%  Description:
%   1. This is a simple MatLab demo to animate the process of convolution.
%      It is meant to help student to visualize how convolution works.
%
%   2. When this script is run, two function f(t) and g(t) are convolved
%      and the output figure will show animated graphical convolution.
%
%   3. The functions "f" and "g" and their range of interval can be changed
%      by editing this script at line numbers around "48 to 64"
%
%   4. Note:  For a better scaled plots of the functions f(t) and g(t1),
%             it is recommended to set the functions such that their
%             maximum value remains comparable. e.g one can use appropriate
%             scaling. Other functions are also given 'commented out'
%
%             Interger values are recommended for the intervals
%
%   5. The animation can be made faster or slower by changing the value of
%      the pause function in the animation loop. (around line number 134)
%
%  Author:
%      Laine Berhane Kahsay
%      Uni-Ulm, Germany
%
%   email: kahsay_2004@yahoo.com
%
%     ver: 1.0, written in Matlab 6.5/7.0
%
%  To see this help - type on the Matlab Prompt: > help acnv
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
clc;
clear;
close all;

help acnv;

% color of axis constant
axis_color= [0.5 0.5 0.5];

% sampling interval constant
s_int = 0.01;

% interval for function 'f(t)'
t = -10:s_int:10;
t_chirp = -5:s_int:5;
doppler = 0;
F0 = 2;
F1 = 2;
filter = chirp(t_chirp, F0 + doppler, t_chirp(end), F1 + doppler, 'linear', -90);
noise = rand(1,500)*0.05;
padded_chirp = [noise filter noise];


% definition of function 'f(t)'
f = padded_chirp;

% interval for function 'g(t1)'
t1 = t_chirp;

% definition of function 'g(t1)'
g = filter;

% convolve: note the multiplation by the sampling interval
c = s_int * conv(f, g, 'same');

% Animation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% start graphical output with three subplots
a_fig = figure;
set(a_fig, 'Name', 'Animated Convolution', 'unit', 'pixel', ...
    'Position', [300, 150, 600, 750]);

% initialize animation the plot of 'g' is slided over the plot of 'f'

% plot f in the subplot number 2
ax_2 = subplot(2,1,1);
p = plot(t, f);
hold on; grid on;
title('Graphical Convolution: f(t) and g(t)', 'Color', axis_color );

% plot g in the subplot number 2
q = plot(t1, g, 'r');
xlim([t(1), t(end)]);
u_ym = get(ax_2, 'ylim');

% plot two vertical lines to show the range of ovelapped area
s_l = line( [min(t) min(t)], [u_ym(1) u_ym(2)], 'color', 'g'  );
e_l = line( [min(t) min(t)], [u_ym(1) u_ym(2)], 'color', 'g'  );
hold on; grid on;

% initialize the plot the convolution result 'c'
ax_3 = subplot(2,1,2);
r = plot(t, c);
grid on; hold on;
xlim([t(1), t(end)]);
title('Convolutional Product c(t)', 'Color', axis_color );

% animation block
tf = t(1):s_int:size(tf,2);
for i=1:length(t)
    
    % control speed of animation minimum is 0, the lower the faster
%     pause(1);
    
    % update the position of sliding function 'g', its handle is 'q'
    tf=tf+s_int;
    set(q,'XData',tf,'YData',g);
    
    % show a vetical line for a left boundary of overlapping region
    sx = min( max( tf(1), min(t) ), max(t) );
    sx_a = [sx sx];
    set(s_l, 'XData', sx_a);
    
    % show a second vetical line for the right boundary of overlapping region
    ex = min( tf(end), max(t) );
    ex_a = [ex ex];
    set(e_l, 'XData', ex_a);
    
    % update the plot of convolutional product 'c', its handle is r
    set(r,'XData',t(1:i),'YData',c(1:i) );
    
    drawnow;
end
%
% end of acnv %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
