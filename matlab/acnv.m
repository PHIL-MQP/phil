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
%   2. When this script is run, two function f(t) and go(t) are convolved
%      and the output figure will show animated graphical convolution.
%
%   3. The functions "f" and "go" and their range of interval can be changed
%      by editing this script at line numbers around "48 to 64"
% 
%   4. Note:  For a better scaled plots of the functions f(t) and go(t1), 
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

help acnv;

% color of axis constant
  axis_color= [0.5 0.5 0.5];

% sampling interval constant
  s_int = 0.1;

% interval for function 'f(t)'
  t = [ -10:s_int:10 ];

% definition of function 'f(t)'
   f = 0.1*(t.^2);
%  f = 5*ones(1, length(t)); 
%  f = t;

% interval for function 'go(t1)'
  t1 = [-7:s_int:7];

% definition of function 'go(t1)'
go = -0.1*(t1.^2);
% go = .1*(t1.^3);
% go = 5*cos(2*pi*t1);
% go = 5*ones(1, length(t1));
% go = zeros(1, length(t1));go(1)=5;


% convolve: note the multiplation by the sampling interval
  c = s_int * conv(f, go);

% Animation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% flip 'go(t1)' for the graphical convolutions g = go(-t1)
  g = fliplr(go);
  tf = fliplr(-t1);

% slide range of 'g' to discard non-ovelapping areas with 'f' in the convolution
  tf = tf + ( min(t)-max(tf) );

% get the range of function 'c' which is the convolution of 'f(t)' and 'go(t1)'
  tc = [ tf t(2:end)];
  tc = tc+max(t1);

% start graphical output with three subplots
  a_fig = figure;
  set(a_fig, 'Name', 'Animated Convolution', 'unit', 'pixel', ...
             'Position', [300, 150, 600, 750]);

% plot f(t) and go(t1)  
  ax_1 = subplot(3,1,1);
  op = plot(t,f, 'b',  t1, go, 'r');
  hold on; grid on;
  set(ax_1, 'XColor', axis_color, 'YColor', axis_color, 'Color', 'w', 'Fontsize', 9);
  xlim( [ ( min(t)-abs(max(tf)-min(tf)) - 1 ) ( max(t)+abs(max(tf)-min(tf)) + 1 ) ] );
  title('Graph of f(t) and go(t)', 'Color', axis_color );
  legend({'f(t)' 'go(t)'});

% initialize animation the plot of 'g' is slided over the plot of 'f'

% plot f in the subplot number 2
  ax_2 = subplot(3,1,2);
  p = plot(t, f);
  hold on; grid on;
  title('Graphical Convolution: f(t) and g = go(-t1)', 'Color', axis_color );
  
% plot g in the subplot number 2
  q = plot(tf, g, 'r');
  xlim( [ ( min(t)-abs(max(tf)-min(tf))-1 ) ( max(t)+abs(max(tf)-min(tf))+1 ) ] );
  u_ym = get(ax_2, 'ylim');

% plot two vertical lines to show the range of ovelapped area
  s_l = line( [min(t) min(t)], [u_ym(1) u_ym(2)], 'color', 'g'  );
  e_l = line( [min(t) min(t)], [u_ym(1) u_ym(2)], 'color', 'g'  );
  hold on; grid on;
  set(ax_2, 'XColor', axis_color, 'YColor', axis_color, 'Color', 'w', 'Fontsize', 9);

  % put a yellow shade on ovelapped region
  sg = rectangle('Position', [min(t) u_ym(1) 0.0001 u_ym(2)-u_ym(1)], ...
                 'EdgeColor', 'w', 'FaceColor', 'y');
  drawnow;
  
  
% initialize the plot the convolution result 'c'
  ax_3 = subplot(3,1,3);
  r = plot(tc, c);
  grid on; hold on;
  set(ax_3, 'XColor', axis_color, 'YColor', axis_color, 'Fontsize', 9);
  % xlim( [ min(tc)-1 max(tc)+1 ] );
  xlim( [ ( min(t)-abs(max(tf)-min(tf)) - 1 ) ( max(t)+abs(max(tf)-min(tf)) + 1 ) ] );
  title('Convolutional Product c(t)', 'Color', axis_color );

% animation block
  for i=1:length(tc)
    
    % control speed of animation minimum is 0, the lower the faster
      pause(0.01);
      drawnow;
      
    % update the position of sliding function 'g', its handle is 'q'
      tf=tf+s_int;
      drawnow;
      set(q,'XData',tf,'YData',g);

    % show overlapping regions
    
    % show a vetical line for a left boundary of overlapping region
      sx = min( max( tf(1), min(t) ), max(t) );  
      sx_a = [sx sx];
      drawnow;
      set(s_l, 'XData', sx_a);

    % show a second vetical line for the right boundary of overlapping region
      ex = min( tf(end), max(t) );  
      ex_a = [ex ex];
      drawnow;
      set(e_l, 'XData', ex_a);
      
    % update shading on ovelapped region
      rpos = [sx u_ym(1) max(0.0001, ex-sx) u_ym(2)-u_ym(1)];  
      set(sg, 'Position', rpos);
      
    % update the plot of convolutional product 'c', its handle is r
%       set(r,'EraseMode','xor');
      drawnow;
      set(r,'XData',tc(1:i),'YData',c(1:i) );
    
  end;
%
% end of acnv %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
