%%

clc;
clear;
close;
format short;

%%
syms ayz azy azx sax say saz bax bay baz;
syms aix aiy aiz;
params = [ayz azy azx sax say saz bax bay baz];
accel = [aix aiy aiz];
ex_param_values = [0.1 0 0.1 1 1 1 0 0.1 0];
ex_accel = [0.2 0.7 0.1];

T = [1 -ayz azy; 0 1 -azx; 0 0 1];
K = [sax 0 0; 0 say 0; 0 0 saz];
b = [bax; bay; baz];
ai = [aix; aiy; aiz];
g = 1;
f= g^2 - norm(T * K *(ai + b))^2;

partials = jacobian(f, [ayz azy azx sax say saz bax bay baz]);
analytic_d = subs(partials, [params, accel], [ex_param_values, ex_accel]);
disp("Analytic Derivatives:");
disp(vpa(analytic_d));