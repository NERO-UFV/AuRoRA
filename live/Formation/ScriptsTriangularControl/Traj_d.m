clear
close all
clc
%% Trajetória desejada

T = 60;
Tf = 60;
rX = 1.0;       % [m]
rY = 1.0;       % [m]

w = 2*pi/T;     % [rad/s]

t_traj = 0:0.01:Tf;

a = 3.*(t_traj/Tf).^2 - 2.*(t_traj/Tf).^3;
tp = a*Tf;

xG = rX*sin(w*tp);
yG = rY*sin(2*w*tp);


plot(xG, yG)