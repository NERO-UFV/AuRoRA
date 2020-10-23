clear
close all
clc

%% Squircle - Superellipse com n = 4

% Parametros da curva
n = 4;
a = 1;
b = 1;

t = 0:0.01:2*pi;


x = abs(cos(t)).^(2/n)*a.*sign(cos(t));
y = abs(sin(t)).^(2/n)*a.*sign(sin(t));

plot(x,y)