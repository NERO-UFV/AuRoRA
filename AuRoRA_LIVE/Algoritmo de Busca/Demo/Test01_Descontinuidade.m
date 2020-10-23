clear all
close all
clc

p = linspace(0,2*pi,250);
X = cos(p);
Y = sin(p);
Z = sin(2*p);


figure
plot3(X,Y,Z)
grid on
axis equal
