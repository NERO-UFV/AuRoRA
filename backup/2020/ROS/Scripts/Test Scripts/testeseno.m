clear all
close all
clc

w = 20*2*pi;
% w = 0.5;

t = 0:0.001:1;

s = sin(w*t);

plot(t,s)

fh = w/(2*pi)

T = 1/fh