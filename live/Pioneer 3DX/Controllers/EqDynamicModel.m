clear all
close all
clc

syms a b c d real
syms kpu kdu kpw kdw real
syms m I J B r R real
syms ka kb real
syms w real

Ma = [(m+J/r^2+2*kdu*ka/r/R), -m*c; ...
    (c*J/r^2+2*c*kdu*ka/r/R), (I+m*b^2+J*d^2/2/r^2+ka*kdw*d/r/R)]

Ca = [(ka*kb/r^2/R+B/r^2+2*kpu*ka/r/R), -m*b*w; ...
    (c*ka*kb/r^2/R+c*B/r^2+2*c*kpu*ka/r/R+m*b*w), (ka*kb*d^2/2/R/r^2+B*d^2/2/r^2+d*kpw*ka/r/R-m*b*c*w)]

A = [2*ka*kpu/r/R 0; 2*c*ka*kpu/r/R, d*ka*kpw/r/R]

pretty(simplify(A\Ma))

pretty(simplify(A\Ca))

B = [2*kpu/d/kpw, 0; 0 1];

pretty(simplify(B*A\Ma))

