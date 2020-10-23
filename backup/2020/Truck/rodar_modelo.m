%% Modelo para supor velocidade linear, dada uma velocidade angular
% Alexandre e Thiago (02/04/2020)
clearvars
close all
clc

load('modelo.mat')
load('v2.mat')
load('w2.mat')
load('v.mat')
load('w.mat')
x = linspace(.1,2*pi/3,500);
yfit = modelo.predictFcn(x);

figure
plot(x,yfit,'LineWidth',1.5)
hold on
plot(w,v,'ro','MarkerSize',2,'LineWidth',3)
plot(linspace(.1,1,15),ones(1,15).*0.75,'r.:','LineWidth',2)
plot(ones(1,15).*.1,linspace(.075,.75,15),'r.:','LineWidth',2)
plot(w2,v2,'r.:','LineWidth',2)
xlim([0,1.2])
ylim([0,1])
hold off
title('Velocidade com Carreta')
xlabel('Angular [rad/s]')
ylabel('Linear [m/s]')
grid on