%% GRÁFICO w x v 

%% CLEARS
    clc;
    clearvars;
    close all;
 
 %% ENTRADA DE DADOS
    w=[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.4,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.7,0.7,0.7,0.7,0.7,0.7,0.8,0.8,0.8,0.8,0.8,0.9,0.9,0.9,0.9,1,1,1];
    v=[0.075,0.15,0.225,0.3,0.375,0.45,0.525,0.6,0.675,0.75,0.15,0.225,0.3,0.375,0.45,0.525,0.6,0.675,0.75,0.15,0.225,0.3,0.375,0.45,0.525,0.6,0.675,0.75,0.225,0.3,0.375,0.45,0.525,0.6,0.675,0.75,0.3,0.375,0.45,0.525,0.6,0.675,0.75,0.3,0.375,0.45,0.525,0.6,0.675,0.75,0.375,0.45,0.525,0.6,0.675,0.75,0.45,0.525,0.6,0.675,0.75,0.525,0.6,0.675,0.75,0.6,0.675,0.75];

    load('w2.mat')
    load('v2.mat')

%% GRÁFICO
    figure(1)
    plot(w,v,"b--o")
    ylabel("Velocidade Linear")
    xlabel("Velocidade Angular")
    
%% REGRESSÃO EXPONENCIAL
x = linspace(.1,1.2,500);
a = -1.012;
b = 3.089;
c = 1.096;
d = 3.036;
fittedmodel = a*exp(b*x) + c*exp(d*x);

figure
plot(x,fittedmodel,'LineWidth',1.5)
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


