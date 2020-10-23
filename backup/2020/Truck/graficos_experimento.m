%% Código de gráfico do experimento  
clc;
clearvars;
close all;
v=linspace(0.075, 0.75,10);
w=linspace(0.1,1,10);
r= v./w';

r(2:end,1)=0;
r(4:end,2)=0;
r(5:end,3)=0;
r(7:end,4)=0;
r(8:end,5)=0;
r(9:end,6)=0;
r(10,7)=0;

surf(v,w,r);
xlabel('Velocidade linear')
ylabel('Velocidade angular')
zlabel('raio')

