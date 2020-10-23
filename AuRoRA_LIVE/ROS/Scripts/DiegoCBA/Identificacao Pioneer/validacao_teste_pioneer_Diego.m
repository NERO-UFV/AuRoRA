%%
%% MATLAB
%Parametros
%Codigo origem : D:\Doutorado\Codigos\controladorPD-cvdrone-master - DIF DKF - LELIS\build\vs2010\Resultados
clc, clear, close all;

%% Load Data
% % Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

dataPath = strcat(pwd,'\ROS\Scripts\DiegoCBA\Identificacao Pioneer\');

% % Carrega o ultimo arquivo simulado
fileName = strcat('Diego_validacao9_Data_20200706T160931.mat');  
fullFileName = strcat(dataPath,fileName);   
load(fileName,'t_hist','u_w_d_hist','u_w_hist','u_p_hist','w_p_hist');


% theta_1 = 0.23025;
% theta_2 = 0.22615;
% theta_3 = 0.00028953;
% theta_4 = 0.95282;
% theta_5 = 0.021357;
% theta_6 = 0.95282;

theta_1 = 0.22475;
theta_2 = 0.19672;
theta_3 = 0.0026587;
theta_4 = 0.97439;
theta_5 = 0.026025;
theta_6 = 0.93105;

U_hist = [];

Theta = [theta_1;theta_2;theta_3;theta_4;theta_5;theta_6];

for k=1:size(u_w_d_hist,2)
G = [u_p_hist(1,k) 0 -u_w_hist(2,k)^2 u_w_hist(1,k) 0 0; 0 w_p_hist(1,k) 0 0 u_w_hist(1,k)*u_w_hist(2,k) u_w_hist(2,k)];
U = G*Theta;
U_hist = [U_hist U];
end

figure();
hold on;
grid on;
plot(t_hist(1,:), U_hist(1,:));
plot(t_hist(1,:), u_w_d_hist(1,:));
legend('estimado', 'real');
title('validacao');
xlabel('t');
ylabel('velocidade linear referencia');

figure();
hold on;
grid on;
plot(t_hist(1,:), U_hist(2,:));
plot(t_hist(1,:), u_w_d_hist(2,:));
legend('estimado', 'real');
title('validacao');
xlabel('t');
ylabel('velocidade angular referencia');