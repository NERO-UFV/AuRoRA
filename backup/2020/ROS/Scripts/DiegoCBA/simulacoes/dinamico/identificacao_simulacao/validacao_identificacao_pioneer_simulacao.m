%%
%% MATLAB
%Parametros
%Codigo origem : D:\Doutorado\Codigos\controladorPD-cvdrone-master - DIF DKF - LELIS\build\vs2010\Resultados
clc, clear, close all;

% % Carrega o ultimo arquivo simulado
fileName = strcat('dinamicoDiego_validacao_simulacao190_Data_20200721T184314');    
load(fileName,'t_hist','u_w_d_hist','u_w_hist','u_p_hist','w_p_hist');

%%%% Conjunto de parametros gerados no processo de identificação
% theta_1 = 0.23025;
% theta_2 = 0.22615;
% theta_3 = 0.00028953;
% theta_4 = 0.95282;
% theta_5 = 0.021357;
% theta_6 = 0.95282;

theta_1 = 0.23081;
theta_2 = 0.12762;
theta_3 = 0.0013423;
theta_4 = 0.95256;
theta_5 = 0.021364;
theta_6 = 0.95281;

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