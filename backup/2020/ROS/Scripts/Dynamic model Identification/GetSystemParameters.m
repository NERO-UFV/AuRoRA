%function output = GetSystemParameters(iFullFileName)
%GETSYSTEMPARAMETERS Summary of this function goes here
%   Detailed explanation goes here

%%
syms psi k1 k2 k3 k4 k5 k6 k7 k8 real;

F = [cos(psi)   -sin(psi)   0   0;
                 sin(psi)   cos(psi)    0   0;
                 0          0           1   0;
                 0          0           0   1];
Ku = [k1 0 0 0;
      0 k3 0 0;
      0 0 k5 0;
      0 0 0 k7];

Kv = [k2 0 0 0;
      0 k4 0 0;
      0 0 k6 0;
      0 0 0 k8];

            
            F1 = [k1*cos(psi)  -k3*sin(psi)    0   0;
                k1*sin(psi) k3*cos(psi)     0   0;
                0           0               k5  0;
                0           0               0   k7];
             
            F2 = [k2*cos(psi)   -k4*sin(psi)    0   0;
                k2*sin(psi)    k4*cos(psi)     0   0;
                0              0               k6  0;
                0              0               0   k8];

simplify(inv(F1))
simplify(inv(F1)*F2*inv(F))

simplify(inv(F*Ku))

simplify(inv(F*Ku)*F*Kv*inv(F))
%%
%% MATLAB
%Parametros
%Codigo origem : D:\Doutorado\Codigos\controladorPD-cvdrone-master - DIF DKF - LELIS\build\vs2010\Resultados
clc, clear, close all;

%% Load Data
%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'Dynamic model Identification';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

dataPath = strcat(pwd,'\Dynamic Model Identification\Data\');
wsName = 'WS_Calibracao.mat';

% % Carrega o ultimo arquivo simulado
% fileName = wsName;
% fullFileName = strcat(dataPath,fileName);
% load(fullFileName,'serialNumber');

% fileName = strcat('Calibracao_X',toStringJSON(serialNumber),'.mat');
% fileName = strcat('Calibracao_PSI101.mat');
% fileName = strcat('Calibracao_Z12 - V01 Long');
% fileName = strcat('Calibracao_X3 - Long.mat');
% fileName = strcat('Calibracao_Y3 - Long.mat');
% fileName = strcat('Calibracao_Z12 - V01 Long.mat');
%fileName = strcat('Calibracao_X81k02w04.mat');
%fileName = strcat('Calibracao_Y84k02w04.mat');
%fileName = strcat('Calibracao_Z69k02w03.mat');
% fileName = strcat('Calibracao_UFV_PSI_Ka=0.4_Kw=0.62832_Serial_4.mat');
% fileName = strcat('Calibracao_UFV_PSI_Ka=0.4_Kw=0.62832_Serial_76.mat');
fileName = strcat('TOPZERADASGALAXIAS_Calibracao_UFV_X_Ka=0.5_Kw=0.62832_Serial_73.mat');
%  fileName = strcat('TOPZERADASGALAXIAS_Calibracao_UFV_X_Ka=0.5_Kw=0.62832_Serial_73.mat');
% fileName = strcat('TOPZERADASGALAXIAS_Calibracao_UFV_Y_Ka=0.5_Kw=0.62832_Serial_71.mat');
% fileName = strcat('TOPZERADASGALAXIAS_Calibracao_UFV_Z_Ka=0.2_Kw=0.62832_Serial_74.mat');

% fileName = strcat('.mat');




fullFileName = strcat(dataPath,fileName);
%load(fullFileName,'comandos','poses','velocidades','tempo');
load(fileName,'comandos','poses','velocidades','tempo','Bebop');

% Normalizando o vetor de velocidades 
MaxXYSpeed = 2.5;
MaxZSpeed = 1;
MaxPSISpeed = 1.7453;
% 
% % velocidadesRecebidas = velocidades;
% % velocidades(:,1:2)   = velocidades(:,1:2)./MaxXYSpeed;
% % velocidades(:,3)     = velocidades(:,3)  ./MaxZSpeed;
% % velocidades(:,4)     = velocidades(:,4)  ./MaxPSISpeed;
% 
comandosAnt = comandos;
% comandos(:,1:2) = comandos(:,1:2) * MaxXYSpeed;
% comandos(:,3) = comandos(:,3) * MaxZSpeed;
% comandos(:,4) = comandos(:,4) * MaxPSISpeed;





%%

inicio=20;
fim=size(comandos,1)-20;
filtro = 2145;

joy_vx = comandos(inicio:fim,1);
joy_vy = comandos(inicio:fim,2);
joy_vz = comandos(inicio:fim,3);
joy_vr = comandos(inicio:fim,4);
%roll = data(inicio:fim,5);
%pitch = data(inicio:fim,6);
yaw= poses(inicio:fim,4);
vx= velocidades(inicio:fim,1);
vy= velocidades(inicio:fim,2);
vz= velocidades(inicio:fim,3);
vpsi = velocidades(inicio:fim,4);

dt= tempo(inicio+1:fim) - tempo(inicio:fim-1);
z= poses(inicio:fim,4);


inicio = 1;
fim=size(joy_vx,1);

acx= (vx(inicio+1:fim)-vx(inicio:fim-1))/ mean(dt); %Uma menira simples de calcular a aceleracao do drone (dps verificar se está de acordo)
acy= (vy(inicio+1:fim)-vy(inicio:fim-1))/ mean(dt); 
acz= (vz(inicio+1:fim)-vz(inicio:fim-1))/ mean(dt); 

acpsi= (vpsi(inicio+1:fim)-vpsi(inicio:fim-1))/ mean(dt); 


mean(dt)
%%
disp('Doutorado');
%%
cor = [0 0 1];
t = 0;
k=1;

PSI{k} = zeros(4,8);
prc{k} = zeros(8,1);
parametros{k} = zeros(8,1);
G1{k} = zeros(1,2);
G2{k} = zeros(1,2);
G3{k} = zeros(1,2);
G4{k} = zeros(1,2);  

u(1,k) = 0;
u(2,k) = 0;
u(3,k) = 0;
u(4,k) = 0;

tempo(k) = t;


%joy = vrjoystick(1,'forcefeedback');

pitch_ant = 0;
roll_ant = 0;


Yfp1 = 0;
Yfp2 = 0;
Yfp3 = 0;
Yfp4 = 0;
    
    
Tfp1 = G1{1};
Tfp2 = G2{1};
Tfp3 = G3{1};
Tfp4 = G4{1};

Tfp = PSI{1};


Theta1 = classRLS(2);
    Theta1.inicializa(Yfp1(1),G1{1});
    
Theta2 = classRLS(2);
    Theta2.inicializa(Yfp2(1),G2{1});
    
Theta3 = classRLS(2);
    Theta3.inicializa(Yfp3(1),G3{1});
    
Theta4 = classRLS(2);
    Theta4.inicializa(Yfp4(1),G4{1});

Yfp = vertcat(u(1,1),u(2,1),u(3,1),u(4,1));
    
%%
%for k=2:size(comandos,1)-1        
for k=2:size(joy_vx,1)-1        
    u(1,k) = joy_vx(k);
    u(2,k) = joy_vy(k);
    u(3,k) = joy_vz(k);
    u(4,k) = joy_vr(k);    


    
   %[parametros{k}, G1{k}, G2{k}, G3{k}, G4{k}, prc{k}, PSI{k}] = CalculaParametros([Milton.robot_acelx(end);Milton.robot_acely(end);Milton.robot_acelz(end);Milton.robot_acelpsi(end)],[Milton.robot_vx(end);Milton.robot_vy(end);Milton.robot_vz(end);Milton.robot_vpsi(end)], Milton.robot_psi(end), uTrajetoria);
   [parametros{k}, G1{k}, G2{k}, G3{k}, G4{k}, prc{k}, PSI{k}] = CalculaParametros([acx(k);acy(k);acz(k);acpsi(k)],[vx(k);vy(k);vz(k); vpsi(k)], yaw(k), [joy_vx(k); joy_vy(k); joy_vz(k); joy_vr(k)]);
      
   Theta1.atualiza(u(1,k),G1{k});    
   Theta2.atualiza(u(2,k),G2{k});    
   Theta3.atualiza(u(3,k),G3{k});    
   Theta4.atualiza(u(4,k),G4{k});  
    
   
   Yfp = vertcat(Yfp, u(1,k),u(2,k),u(3,k),u(4,k));
   Tfp1 = vertcat(Tfp1, G1{k});    
   Tfp2 = vertcat(Tfp2, G2{k});    
   Tfp3 = vertcat(Tfp3, G3{k});    
   Tfp4 = vertcat(Tfp4, G4{k});    
    
   Tfp = vertcat(Tfp, PSI{k}); 
end

%%
theta = zeros(8,1);
parametrosk = zeros(8,1);
%%
theta = inv(Tfp'*Tfp)*Tfp'*Yfp;

k1 = 1/theta(1);
k2 = k1*theta(2);
k3 = 1/theta(3);
k4 = k3*theta(4);
k5 = 1/theta(5);
k6 = k5*theta(6);
k7 = 1/theta(7);
k8 = k7*theta(8);

disp('Acoplado');
disp(['k1=' num2str(k1) ' k2=' num2str(k2)]);
disp(['k3=' num2str(k3) ' k4=' num2str(k4)]);
disp(['k5=' num2str(k5) ' k6=' num2str(k6)]);
disp(['k7=' num2str(k7) ' k8=' num2str(k8)]);
%%
theta1 = inv(Tfp1'*Tfp1)*Tfp1'*Yfp1;
k1 = 1/theta1(1);
k2 = k1*theta1(2);
%
theta2 = inv(Tfp2'*Tfp2)*Tfp2'*Yfp2;
k3 = 1/theta2(1);
k4 = k3*theta2(2);

theta3 = inv(Tfp3'*Tfp3)*Tfp3'*Yfp3;
k5 = 1/theta3(1);
k6 = k5*theta3(2);

theta4 = inv(Tfp4'*Tfp4)*Tfp4'*Yfp4;
k7 = 1/theta4(1);
k8 = k7*theta4(2);

disp('Desacoplado');
disp(['k1=' num2str(k1) ' k2=' num2str(k2)]);
disp(['k3=' num2str(k3) ' k4=' num2str(k4)]);
disp(['k5=' num2str(k5) ' k6=' num2str(k6)]);
disp(['k7=' num2str(k7) ' k8=' num2str(k8)]);
%%
figure,
plot(Theta1.T(1,:)), hold on
plot(Theta1.T(2,:)), 
legend({['\theta_1 = '  num2str(Theta1.T(1,end))], ['\theta_2 = '  num2str(Theta1.T(2,end))]},'FontSize',14);
% title('Variando X')
grid on
axis tight
%
figure,
plot(Theta2.T(1,:)), hold on
plot(Theta2.T(2,:)), 
legend({['\theta_3 = '  num2str(Theta2.T(1,end))], ['\theta_4 = '  num2str(Theta2.T(2,end))]},'FontSize',14);
% title('Variando Y')
grid on
axis tight
%
figure,
plot(Theta3.T(1,:)), hold on
plot(Theta3.T(2,:)), 
legend({['\theta_5 = '  num2str(Theta3.T(1,end))], ['\theta_6 = '  num2str(Theta3.T(2,end))]},'FontSize',14);
% title('Variando Z')
grid on
axis tight
%
figure,
plot(Theta4.T(1,:)), hold on
plot(Theta4.T(2,:)), 
legend({['\theta_7 = '  num2str(Theta4.T(1,end))], ['\theta_8 = '  num2str(Theta4.T(2,end))]},'FontSize',14);
% title('Variando \psi')
grid on
axis tight


% prompt = 'Type any key to continue! ';
% aux = input(prompt);


% Retorno da Funcao
%output = [theta1 theta2 theta3 theta4];



%end


