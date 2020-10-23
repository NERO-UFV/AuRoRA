%% Tarefa 1

% Limpa as vari�vies
clear all; 
close all;
warning off;
clc;

% Carrega o diret�rio corrente e subdiret�rios 

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Drone  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
try
    fclose(instrfindall);
end


%% Load Class
try     
    % Load Classes
    RI = RosInterface; 
    RI.rConnect;    
    B = Bebop2(1);        
    
    disp('### Load Class Success');
    
catch ME
    disp(' #### Load Class Issues ');
    disp('');
    disp(ME);
    rosshutdown;
    return;
end




tmax = 15;
t  = tic; % Tempo de simula��o
tc = tic; % Tempo de controle
tp = tic; % Tempo de plotagem e exibi��o
countWhile = 0;

%B.rGetSensorData;
%NavDataSub = rossubscriber('/bebop/odom','nav_msgs/Odometry');
%msg = rostopic('echo','/bebop/odom');
%showdetails(navData);

B.rTakeOff;
pause(2);

disp('  ');
disp('Enter loop Control....');
disp('  ');

 try    
    while toc(t) < tmax
        countWhile = countWhile + 1;  
        
        B.rGetSensorData;   
        showdetails(B.pOdom.Pose.Pose.Position);
        B.pOdom.Pose.Pose.Position.
    
        
        B.pSC.Ud(1) = 0; % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avan�a, Move frente para baixo
        B.pSC.Ud(2) = 0; % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
        B.pSC.Ud(3) = 0; % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
        B.pSC.Ud(4) = 0; % Angulo do drone [-1,1] (+) rotaciona para Direita em torno do Eixo Z 
        B.pSC.Ud(5) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda 
        B.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z 


        % Envia Comando para o Drone
        B.rCommand;
        
              
    end
   
 catch Exception    
      B.rLand;          
      disp("Pouso for�ado via comando da estrutura Try Catch");
      disp(toStringJSON(Exception));
      rosshutdown;
 end


  B.rLand;            
  rosshutdown;
  
  
  
  