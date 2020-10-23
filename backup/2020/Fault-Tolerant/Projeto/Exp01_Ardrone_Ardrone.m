clear; close all; clc;
try
    fclose(instrfindall);
catch
end

%
% % Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes

%% Load Class
    
    % Initiate classes
    A{1} = ArDrone;
    A{2} = ArDrone;
    
    A{1}.pPos.X(1:3) = [1 1 0.75];
    A{2}.pPos.X(1:3) = [0 0 0.75];
    
%% Plot


%% Variable initialization
data = [];

% Time variables initialization
Xd = [0 0 0 0];
dXd = [0 0 0 0];

fprintf('\nStart..............\n\n');

A{1}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216  ]';
A{2}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216  ]';

pause(5);
disp('Taking Off End Time....');

% Figura da simulação
figure
hold on
grid on
A{1}.mCADplot;
A{2}.mCADplot;
axis([-3 3 -3 3 0 10])
% view([0 90])
pause

% timers
T_exp = 30;
T_run = 1/30;       % Tempo de amostragem do experimento
t_run = tic;
t_task = tic;
T_task = 13;
i_task = 0;
t_exp = tic;


t  = tic;

A{2}.pPar.ti = tic;
A{1}.pPar.ti = tic;
A{2}.pPar.Ts = 1/30;
A{1}.pPar.Ts = 1/30;

% Dados trajetória
rX = 1;           % [m]
rY = 1;           % [m]
T = 30;             % [s]
w = 2*pi/T;         % [rad/s]

    while toc(t) < T_exp
        
        if toc(t_run) > T_run
            
            t_run = tic;
            t_atual = toc(t);

            
%% TAREFA

%Trajetória   

%% Ciclo 1
if t_atual < T_exp
        
        Xd_1 = [rX*cos(w*t_atual);
            rY*sin(w*t_atual);
            1;
            0];
   
        Xd_2 = Xd_1;    
        Xd_2(3) = Xd_1(3) + 1.5;      

        dXd_1 = [-w*rX*sin(w*t_atual);
                w*rY*cos(w*t_atual);
                0;
                0];

        dXd_2 = dXd_1;

%% CONTROLE DINÂMICO DOS ROBÔS
% Atribuindo trajetória

        A{1}.pPos.Xd([1:3 6]) = Xd_2;
        A{2}.pPos.Xd([1:3 6]) = Xd_1;
        

        A{1}.pPos.Xd([7:9 12]) = dXd_2;
        A{2}.pPos.Xd([7:9 12]) = dXd_1;

        A{1}.rGetSensorData;
        A{2}.rGetSensorData;
% ArDrone
        A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});
        A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});

        A{2}.pPar.ti = tic;
        A{1}.pPar.ti = tic;
        
        A{1}.rSendControlSignals;
        A{2}.rSendControlSignals;
%         if t_atual > 3/4*T
%             A{2}.rLand;
%         end  
        
%% DATA            
            data = [  data  ; A{1}.pPos.Xd(1:3)'     A{1}.pPos.X(1:3)' ...
                              A{2}.pPos.Xd(1:3)'     A{2}.pPos.X(1:3)' zeros(6,1)'  ...
                              zeros(8,1)'  A{1}.pPar.Model_simp' t_atual];
            
            %         %   1 -- 3      4 -- 6     
            %         B{1}.pPos.Xd'  B{1}.pPos.X' 
            %
            %         %   7 -- 9     10 -- 12        13 -- 18
            %         B{2}.pPos.Xd'  B{2}.pPos.X' barL.pPos.X_load'
            %
            %         %  19 -- 26             27 -- 34          35
            %      B{1}.pPar.Model_simp  B{2}.pPar.Model_simp  toc(t) ];
            
        
A{1}.mCADplot;
A{2}.mCADplot;
drawnow
            
        end
    end

    end

plot3(data(:,4),data(:,5),data(:,6))
hold on
plot3(data(:,10),data(:,11),data(:,12))