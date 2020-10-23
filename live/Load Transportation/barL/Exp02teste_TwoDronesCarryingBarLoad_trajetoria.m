%% 3D Line Formation Pioneer-Drone
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [ xf yf zf rho alfa beta ]
% Initial Comands

clear; close all; clc;
try
    fclose(instrfindall);
catch
end
%
% % Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

%% Load Classes

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    RI.rConnect('192.168.0.144');
    
    % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
    
    % Initiate classes
    B{1} = Bebop(1,'B1');
    B{2} = Bebop(2,'B2');
%     
%     L{1} = Load;
%     L{2} = Load;
%     idL{1} = getID(OPT,Load,3);
%     idL{2} = getID(OPT,Load,4);
    
    % Valores: exposure 330 threshold 150
%     rb = OPT.RigidBody;
%     if rb(idL{1}).isTracked
%         L{1} = getOptData(rb(idL{1}),L{1});
%     end
%     
%     if rb(idL{2}).isTracked
%         L{2} = getOptData(rb(idL{2}),L{2});
%     end
    
    %P = Pioneer3DX(1);  % Pioneer Instance
    
    % Joystick
    J = JoyControl;
    
  
    disp('################### Load Class Success #######################');
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end



%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);


%% Variable initialization
data = [];

rX = .5;           % [m]
rY = .5;           % [m]
T = 10;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]

% Time variables initialization
Xd = [0 0 0 0];
dXd = [0 0 0 0];

fprintf('\nStart..............\n\n');

pause(3)

B{1}.pPar.Model_simp = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295  ]';
B{2}.pPar.Model_simp = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295  ]';
%% Variable Initialization
barL.pPos.X = zeros(6,1);
barL.pPos.Xd = zeros(6,1);
barL.pPos.Xr = zeros(6,1);

barL.pPos.dXr = zeros(6,1);
barL.pPos.dXd = zeros(6,1);

barL.pPos.X_load = zeros(6,1);

barL.pPos.Qd = zeros(6,1);

%% Parâmetros da Carga barL 
% Comprimento dos cabos
barL.pPar.l1 = 1.05;
barL.pPar.l2 = barL.pPar.l1;

% Comprimento da barL
barL.pPar.L = 1.45;  
barL.pPar.L = 2;  


% Beboop
disp('Start Take Off Timming....');
B{1}.rTakeOff;
% B{2}.rTakeOff;
pause(3);
disp('Taking Off End Time....');

%% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
barL.pPos.Qd = [0 .5 .6 deg2rad(0) deg2rad(0) barL.pPar.L]';

% Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d
barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
barL.pPos.dXd = [dXd(1) dXd(2) dXd(3) dXd(1) dXd(2) dXd(3)]';

% Povoando a variável X e X_load da classe
barL.pPos.X = [B{1}.pPos.X(1:3); B{2}.pPos.X(1:3)];
% barL.pPos.X_load = [L{1}.pPos.X(1) L{1}.pPos.X(2) L{1}.pPos.X(3) L{2}.pPos.X(1) L{2}.pPos.X(2) L{2}.pPos.X(3)]';

% Cálculo do erro nos drones
barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;

% timers
T_exp = 120; % tempo de experimento
T_run = 1/30; % período de amostragem do experimento
t_run = tic;
t  = tic;

B{1}.pPar.ti = tic;
B{2}.pPar.ti = tic;

try
    while toc(t) < T_exp
        
        if toc(t_run) > T_run
            
            t_run = tic;
% %Trajetória           
                Xd = [rX*sin(w*toc(t));
                rY*cos(0.5*w*toc(t));
                0.5 ;
                0];
            
           
            dXd = [w*rX*cos(w*toc(t));
                -0.5*w*rY*sin(0.5*w*toc(t));
                0;
                0];
%             
% % Posição
%             dXd = [0;
%                 0;
%                 0;
%                 0];
%         
%             if toc(t) < 20
%                 
%                 Xd = [0;
%                     0;
%                     0.5;
%                     0];                        
%             end
%                     if toc(t) > 20
%                 
%                 Xd = [.5;
%                     .5;
%                     1;
%                     0];                        
%                     end
%                     
%                     if toc(t) > 40
%                 
%                 Xd = [-.5;
%                     -.5;
%                     0.5;
%                     0];                        
%             end
            

                
                %% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
                barL.pPos.Qd = [Xd(1) Xd(2) Xd(3) deg2rad(0) deg2rad(0) barL.pPar.L]';
                
                % Transformação inversa Q -> X, i.e., (xc,yc,zc,alpha,gamma,L)_d-> (x1,y1,z1,x2,y2,z2)_d
                barL.pPos.Xd(1) = barL.pPos.Qd(1) - cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(2) = barL.pPos.Qd(2) - cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(3) = barL.pPos.Qd(3) - sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l1;
                barL.pPos.Xd(4) = barL.pPos.Qd(1) + cos(barL.pPos.Qd(5))*sin(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(5) = barL.pPos.Qd(2) + cos(barL.pPos.Qd(5))*cos(barL.pPos.Qd(4))*barL.pPos.Qd(6)/2;
                barL.pPos.Xd(6) = barL.pPos.Qd(3) + sin(barL.pPos.Qd(5))*barL.pPos.Qd(6)/2 + barL.pPar.l2;
                barL.pPos.dXd = [dXd(1) dXd(2) dXd(3) dXd(1) dXd(2) dXd(3)]';
                

        % Povoando a variável X e X_load da classe
        barL.pPos.X = [B{1}.pPos.X(1:3); B{2}.pPos.X(1:3)];
%         barL.pPos.X_load = [L{1}.pPos.X(1) L{1}.pPos.X(2) L{1}.pPos.X(3) L{2}.pPos.X(1) L{2}.pPos.X(2) L{2}.pPos.X(3)]';

        % Cálculo do erro nos drones
        barL.pPos.Xtil = barL.pPos.Xd - barL.pPos.X;
    
        %% Atribuindo missão
        % Obter os dados dos sensores
        
        % Carga
%         rb = OPT.RigidBody;
%         if rb(idL{1}).isTracked
%             L{1} = getOptData(rb(idL{1}),L{1});
%         end
%         
%         if rb(idL{2}).isTracked
%             L{2} = getOptData(rb(idL{2}),L{2});
%         end
        
        % Bebop
        B{1}.rGetSensorDataOpt;
        B{2}.rGetSensorDataOpt;
        
        % Atribuindo trajetória
        B{1}.pPos.Xd(1:3) = barL.pPos.Xd(1:3);
        B{2}.pPos.Xd(1:3) = barL.pPos.Xd(4:6);
        
        B{1}.pPos.dXd(1:3) = barL.pPos.dXd(1:3);
        B{2}.pPos.dXd(1:3) = barL.pPos.dXd(4:6);
        


        % Controle
        
        B{1}.cInverseDynamicController_Compensador;
%         B{1}.cInverseDynamicController_Adaptativo;

%           B{2}.cInverseDynamicController_Compensador;
%         B{2}.cInverseDynamicController_Adaptativo;

          
        B{1}.pPar.ti = tic;
        B{2}.pPar.ti = tic;
            
            %% Save data
            
%             % Variable to feed plotResults function
            data = [  data  ; B{1}.pPos.Xd(1:3)'     B{1}.pPos.X(1:3)' ...
                              B{2}.pPos.Xd(1:3)'     B{2}.pPos.X(1:3)' zeros(6,1)'  ...
                              B{1}.pPar.Model_simp'  B{2}.pPar.Model_simp' toc(t)];
            
            %         %   1 -- 3      4 -- 6     
            %         B{1}.pPos.Xd'  B{1}.pPos.X' 
            %
            %         %   7 -- 9     10 -- 12        13 -- 18
            %         B{2}.pPos.Xd'  B{2}.pPos.X' barL.pPos.X_load'
            %
            %         %  19 -- 26             27 -- 34          35
            %      B{1}.pPar.Model_simp  B{2}.pPar.Model_simp  toc(t) ];
            
            
            % Beboop
            % Joystick Command Priority
            B{1} = J.mControl(B{1});                    % joystick command (priority)
            B{1}.rCommand;
            B{2} = J.mControl(B{2});                    % joystick command (priority)
            B{2}.rCommand;
            
            
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            if btnEmergencia ~= 0 || B{1}.pFlag.EmergencyStop ~= 0 || B{1}.pFlag.isTracked ~= 1
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg
                    disp("End Land Command");
                    B{1}.rCmdStop;
                    B{1}.rLand;
                end
%                 break;
            end
            
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            if btnEmergencia ~= 0 || B{2}.pFlag.EmergencyStop ~= 0 || B{2}.pFlag.isTracked ~= 1
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg
                    disp("End Land Command");
                    B{2}.rCmdStop;
                    B{2}.rLand;
                end
%                 break;
            end
          disp(toc(t_run)*1000)  
        end
    end
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B{1}.rCmdStop;
    disp('');
    disp(ME);
    disp('');
    B{1}.rLand
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B{2}.rCmdStop;
    disp('');
    disp(ME);
    disp('');
    B{2}.rLand
    
end

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B{1}.rCmdStop;
    B{1}.rLand
    
    disp("End Land Command");
    B{2}.rCmdStop;
    B{2}.rLand
end


% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

% Plot results
B1_Xtil = data(:,1:3) - data(:,4:6);
B2_Xtil = data(:,7:9) - data(:,10:12);
Load_Xtil = [data(:,1:3) - data(:,13:15) data(:,7:9) - data(:,16:18)];
Load_Xtil(:,[3 6]) = Load_Xtil(:,[3 6]) - barL.pPar.l1;


% 
figure();
hold on;
grid on;
plot(data(:,35),B1_Xtil(:,1));
plot(data(:,35),B1_Xtil(:,2));
plot(data(:,35),B1_Xtil(:,3));
title('Erro de Posição Q1');
legend('Pos X','Pos Y','Pos Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');

figure();
hold on;
grid on;
plot(data(:,35),B2_Xtil(:,1));
plot(data(:,35),B2_Xtil(:,2));
plot(data(:,35),B2_Xtil(:,3));
title('Erro de Posição Q2');
legend('Pos X','Pos Y','Pos Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');

figure();
hold on;
grid on;
plot(data(:,35),Load_Xtil(:,1));
plot(data(:,35),Load_Xtil(:,2));
plot(data(:,35),Load_Xtil(:,3));
title('Erro de Posição L1');
legend('Pos X','Pos Y','Pos Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');

figure();
hold on;
grid on;
plot(data(:,35),Load_Xtil(:,4));
plot(data(:,35),Load_Xtil(:,5));
plot(data(:,35),Load_Xtil(:,6));
title('Erro de Posição L2');
legend('Pos X','Pos Y','Pos Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');


% % 
% % figure();
% % hold on;
% % grid on;
% % plot(data(:,1),data(:,2));
% % plot(data(:,13),data(:,14));
% % title('XY');
% % xlabel('X [m]');
% % ylabel('Y [m]');
% % 
% % figure();
% % subplot(411)
% % hold on;
% % grid on;
% % plot(data(:,35),data(:,19));
% % plot(data(:,35),data(:,7));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [m/s]');
% % legend('dX', 'dXd');
% % 
% % subplot(412)
% % hold on;
% % grid on;
% % plot(data(:,35),data(:,20));
% % plot(data(:,35),data(:,8));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [m/s]');
% % legend('dY', 'dYd');
% % 
% % subplot(413)
% % hold on;
% % grid on;
% % plot(data(:,35),data(:,21));
% % plot(data(:,35),data(:,9));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [m/s]');
% % legend('dZ', 'dZd');
% % 
% % subplot(414)
% % hold on;
% % grid on;
% % plot(data(:,35),data(:,24));
% % plot(data(:,35),data(:,12));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [rad/s]');
% % legend('phi', 'dphi');
% % 
% % 
figure();
sgtitle('Q1')
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,4));
plot(data(:,end),data(:,1));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('X', 'Xd');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,5));
plot(data(:,end),data(:,2));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('Y', 'Yd');

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,6));
plot(data(:,end),data(:,3));
axis([0,70,1,2])
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('Z', 'Zd');


figure();
sgtitle('Q2')
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,10));
plot(data(:,end),data(:,7));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('X', 'Xd');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,11));
plot(data(:,end),data(:,8));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('Y', 'Yd');

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,12));
plot(data(:,end),data(:,9));
axis([0,70,1,2])
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('Z', 'Zd');

figure();
subplot(421)
grid on;
plot(data(:,35),data(:,19));
xlabel('Tempo[s]');
ylabel('$K_1$','interpreter','latex')
legend(['$K_1 = $' num2str(data(end,19))],'interpreter','latex')

subplot(422)
grid on;
plot(data(:,35),data(:,20));
xlabel('Tempo[s]');
ylabel('$K_2$','interpreter','latex')
legend(['$K_2 = $' num2str(data(end,20))],'interpreter','latex')

subplot(423)
grid on;
plot(data(:,35),data(:,21));
xlabel('Tempo[s]');
ylabel('$K_3$','interpreter','latex')
legend(['$K_3 = $' num2str(data(end,21))],'interpreter','latex')

subplot(424)
grid on;
plot(data(:,35),data(:,22));
xlabel('Tempo[s]');
ylabel('$K_4$','interpreter','latex')
legend(['$K_4 = $' num2str(data(end,22))],'interpreter','latex')

subplot(425)
grid on;
plot(data(:,35),data(:,23));
xlabel('Tempo[s]');
ylabel('$K_5$','interpreter','latex')
legend(['$K_5 = $' num2str(data(end,23))],'interpreter','latex')

subplot(426)
hold on;
grid on;
plot(data(:,35),data(:,24));
xlabel('Tempo[s]');
ylabel('$K_6$','interpreter','latex')
legend(['$K_6 = $' num2str(data(end,24))],'interpreter','latex')

subplot(427)
hold on;
grid on;
plot(data(:,35),data(:,25));
xlabel('Tempo[s]');
ylabel('$K_7$','interpreter','latex')
legend(['$K_7 = $' num2str(data(end,25))],'interpreter','latex')

subplot(428)
hold on;
grid on;
plot(data(:,35),data(:,26));
xlabel('Tempo[s]');
ylabel('$K_8$','interpreter','latex')
legend(['$K_8 = $' num2str(data(end,26))],'interpreter','latex')

figure();
subplot(421)
grid on;
plot(data(:,35),data(:,27));
xlabel('Tempo[s]');
ylabel('$K_1$','interpreter','latex')
legend(['$K_1 = $' num2str(data(end,27))],'interpreter','latex')

subplot(422)
grid on;
plot(data(:,35),data(:,28));
xlabel('Tempo[s]');
ylabel('$K_2$','interpreter','latex')
legend(['$K_2 = $' num2str(data(end,28))],'interpreter','latex')

subplot(423)
grid on;
plot(data(:,35),data(:,29));
xlabel('Tempo[s]');
ylabel('$K_3$','interpreter','latex')
legend(['$K_3 = $' num2str(data(end,29))],'interpreter','latex')

subplot(424)
grid on;
plot(data(:,35),data(:,30));
xlabel('Tempo[s]');
ylabel('$K_4$','interpreter','latex')
legend(['$K_4 = $' num2str(data(end,30))],'interpreter','latex')

subplot(425)
grid on;
plot(data(:,35),data(:,31));
xlabel('Tempo[s]');
ylabel('$K_5$','interpreter','latex')
legend(['$K_5 = $' num2str(data(end,31))],'interpreter','latex')

subplot(426)
hold on;
grid on;
plot(data(:,35),data(:,32));
xlabel('Tempo[s]');
ylabel('$K_6$','interpreter','latex')
legend(['$K_6 = $' num2str(data(end,32))],'interpreter','latex')

subplot(427)
hold on;
grid on;
plot(data(:,35),data(:,33));
xlabel('Tempo[s]');
ylabel('$K_7$','interpreter','latex')
legend(['$K_7 = $' num2str(data(end,33))],'interpreter','latex')

subplot(428)
hold on;
grid on;
plot(data(:,35),data(:,34));
xlabel('Tempo[s]');
ylabel('$K_8$','interpreter','latex')
legend(['$K_8 = $' num2str(data(end,34))],'interpreter','latex')
