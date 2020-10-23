%% Testar modelo dinâmico do ArDrone em uma trajetória

% Inicialização
close all
clear
clc

try
    fclose(instrfindall);
end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Joystick Control

% Joystick
J = JoyControl;


%% Robot initialization
A = ArDrone;
% gains = [1.5 3.2 1.5 3.2 1.5 2;
%          10 5 10 5 1 5];
% gains = [1 5 1 5 1.5 2;
%          10 5 10 5 1 5];

gains = [0.5 3 0.6 3 2 15;
    10 3 8 3 1 5];

A.pPar.uSat(1:2) = A.pPar.uSat(1:2)*2.0;
A.pPar.uSat(3) = A.pPar.uSat(3)*1.0;
A.pPar.uSat(4) = A.pPar.uSat(4)*1.0;


% A.rConnect;
% A.rTakeOff;
% A.rGetSensorCalibration;
pause(3)

%% Window definition
figure(1)
axis([-3 3 -3 3 0 3])
grid on
A.mCADplot
drawnow
% pause

%% Variables Initialization
a = 1;        % "raio" do circulo em x
b = 1;          % "raio" do circulo em y
% w = 0.05;     % lemniscata
w = 0.2;     % circulo

XX = [];            % position data
% X = zeros(1,19);  % Dados correntes da simulação

% Time variables
tmax = 60;     % Simulation duration
tc = tic;      % drone frequency
tp = tic;      % graph refresh rate
tt = tic;      % trajectory time
t = tic;       % simulation current time

%% Simulation loop
while toc(t) < tmax
    
    if toc(tc) > 1/30
        tc = tic;
        
        % Trajetória desejada
        %         % Lemniscata
        %         tt = toc(t);
        %         A.pPos.Xd(1) = a*sin(2*pi*w*tt);            % x
        %         A.pPos.Xd(7) = a*2*pi*w*cos(2*pi*w*tt);     % dx
        %         A.pPos.Xd(2) = b*sin(4*pi*w*tt);          % y
        %         A.pPos.Xd(8) = b*4*pi*w*cos(4*pi*w*tt); % dy
        %         A.pPos.Xd(3) = 1;                         % z
        %
        if toc(t)>50
            % Return to initial position
            A.pPos.Xd(1)  = 0;    % posição x
            A.pPos.Xd(2)  = 0;    % posição y
            A.pPos.Xd(7)  = 0;  % velocidade em x
            A.pPos.Xd(8)  = 0;   % velocidade em y
            A.pPos.Xd(3)  = 1.5;
        else
            %         % Circle
                    tt = toc(t);
                    A.pPos.Xd(1)  = a*cos(w*tt);    % posição x
                    A.pPos.Xd(2)  = b*sin(w*tt);    % posição y
                    A.pPos.Xd(7)  = -a*w*sin(w*tt);  % velocidade em x
                    A.pPos.Xd(8)  = b*w*cos(w*tt);   % velocidade em y
                    A.pPos.Xd(3)  = 1.5;               % z
            
            %           A.pPos.Xd(6)  =  w*tt;
            %         A.pPos.Xd(12) = w;
            
            %         A.pPos.Xd(6)  =  atan2(A.pPos.Xd(8),A.pPos.Xd(7));
            %         A.pPos.Xd(12) = 1/(1+(A.pPos.Xd(8)/A.pPos.Xd(7))^2)* ...
            %            ((A.pPos.Xd(8)*A.pPos.Xd(1)-A.pPos.Xd(2)*A.pPos.Xd(7))/A.pPos.Xd(7)^2);
            %
            % lemniscata
%             tt = toc(t);
%             A.pPos.Xd(1) = a*sin(2*pi*w*tt);            % x
%             A.pPos.Xd(7) = a*2*pi*w*cos(2*pi*w*tt);     % dx
%             A.pPos.Xd(2) = b*sin(4*pi*w*tt);          % y
%             A.pPos.Xd(8) = b*4*pi*w*cos(4*pi*w*tt); % dy
%             A.pPos.Xd(3) = 2;
            
            
        end
        % Controlador
        A.rGetSensorData
        
        A = cUnderActuatedControllerMexido(A,gains);
        A = J.mControl(A);           % joystick command (priority)
        A.rSendControlSignals;
        
        XX = [XX; [A.pPos.Xd'; A.pPos.X'; toc(t)]];
        
    end
    
    % Draw robot
    if toc(tp) > 0.3
        tp = tic;
        A.mCADplot;
        plot3(XX(:,1),XX(:,2),XX(:,3),'r-')
        drawnow
    end
    
end

% Land drone
if A.pFlag.Connected == 1
    A.rLand;
    %     A.rDisconnect;
end

%% Plot results
% Roll and pitch angles
% figure
% subplot(311),plot(XX(end,:),XX([4 16],:)'*180/pi)
% legend('\phi_{Des}','\phi_{Atu}')
% grid
% subplot(312),plot(XX(end,:),XX([5 17],:)'*180/pi)
% legend('\theta_{Des}','\theta_{Atu}')
% grid
% subplot(313),plot(XX(end,:),XX([6 18],:)'*180/pi)
% legend('\psi_{Des}','\psi_{Atu}')
% grid

% Trajectory
figure
plot(XX(:,[1 13]),XX(:,[2 14]))
% axis([-1.5 1.5 -1.5 1.5])
axis equal

% % X and Y
% figure
% subplot(211),plot(XX(end,:),XX([1 13],:)')
% legend('x_{Des}','x_{Atu}')
% grid
% subplot(212),plot(XX(end,:),XX([2 14],:)')
% legend('y_{Des}','y_{Atu}')
% grid
% 
% % dX and dY
% figure
% subplot(211),plot(XX(end,:),XX([7 19],:)')
% legend('dx_{Des}','dx_{Atu}')
% grid
% subplot(212),plot(XX(end,:),XX([8 20],:)')
% legend('dy_{Des}','dy_{Atu}')
% grid
% 

