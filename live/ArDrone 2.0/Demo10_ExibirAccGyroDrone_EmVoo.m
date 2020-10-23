% Exibir imagem ArDrone conectado

close all
clear
clc
try
    fclose(instrfindall);
end
addpath(genpath(pwd))

%% Inicializa a figura De acelerômetros 
tx = 20;
nn = 0:.05:tx;

hh = figure(2);
set(hh,'units','pix','pos',[5 100 1000 900],'PaperPositionMode','auto')

for k = 1:6
    ax(k) = subplot(6,1,k);
    if k < 4
        tracet(k) = plot(nn,zeros(1,size(nn,2)),'b--'); hold on
    end
    trace(k) = plot(nn,zeros(1,size(nn,2)),'r'); hold on
    xlim([0 nn(end)])
    grid on
end

Lax = {'Vel_x' 'Vel_y' 'Vel_z' 'Acc_x' 'Acc_y' 'Acc_z'};
legendas = {'Phi' 'Acc' 'Theta' 'Acc' 'Psi' 'Acc'};
% for idx = 1:3
% legend(ax(idx),legendas{2*idx-1:2*idx},'Location','southeast');
% end

for idx = 1:5
set(ax(idx),'YLim',[-1 1]);ylabel(ax(idx),Lax{idx})
end
set(ax(6),'YLim',[-2 0]);ylabel(ax(6),Lax{6})
%% Inicializa a Figura dos pwms
hi = figure(3);
set(hi,'units','pix','pos',[1200 700 500 300],'PaperPositionMode','auto')
for k = 1:4
    subplot(4,1,k);
    pw(k) = plot(nn,zeros(1,size(nn,2)),'r'); hold on
    xlim([0 nn(end)])
    ylim([0 255]);
    grid on
end

%%
% hg = figure(1);
% set(hg,'units','pix','pos',[800 100 1000 900],'PaperPositionMode','auto')
% axis([-1 1 -1 1 -0.5 0.5])
% grid on
% drawnow

disp('XBOX GAMEPAD COMMANDS:');
disp('Takeoff:  Y');
disp('Land:     A');
disp('Pitch:    Right Vertical Axis ');
disp('Roll:     Right Horizontal Axis');
disp('Altitude: Left Vertical Axis');
disp('Yaw:      LT/RT')
disp('-------------------------------');
disp('Start controlling......')
disp('Waiting take-off command')

disp('-------------------------------');
%%
A = ArDrone;
A.pFlag.Type = 'ArDrone';
A.rConnect;

% Conectar Joystick
A.cJoy;
Files = CreateDataFile(A);

%%
tmax = 20; % Tempo Simulação em segundos
PhysMes = zeros(6,size(nn,2));
Angles = zeros(3,size(nn,2));
motor = zeros(4,size(nn,2));
k = 1;
idc = 1;
% =========================================================================
t  = tic; % Tempo de simulação
tc = tic; % Tempo de controle
tp = tic; % Tempo de plotagem e exibição
%%
while toc(t) < tmax
    if toc(tc) > 1/30
        tc = tic;
        
        % Obter dados de voo
        A.rGetStatusRawData
        
        dat(:,idc) = A.pCom.cRawData';
        
        idc = idc+1;
        A.pSC.Ud = zeros(4,1);  
        % Controlador Joystick
        A.cJoyControl;
        A.rCommand;
        SaveData(Files,A);
        % Atribuir variáveis
        A.pPos.X(4) = A.pCom.cRawData(2)*pi/180;
        A.pPos.X(5) = A.pCom.cRawData(3)*pi/180;
        A.pPos.X(6) = A.pCom.cRawData(4)*pi/180;
    end
    if toc(tp) > 0.05
        for ii = 1:4
            motor(ii,k) = A.pCom.cRawData(ii+14);
            pw(ii).YData = motor(ii,end-size(nn,2)+1:end);
        end
        
        % Capture Velocities
        for ii = 1:3
            PhysMes(ii,k) = A.pCom.cRawData(ii+5);
            trace(ii).YData = PhysMes(ii,end-size(nn,2)+1:end);
        end
        % Capture Accelerations
        for ii = 4:6
            PhysMes(ii,k) = A.pCom.cRawData(ii+5);
            trace(ii).YData = PhysMes(ii,end-size(nn,2)+1:end);
        end
        % Capture Angles
        for ii = 1:2
            Angles(ii,k) = A.pPos.X(ii+3);
            tracet(ii).YData = Angles(ii,end-size(nn,2)+1:end);
        end
        k = k+1;
%         A.mCADplot;
        drawnow
    end
end
%%

A.rSetLed(2,10,5);
A.rLand;
A.rDisconnect
CloseDataFile(Files)
