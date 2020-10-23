%% Pioneer Position Control

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  %
% Initial Comands

clear; close all; clc;

try
    fclose(instrfindall);
catch
end
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %
% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes

% Robots
P = Pioneer3DX(1);

% Joystick
J = JoyControl;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network
Rede = NetDataShare;

%% Network communication check
tm = tic;
while true
    
    if isempty(Rede.pMSG.getFrom)
        Rede.mSendMsg(P);
        if toc(tm) > 0.1
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message......')
        end
    elseif length(Rede.pMSG.getFrom) > 1
        if isempty(Rede.pMSG.getFrom{2})
            Rede.mSendMsg(P);
            
            tm = tic;
            Rede.mReceiveMsg;
            disp('Waiting for message 1......')
            
        else
            break
        end
    end
end

%% Robots initial pose

% Xo = [0 0 0 0];
% P.rSetPose(Xo);    % define pose do robô
% P.pPos.X(1:2) = [0 0];

idP = getID(OPT,P);            % pioneer ID on optitrack
rb = OPT.RigidBody;            % read optitrack data
P = getOptData(rb(idP),P);    % get pioneer data


%% Initial Plot
figure(1);
axis([-2 2 -2 2]);
xlabel('Eixo x [m]','FontSize',12,'FontWeight','bold');
ylabel('Eixo y [m]','FontSize',12,'FontWeight','bold');
hold on;
grid on;

P.mCADdel;
P.mCADplot(1.7,'y');
p1 = plot(0,0,'or','LineWidth',2);
p2 = plot(0,0,'b--','LineWidth',2);
legend([p1,p2],'X_{d}','X','Location','northwest');

xinc = 0.5;
yinc = 0.2;
xp = P.pPos.X(1)+xinc;
yp = P.pPos.X(2)+yinc;
zp = 0;
t = tic;
tt_T = text(xp,yp+yinc*2,['t = ',num2str(toc(t),'%3.1f'),' [s]'],'FontWeight','bold');
tt_V = text(xp,yp+yinc,['v = ',num2str(P.pSC.U(1),'%3.1f'), ' [m/s]'],'FontWeight','bold');
tt_W = text(xp,yp,['w = ',num2str(180/pi*P.pSC.U(2),'%2.1f'), ' [°/s]'],'FontWeight','bold');
drawnow;

%% Variable initialization

data = [];

% Time variables initialization

T_CONTROL = 0.030;
T_PIONEER = 0.030;
T_PLOT = 0.100;

t_control = tic;
t_plot = tic;
t_Pioneer = tic;        % pioneer cycle

t  = tic;

disp('Start JoyControl ....');
while toc(t) < 0
    if toc(t_control) > T_CONTROL
        t_control = tic;
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);       % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
        end
        rb = OPT.RigidBody;             % read optitrack
        P = getOptData(rb(idP),P);
        
        %% Save data
        % Variable to feed plotResults function
        data = [  data  ; P.pPos.Xd'     P.pPos.X'        P.pSC.Ud(1:2)'    P.pSC.U(1:2)' ...
            toc(t)];
        % %       %   1 -- 12         13 -- 24          25 -- 26           27 -- 28
        % %       %   P1.pPos.Xd'     P1.pPos.X'        P1.pSC.Ud'         P1.pSC.U'
        % %
        % %       %   29
        % %       %   toc(t)  ];
        
        if toc(t_plot) > T_PLOT
            P.mCADdel;
            P.mCADplot(.7,'r');
            
            % Plotar rastros
            try
            catch
                delete (fig);
            end
            
            fig(1) = plot(data(end,1),data(end,2),'or','LineWidth',2);
            fig(2) = plot(data(:,13),data(:,14),'b--','LineWidth',2);
            
            legend([fig(1),fig(2)],'X_{d}','X','Location','northwest');
            
            xp = P.pPos.X(1)+xinc;
            yp = P.pPos.X(2)+yinc;
            zp = 0;
            
            tt_T.String = ['t = ',num2str(toc(t),'%3.1f'),' [s]'];
            tt_V.String = ['v = ',num2str(P.pSC.U(1),'%3.1f'), ' [m/s]'];
            tt_W.String = ['w = ',num2str(180/pi*P.pSC.U(2),'%2.1f'), ' [°/s]'];
            
            tt_T.Position = [xp ; yp+yinc*2 ; zp];
            tt_V.Position = [xp ; yp+yinc ; zp];
            tt_W.Position = [xp ; yp ; zp];
            
            drawnow;
        end
        
        %% Send Control Signal
        P = J.mControl(P);
%         Rede.mSendMsg(P);
    end
end
disp('JoyControl End....');

%% Simulation

fprintf('\nStart..............\n\n');

% Time variables initialization

T_CONTROL = 0.030;
T_PIONEER = 0.030;
T_PLOT = 0.200;

t_control = tic;
t_plot = tic;
t_Pioneer = tic;        % pioneer cycle

t  = tic;
flag=0;
while toc(t) < inf
    
    if toc(t_control) > T_CONTROL
        %% Get data from robot and Optitrack
        t_control = tic;
        Rede.mReceiveMsg;
        if length(Rede.pMSG.getFrom)>1
            P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);       % current velocities (robot sensors)
            PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
        end
        rb = OPT.RigidBody;             % read optitrack
        P = getOptData(rb(idP),P);
        %% Control
        
        %         cgains = [ 0.75  0.75  1.00  1.00  0.75  0.75  0.12  0.035 ];
        %         cgains = [ 0.40  0.20  0.75  0.75  0.75  0.4  0.2  0.03 ];
        %         P = fDynamicController(P);     % Pioneer Dynamic Compensator
        %         P.pPos.Xtil=P.pPos.Xd-P.pPos.X;
        %         k1=0.3; % Ganho do controlador
        %         K = [cos(P.pPos.X(6)) -P.pPar.a*sin(P.pPos.X(6));
        %             sin(P.pPos.X(6)) P.pPar.a*cos(P.pPos.X(6))];
        %         P.pSC.Ud=K\(k1.*[atan2(P.pPos.Xtil(1),1); atan2(P.pPos.Xtil(2),1)]);
        
        %         P = fControladorCinematico(P);
        
        %         if abs(P.pPos.Xtil(6))> 0.2
        %             P.pSC.Ud(1) = 0;
        %         else
        %             P.pSC.Ud(2) = 0;
        %         end
        
        % Controle do robô
        
        k1 = [0.50 0.00 ; 0.00 0.50]; % ganho de saturação
        k2 = [0.50 0.00 ; 0.00 0.50]; % ganho da inclinação da "reta"
        
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
        psi = atan2(P.pPos.Xtil(2),P.pPos.Xtil(1));
        angle = abs(P.pPos.X(6) - psi);
        
        K = [ cos(P.pPos.X(6))   -P.pPar.a*sin(P.pPos.X(6)); ...
            sin(P.pPos.X(6))    P.pPar.a*cos(P.pPos.X(6)) ];
        
        Xtil = P.pPos.Xtil(1:2);
        
        U = k1*tanh(k2*Xtil);
%         U = k1*Xtil./((k2*Xtil).^2+1); % sinal de controle
        
        P.pSC.Ud = diag([1,0.25])*(K\(P.pPos.Xd([7, 8]) + U)); % controlador
        
        % Orienta e depois posiciona
        if angle == pi
            P.pSC.Ud(1) = 0; % velocidade linear
            P.pSC.Ud(2) = .5; % velocidade angular
        end
        
        % se o angulo do robo e o ponto for maior que 10 aciona a flag
        if angle >= 10*pi/180
            flag = 1;
        end
        % se a flag for acionada e
        if flag == 1 && angle >= 5*pi/180
            P.pSC.Ud(1) = 0;
        elseif angle < 5*pi/180
            flag = 0;
        elseif flag == 0
            P.pSC.Ud(2) = 0;
        end
        
        
        
        
        
        %% Save data
        % Variable to feed plotResults function
        data = [  data  ; P.pPos.Xd'     P.pPos.X'        P.pSC.Ud(1:2)'    P.pSC.U(1:2)' ...
            toc(t)];
        % %       %   1 -- 12         13 -- 24          25 -- 26           27 -- 28
        % %       %   P1.pPos.Xd'     P1.pPos.X'        P1.pSC.Ud'         P1.pSC.U'
        % %
        % %       %   29
        % %       %   toc(t)  ];
        
        if toc(t_plot) > T_PLOT
            P.mCADdel;
            P.mCADplot(.7,'r');
            
            % Plotar rastros
            try
            catch
                delete (fig);
            end
            
            fig(1) = plot(data(end,1),data(end,2),'or','LineWidth',2);
            fig(2) = plot(data(:,13),data(:,14),'b--','LineWidth',2);
            
            legend([fig(1),fig(2)],'X_{d}','X','Location','northwest');
            
            xp = P.pPos.X(1)+xinc;
            yp = P.pPos.X(2)+yinc;
            zp = 0;
            
            tt_T.String = ['t = ',num2str(toc(t),'%3.1f'),' [s]'];
            tt_V.String = ['v = ',num2str(P.pSC.U(1),'%3.1f'), ' [m/s]'];
            tt_W.String = ['w = ',num2str(180/pi*P.pSC.U(2),'%2.1f'), ' [°/s]'];
            
            tt_T.Position = [xp ; yp+yinc*2 ; zp];
            tt_V.Position = [xp ; yp+yinc ; zp];
            tt_W.Position = [xp ; yp ; zp];
            
            drawnow;
        end
        
        %% Send control signals to robots
        %         P = J.mControl(P);
        
        if abs(norm(P.pPos.Xtil(1:2)))<0.1
            P.pSC.Ud = [0; 0];
            P.pPos.Xd(1:2)=ginput(1)';
        end
        Rede.mSendMsg(P);
        %         P.rSendControlSignals;
        
        %         if toc(t_control) > 0.030
        %             disp(['Estourou o tempo: ',num2str(1000*(toc(t_control)-0.030))]);
        %         end
        
    end
    
end

%% Send control signals
P.pSC.Ud = [-.5  ;  0];

for ii = 1:50
    Rede.mSendMsg(P);
end

%% Send control signals
P.pSC.Ud = [0  ;  0];
for ii = 1:50
    Rede.mSendMsg(P);
end

