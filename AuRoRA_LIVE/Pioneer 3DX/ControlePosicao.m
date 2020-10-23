clear; close all; clc;
% Close all the open connections
try
    fclose(instrfindall);
catch
end

%% Look for root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Classes initialization
% Robot
P = Pioneer3DX;

%% Conexão com robô/simulador
P.rConnect;             % robô ou mobilesim
pause(3);
clc;
fprintf('\nInício..............\n\n')

%% Posição inicial do robô
Xo = [0 0 0 0];
P.rSetPose(Xo);    % define pose do robô
P.pPos.X(1:2) = [0 0];

Hist = [];
IAE = 0;
ITAE = 0;
IASC = 0;

figure(1);
axis([-4 4 -4 4]);
xlabel('Eixo x [m]','FontSize',12,'FontWeight','bold');
ylabel('Eixo y [m]','FontSize',12,'FontWeight','bold');
hold on;
grid on;

P.mCADdel;
P.mCADplot(.7,'r');
p1 = plot(0,0,'r--','LineWidth',2);
p2 = plot(0,0,'b','LineWidth',2);
legend([p1,p2],'X_{d}','X','Location','northwest');

drawnow;

%% Laço de Repetição
pause(2);
tmax = 45; % tempo de simulação
t = tic; % timer geral
ta = tic; % timer para controle
tplot = tic;
pos = 0;
traj = 1;

if pos == 1 && traj == 0
    tmax = 45;
    tparcial = tmax/3;
else
end

while toc(t) < tmax+0.2
    
    if toc(ta) > .1
        ta = tic;
        
        % Definição da trajetória
%         rA = 1.5; % [m]
%         rB = 1; % [m]
%         T = 30; % [s]

        rA = 2; % [m]
        rB = 1.5; % [m]
        T = 60; % [s]
        
        w = 2*pi/T; % [rad/s]
        
        if (toc(t) < tmax) && ((pos == 0) && (traj == 1))
            P.pPos.Xd(1) = rA*sin(w*toc(t));
            P.pPos.Xd(2) = rB*sin(2*w*toc(t));
            P.pPos.Xd(7) = rA*w*cos(w*toc(t));
            P.pPos.Xd(8) = 2*rB*w*cos(2*w*toc(t));
            
            k1 = [0.75 0.00 ; 0.00 0.75]; % ganho de saturação
            k2 = [0.50 0.00 ; 0.00 0.50]; % ganho da inclinação da "reta"
            
        elseif ((pos == 1) && (traj == 0))
            if toc(t) < tparcial
                P.pPos.Xd(1) = 2;
                P.pPos.Xd(2) = 2;
            elseif (tparcial < toc(t)) && (toc(t) < 2*tparcial)
                P.pPos.Xd(1) = 2;
                P.pPos.Xd(2) = 0;
            else
                P.pPos.Xd(1) = 0;
                P.pPos.Xd(2) = 2;
            end
            
            k1 = [0.50 0.00 ; 0.00 0.50]; % ganho de saturação
            k2 = [0.50 0.00 ; 0.00 0.50]; % ganho da inclinação da "reta"
        end
        
        % Controle do robô
        P.rGetSensorData;
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
        K = [ cos(P.pPos.X(6))   -P.pPar.a*sin(P.pPos.X(6)); ...
            sin(P.pPos.X(6))    P.pPar.a*cos(P.pPos.X(6)) ];
        
        Xtil = P.pPos.Xtil(1:2);
        
%         U = k1*Xtil./((k2*Xtil).^2+1); % sinal de controle
        
        % Tanh
        k1 = [0.75 0.00 ; 0.00 0.75]; % ganho de saturação
        k2 = [0.250 0.00 ; 0.00 0.250]; % ganho da inclinação da "reta"
        U = k1*tanh(k2*P.pPos.Xtil(1:2)); % sinal de controle
        
        if toc(t) >= tmax
            P.pSC.Ud = [0 ; 0];
        else
            P.pSC.Ud = K\(P.pPos.Xd([7, 8]) + U);
        end
        
        P.rSendControlSignals;
        
        Hist = [Hist; [P.pPos.Xd' P.pPos.X' P.pPos.Xtil' P.pSC.U' P.pSC.Ud' P.pSC.Ur' toc(t)]];
        
        IAE = IAE + .1*norm(P.pPos.Xtil([1 2]));
        ITAE = ITAE + toc(t)*.1*norm(P.pPos.Xtil([1 2]));
        IASC = IASC + .1*norm(P.pSC.U);
        
        % Plotar o robô
        if toc(tplot) > .1
            tplot = tic;
            P.mCADdel;
            P.mCADplot(.7,'r');
            
            % Plotar rastros
            try
                delete (fig);
            end
            
            if pos == 0 && traj == 1
                fig(1) = plot(Hist(:,1),Hist(:,2),'r--','LineWidth',2);
                fig(2) = plot(Hist(:,13),Hist(:,14),'b','LineWidth',1);
                legend([p1,p2],'X_{d}','X','Location','northwest');
                
            else
                fig(1) = plot(Hist(:,1),Hist(:,2),'or','LineWidth',2);
                fig(2) = plot(Hist(:,13),Hist(:,14),'b','LineWidth',1);
                legend([p1,p2],'X_{d}','X','Location','northwest');
                
            end
                       
            drawnow;
        end
    end
    
end
%% Plotagem

% % Hist = [Hist; [P.pPos.Xd' P.pPos.X' P.pPos.Xtil' P.pSC.U' P.pSC.Ud' P.pSC.Ur' toc(t)]];
%
% if pos == 0 && traj == 1
    % Eixo x [ X(1) ]
%     figure(2);
%     plot(Hist(:,43),Hist(:,1),'--r','LineWidth',2);
%     hold on;
%     plot(Hist(:,43),Hist(:,13),'k','LineWidth',2);
%     grid on;
%     legend('x_{d}','x');
%     axis([0 tmax -2 2]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('Eixo x [m]','FontSize',12,'FontWeight','bold');
%
%     % Eixo y [ X(2) ]
%     figure(3);
%     plot(Hist(:,43),Hist(:,2),'--r','LineWidth',2);
%     hold on;
%     plot(Hist(:,43),Hist(:,14),'k','LineWidth',2);
%     grid on;
%     legend('y_{d}','y');
%     axis([0 tmax -2 2]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('Eixo y [m]','FontSize',12,'FontWeight','bold');
%
%     % Psi [ X(6) ]
%     figure(4);
%     plot(Hist(:,43),180/pi*Hist(:,18),'k','LineWidth',2);
%     hold on;
%     grid on;
%     legend('\psi');
%     axis([0 tmax -200 200]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('\psi [ ° ]','FontSize',12,'FontWeight','bold');
%
%     % Erro Linear [ Xtil(1) , Xtil(2) ]
%     figure(5);
%     plot(Hist(:,43),Hist(:,25),'r','LineWidth',2);
%     hold on;
%     plot(Hist(:,43),Hist(:,26),'k','LineWidth',2);
%     grid on;
%     legend('x_{erro}','y_{erro}');
%     axis([0 tmax -0.1 0.5]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('Erro Linear [m]','FontSize',12,'FontWeight','bold');
%
%     % Velocidade Linear [ Ud(1) , U(1) ]
%     figure(6);
%     plot(Hist(:,43),Hist(:,39),'--r','LineWidth',2);
%     hold on;
%     plot(Hist(:,43),Hist(:,37),'k','LineWidth',2);
%     plot(Hist(:,43),.75*ones(length(Hist)),'--b','LineWidth',2);
%     grid on;
%     legend('U_{d}','U','U_{sat}');
%     axis([0 tmax 0 1]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('Velocidade Linear [m/s]','FontSize',12,'FontWeight','bold');
%
%     % Velocidade angular [ Ud(2) , U(2) ]
%     figure(7);
%     plot(Hist(:,43),180*Hist(:,40)/pi,'--r','LineWidth',2);
%     hold on;
%     plot(Hist(:,43),180*Hist(:,38)/pi,'k','LineWidth',2);
%     plot(Hist(:,43),[100*ones(length(Hist)), -100*ones(length(Hist))],'--b','LineWidth',2);
%     grid on;
%     legend('U_{d}','U','U_{sat}');
%     axis([0 tmax -250 250]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('Velocidade Angular [°/s]','FontSize',12,'FontWeight','bold');
%
%     fprintf('.................Fim\n\n')
%
% else
%     % Eixo x [ X(1) ]
%     figure(2);
%     plot(Hist(:,43),Hist(:,1),'--r','LineWidth',2);
%     hold on;
%     plot(Hist(:,43),Hist(:,13),'k','LineWidth',2);
%     grid on;
%     legend('x_{d}','x');
%     axis([0 tmax -.5 3]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('Eixo x [m]','FontSize',12,'FontWeight','bold');
%
%     % Eixo y [ X(2) ]
%     figure(3);
%     plot(Hist(:,43),Hist(:,2),'--r','LineWidth',2);
%     hold on;
%     plot(Hist(:,43),Hist(:,14),'k','LineWidth',2);
%     grid on;
%     legend('y_{d}','y');
%     axis([0 tmax -.5 3]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('Eixo y [m]','FontSize',12,'FontWeight','bold');
%
%     % Psi [ X(6) ]
%     figure(4);
%     plot(Hist(:,43),180/pi*Hist(:,18),'k','LineWidth',2);
%     hold on;
%     grid on;
%     legend('\psi');
%     axis([0 tmax -200 200]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('\psi [ ° ]','FontSize',12,'FontWeight','bold');
%
%     % Erro Linear [ Xtil(1) , Xtil(2) ]
%     figure(5);
%     plot(Hist(:,43),Hist(:,25),'r','LineWidth',2);
%     hold on;
%     plot(Hist(:,43),Hist(:,26),'k','LineWidth',2);
%     grid on;
%     legend('x_{erro}','y_{erro}');
%     axis([0 tmax -3 3]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('Erro Linear [m]','FontSize',12,'FontWeight','bold');
%
%     % Velocidade Linear [ Ud(1) , U(1) ]
%     figure(6);
%     plot(Hist(:,43),Hist(:,39),'--r','LineWidth',2);
%     hold on;
%     plot(Hist(:,43),Hist(:,37),'k','LineWidth',2);
%     plot(Hist(:,43),.75*ones(length(Hist)),'--b','LineWidth',2);
%     plot(Hist(:,43),-.75*ones(length(Hist)),'--b','LineWidth',2);
%     grid on;
%     legend('U_{d}','U','U_{sat}');
%     axis([0 tmax -1 1]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('Velocidade Linear [m/s]','FontSize',12,'FontWeight','bold');
%
%     % Velocidade angular [ Ud(2) , U(2) ]
%     figure(7);
%     plot(Hist(:,43),180*Hist(:,40)/pi,'--r','LineWidth',2);
%     hold on;
%     plot(Hist(:,43),180*Hist(:,38)/pi,'k','LineWidth',2);
%     plot(Hist(:,43),[100*ones(length(Hist)), -100*ones(length(Hist))],'--b','LineWidth',2);
%     grid on;
%     legend('U_{d}','U','U_{sat}');
%     axis([0 tmax -250 250]);
%     xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
%     ylabel('Velocidade Angular [°/s]','FontSize',12,'FontWeight','bold');
%
%     fprintf('.................Fim\n\n')
% end

% %%
%
% % s = tf('s');
% % H(1) = (280*exp(-120*s)) / (1200*s + 1);
% % H(2) = (280) / (1200*s + 1);
% % step(H(1),H(2));
% % grid on;
% % legend('com','sem');
%
% figure(8);
% hold on;
% grid on;
% x = linspace(-10,10);
% fx{1} = 0.25*x./((.5*x).^2+1);
% fx{2} = 0.50*x./((.5*x).^2+1);
% fx{3} = 0.75*x./((.5*x).^2+1);
% fx{4} = 1.00*x./((.5*x).^2+1);
% plot(x,fx{1},'LineWidth',2);
% plot(x,fx{2},'LineWidth',2);
% plot(x,fx{3},'LineWidth',2);
% plot(x,fx{4},'LineWidth',2);
% legend('0.25','0.50','0.75','1.00','Location','northwest');
% title('f(x) = k_{1} * x  / [ ( k_{2} * x )^2 + 1 ]    ,    k_{2} = 0.50');
% xlabel('x','FontSize',12,'FontWeight','bold');
% ylabel('f(x)','FontSize',12,'FontWeight','bold');
% axis([-10 10 -1.5 1.5]);
%
% %%
%
% figure(9);
% hold on;
% grid on;
% x = linspace(-10,10);
% fx{1} = .5*x./((.25*x).^2+1);
% fx{2} = .5*x./((.5*x).^2+1);
% fx{3} = .5*x./((.75*x).^2+1);
% fx{4} = .5*x./((1*x).^2+1);
% plot(x,fx{1},'LineWidth',2);
% plot(x,fx{2},'LineWidth',2);
% plot(x,fx{3},'LineWidth',2);
% plot(x,fx{4},'LineWidth',2);
% legend('0.25','0.50','0.75','1.00','Location','northwest');
% title('f(x) = k_{1} * x  / [ ( k_{2} * x )^2 + 1 ]    ,    k_{1} = 0.50');
% xlabel('x','FontSize',12,'FontWeight','bold');
% ylabel('f(x)','FontSize',12,'FontWeight','bold');
% axis([-10 10 -1.5 1.5]);


