%% Pioneer + Carreta de cargas vs Esmeril 
% Alexandre Caldeira e Thiago Ridogério (11/02/20)
% Desenhar carreta:
figure
grid on
axis([-3,3,-3,3])
draw_rectangle([1,0],truck.pPar.L,truck.pPar.H,pi/2,[1,0,0])

%%
% Boas práticas
clearvars
close all
clc

% Carregando a classe do Pioneer:
P = Pioneer3DX;

% Definindo os parametros do Pioneer:
P.pPar.a = 0;
P.pPar.alpha = 0;
pgains = 10*[.2 .2 1];

% Definindo os parâmetros da carreta:
truck.pPar.F = .5;
truck.pPar.L = .45;
truck.pPar.H = .35;
truck.pPar.a = -.25;
truck.pPar.alpha = 0;
truck.pPar.Ts = P.pPar.Ts;
truck.pPos.X = zeros(12,1);
truck.pSC.U = [0;0];
D = 0;
% phi = 2*atan2(,)

% Trajetoria:
V_MAX = 0.25;
rx = 1;
ry = rx;
T_MAX = 2*pi*rx/V_MAX;
W = 2*pi/T_MAX;
Xd = [rx*sin(W*0); ry*sin(W*0); 0]; dXd = [W*rx*cos(W*0);W*ry*cos(W*0);0];
P.pPos.Xd([1:3 6]) = [Xd; atan2(dXd(2),dXd(1))];


% Definir posição inicial do robô e carreta:
P.rSetPose(P.pPos.Xd([1:3 6]));

truck.pPos.X([1 2 6]) = [P.pPos.X(1)-cos(P.pPos.X(6))*truck.pPar.F;...
                         P.pPos.X(2)-sin(P.pPos.X(6))*truck.pPar.F; P.pPos.X(6)];

% Plot
figure
% subplot(2,2,[1 3])
P.mCADplot(1,'k')
grid on
hold on
[trk2,trk] = draw_rectangle([truck.pPos.X(1),truck.pPos.X(2)],truck.pPar.L,...
                                     truck.pPar.H,truck.pPos.X(6),[1,0,0]);
pont = plot(truck.pPos.X(1),truck.pPos.X(2),'r*'); 
% [trk2,trk] = draw_rectangle([1,-0.5],truck.pPar.L,...
%                             truck.pPar.H,P.pPos.X(6),[1,0,0]);
axis equal
axis([-1.5 1.5 -1.5 1.5])

% Armazenador de dados
DADOS = [];
TADOS = [];

% Temporizadores
T_AMOSTRAGEM = 1/30;
T_PLOT = 1/30;

T = tic;
Ta = tic;
Tp = tic;
T_dXd = tic;
flag =0;
%%
% Laço de simulação
while toc(T) < T_MAX
    % Laço de controle
    if toc(Ta) > T_AMOSTRAGEM
        Ta = tic;
        
        XA = P.pPos.X;
        % Pegar sinais de posição do pioneer
        P.rGetSensorData;
        
        % Trajetoria
        XdA = Xd;
        Xd = [rx*sin(W*toc(T));
              ry*sin(2*W*toc(T));
              0];
          
%         Xd = [rx; 1.885*toc(T)/T_MAX; 0];
          
        dXd = (Xd-XdA)/toc(T_dXd);
        T_dXd = tic;
        
        P.pPos.Xd(1:3) = Xd;
        P.pPos.Xd(7:9) = dXd;
        
        % Controle
        P = fKinematicControllerExtended(P,pgains);
        
        truck.pSC.X([7 8 12]) = P.pPos.X([7 8 12]);
        truck.pSC.U = [norm(P.pPos.X(7:8));P.pPos.X(12)];
                
        if P.pPos.X(12) == 0
            D = 0;
            else
            D = norm(P.pPos.X(7:8))/P.pPos.X(12);
        end
        
        truck.pPar.alpha = 2*atan2(truck.pPar.F,D);
        if abs(truck.pPar.alpha) > pi
            if truck.pPar.alpha < 0
                truck.pPar.alpha = truck.pPar.alpha + 2*pi;
            else
                truck.pPar.alpha = truck.pPar.alpha - 2*pi;
            end
        end
        
        truck.pPos.X(6) = P.pPos.X(6) - truck.pPar.alpha;
        
        if abs(truck.pPos.X(6)) > pi
            if truck.pPos.X(6) < 0
                truck.pPos.X(6) = truck.pPos.X(6) + 2*pi;
            else
                truck.pPos.X(6) = truck.pPos.X(6) - 2*pi;
            end
        end

        truck.pPos.X([1 2]) = [P.pPos.X(1)-cos(P.pPos.X(6))*truck.pPar.F;...
                               P.pPos.X(2)-sin(P.pPos.X(6))*truck.pPar.F];
                           
        truck = truckKinematicModel(truck);
        
        % Armazenando dados
        DADOS(end+1,:) = [P.pPos.Xd' P.pPos.X' P.pSC.Ud' toc(T)];
        
        TADOS(end+1,:) = [truck.pPos.X' P.pSC.U' toc(T)];     
        %                 1--12         13--24          25--26          27
        %                 P.pPos.Xd'    P.pPos.X'       P.pSC.Ud        toc(T)
        
        % Enviar sinais de controle
        P.rSendControlSignals;
    end
    % Laço de plot
    if toc(Tp) > T_PLOT
        try
            P.mCADdel;
            delete(trk2)
            delete(trk)
            delete(pont)
        catch
        end
       
        P.mCADplot(1,'k');
        Traj = plot([XdA(1) Xd(1)],[XdA(2) Xd(2)],'--k','LineWidth',1.6);
        Rastro = plot([XA(1) P.pPos.X(1)],[XA(2) P.pPos.X(2)],'r','LineWidth',1.6);
        
        pont = plot(truck.pPos.Xc(1),truck.pPos.Xc(2),'k*','MarkerSize',10); 
        
        [trk2,trk] = draw_rectangle([truck.pPos.X(1),truck.pPos.X(2)],truck.pPar.L,...
                                     truck.pPar.H,truck.pPos.X(6),[1,0,0]);
            
        title('Simulação')
        xlabel('X [m]')
        ylabel('Y [m]')
        drawnow
    end
end

%% Gráficos
% figure(2)
% subplot(121)
% plot(DADOS(:,end),DADOS(:,25),'b','LineWidth',1.2)
% hold on
% plot(DADOS(:,end),DADOS(:,26),'r','LineWidth',1.2)
% title('Sinal de controle no tempo')
% xlabel('Tempo [s]')
% legend('U_d','W_d')
% grid on
% 
% % figure(3)
% % plot(DADOS(:,end),(DADOS(:,1)-DADOS(:,13))*.2,'b')
% % hold on
% % grid on
% % plot(DADOS(:,end),(DADOS(:,2)-DADOS(:,14))*.2,'r')
% 
% % figure(2)
% subplot(122)
% plot(DADOS(:,end),DADOS(:,7),'b','LineWidth',1.2)
% hold on
% plot(DADOS(:,end),DADOS(:,8),'r','LineWidth',1.2)
% plot(DADOS(:,end),sqrt(DADOS(:,7).^2+DADOS(:,8).^2),'k','LineWidth',1.2)
% title('Velocidades no tempo')
% xlabel('Tempo [s]')
% legend('dX_d','dY_d','U_d')
% ylim([-1.2*max(sqrt(DADOS(:,7).^2+DADOS(:,8).^2)) ...
%       +1.2*max(sqrt(DADOS(:,7).^2+DADOS(:,8).^2))])
% grid on


