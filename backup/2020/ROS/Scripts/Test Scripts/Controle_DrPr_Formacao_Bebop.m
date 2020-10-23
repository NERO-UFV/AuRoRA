%% Tarefa 1

% Limpa as variávies
clear all; 
close all;
warning off;
clc;

% Carrega o diretório corrente e subdiretórios 
%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


%%%%%%%%%%%%%%%%%%%%%%%%%%%% Drone  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
try
    fclose(instrfindall);
end



he=figure(10);
%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);


set(he,'units','pix','pos',[50 50 500 500],'PaperPositionMode','auto')

         



%%
hg = figure(1); 
set(hg,'units','pix','pos',[800 100 1000 900],'PaperPositionMode','auto')
axis([-1 1 -1 1 -0.5 0.5])
grid on

drawnow


%% Load Class
try     
    % Load Classes
    RI = RosInterface;         
    RI.rConnect;    
    B = Bebop(1);        
    
    disp('### Load Class Success');
    
catch ME
    disp(' #### Load Class Issues ');
    disp('');
    disp(ME);
    RI.rDisconnect;
    rosshutdown;
    return;
end


%% Inicializa a figura dos acelerômetros 
tx = 30;
nn = 0:.05:tx;

hh = figure(2);
set(hh,'units','pix','pos',[5 100 1000 900],'PaperPositionMode','auto')

spNum = 7;
for k = 1:spNum
    ax(k) = subplot(spNum,1,k);
    if k < 8
        tracet(k) = plot(nn,zeros(1,size(nn,2)),'b--'); hold on
    end
    trace(k) = plot(nn,zeros(1,size(nn,2)),'r'); hold on
    xlim([0 nn(end)])
    grid on
end
Lax = {'Alt_z' 'Vel_y' 'Vel_y' 'Vel_z' 'Ang_Y' 'Ang_X' 'Ang_z'};


set(ax(1),'YLim',[-1 4]);ylabel(ax(1),Lax{1})
set(ax(2),'YLim',[-1 1]);ylabel(ax(2),Lax{2})
set(ax(3),'YLim',[-1 1]);ylabel(ax(3),Lax{3})
set(ax(4),'YLim',[-20 20]);ylabel(ax(4),Lax{4})
set(ax(5),'YLim',[-1 1]);ylabel(ax(5),Lax{5})
set(ax(6),'YLim',[-1 1]);ylabel(ax(6),Lax{6})
set(ax(7),'YLim',[-5 5]);ylabel(ax(7),Lax{7})

hold off;

           

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Pioneer  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define o tempo de amostragem (100ms)
%sampleTime = 0.1;

% Define o tempo
%t = 0:sampleTime:25; 

% Inicializa as variáveis 
a = .2;
ku=0.4; %Dinamica
lu=0.2; %Dinamica
kw=0.4; %Dinamica
lw=0.2; %Dinamica
theta=[0.2604 0.2509 -0.000499 0.9965 0.00263 1.07680];
H=[theta(1) 0;0 theta(2)];

pos(3,1) = 0;
vD(2,1) = 0;
vRef(2,1) = 0;
vRobot(2,1) = 0;
vErro(2,1) = 0;
aux = 1;

u_robot=[0 0];
u_ref=[0 0];
velantX=0;velantY=0;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Drone %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Condições Iniciais de Posição do Drone
x(1) = 0;
y(1) = 0;
z(1) = 1;
phi(1) = 0;
phid = 0;

% Inicializa Parametros de configuração do teste
tmax = 60; % Tempo Simulação em segundos
PhysMes = zeros(6,size(nn,2));
Angles = zeros(3,size(nn,2));
k = 1;

% Variavel do botao de emergencia
btnEmergencia = 0;
countWhile = 0;

% Ganho do Controledor do Drone Kp na saída de comando
% Reduzir este ganho para teste de novos parametros
ganho = 0.01;

% Tempo de Amostragem
t  = tic; % Tempo de simulação
tc = tic; % Tempo de controle
tp = tic; % Tempo de plotagem e exibição
sampleTime = 1/5;
drawTime = inf;

nLandMsg = 5;    % Numero de mensagens enviadas para pouso do drone
dLandMsg = 0.5 ; % Atraso de tempo entre messagens de pouso


%% Decolagem do Drone

% Assegura a decolagem vertical 
% Os valores de comando na decolagem sejam iguais a zero
B.pSC.Ud(1) = 0; % Esquerda/Direita [-1,1] (-) Move Drone para Esquerda
B.pSC.Ud(2) = 0; % Frente/Tras [-1,1] (-) Avanï¿½a - Move frente para baixo
B.pSC.Ud(3) = 0; % Velocidade Vertical [-1,1] (+) Eleva o drone
B.pSC.Ud(4) = 0; %   

%Inicio impressão do pioneer
[x,y,psi,~,~,~,~] = leerRobot();
scrsz=get(0,'ScreenSize');
off1=0;
off2=10;
figpos=[off1 off2 scrsz(3)-off1 scrsz(4)-off2];

f1=figure(1);
set(f1,'Color','w','Position',figpos);
paso=2; axis 'equal'
Robot_Dimension(2);
Ho=Robot_Plot_3D(x,y,psi,'g'); hold on
H1 = plot(x,y,'*m'); hold on;
H2 = plot(x,y,'--r','LineWidth',2); hold on;

%plot inicial drone
H1d = plot3(x(1),y(1),z(1),'yo','LineWidth',2,'MarkerEdgeColor','k',...
        'MarkerFaceColor','y','MarkerSize',10);
H2d = plot3(x(1),y(1),z(1),'-g','LineWidth',2); hold on;

%Objetivo da formacao
qDes = [1 0 0 1 0 pi/3]';


% Envia Comando de Decolagem para o Drone e aguarda estabilização de
B.rTakeOff;
pause(4);

disp(" ");
disp("Final da Estabilizaçao .... Controlador ON...")
disp(" ");


%%
while toc(t) < tmax
    try  
 
        % Verifica Condicao de emergencia a cada loop
        if btnEmergencia ~= 0 
            disp('EMERGENCIA Matlab Acionado...');
            escribirRobot(0,0);
            B.rLand; 
            break;
        end
        
        
        %Control Loop
        if toc(tc) > sampleTime
            tc = tic;
            countWhile = countWhile + 1;             
        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                   Atualização de Dados           
            
            % Leitura dos dados do pioneer                        
            [pos(1,k),pos(2,k),pos(3,k),vRobot(1,k),vRobot(2,k),~,~] = leerRobot();
            
            % Leitura dos dados do bebop                        
            B.rGetSensorData;  
            x(k) = B.pPos.X(1);
            y(k) = B.pPos.X(2);
            z(k) = B.pPos.X(3);
            phi(k) = 0;
            
            
            % Capture Altitude
            PhysMes(1,k) = B.pPos.X(3);
            tracet(1).YData = PhysMes(1,end-size(nn,2)+1:end);
            Nav(k,1) = PhysMes(1,k);

            % Capture Velocidades          
            for ii = 1:3               
                PhysMes(ii+1,k) = B.pPos.X(ii+6);
                tracet(ii+1).YData = PhysMes(ii,end-size(nn,2)+1:end);
                Nav(k,ii+1) = PhysMes(ii,k);
            end 
           
            % Capture Angles            
            for ii = 1:3
                Angles(ii,k) = B.pPos.X(ii+3);
                tracet(ii+4).YData = Angles(ii,end-size(nn,2)+1:end);
                Nav(k,ii+4) = Angles(ii,k);
            end 

            
            %controlador da formacao
            xf = pos(1,k);
            yf = pos(2,k);
            zf = 0;
            rhof = sqrt((x(k) - pos(1,k))^2 + (y(k) - pos(2,k))^2 + (z(k)- 0)^2);            
            betaf = atan2((y(k) - pos(2,k)),(x(k) - pos(1,k)));
            alphaf = atan2((z(k) - 0),sqrt((x(k) - pos(1,k))^2 + (y(k) - pos(2,k))^2));
            q = [xf yf zf rhof betaf alphaf];
            qTil(:,k) = qDes - q';

            %Ganhos
            L  = 0.5*diag([1 1 1 1 1 1]);
            kp = 0.4*diag([1 1 1 1 1 1]);

            
            qRefPonto = L*tanh(inv(L)*kp*qTil(:,k));

            jacob = [ 1, 0, 0, 0, 0, 0; ...
                      0, 1, 0, 0, 0, 0; ...
                      0, 0, 1, 0, 0, 0; ...
                      1, 0, 0, cos(alphaf)*cos(betaf), -rhof*cos(alphaf)*sin(betaf), -rhof*cos(betaf)*sin(alphaf); ...
                      0, 1, 0, cos(betaf)*sin(alphaf), -rhof*sin(alphaf)*sin(betaf),  rhof*cos(alphaf)*cos(betaf); ...
                      0, 0, 1, sin(betaf), rhof*cos(betaf), 0];

            xRefPonto = jacob*qRefPonto;

            K = [cos(pos(3,k)) sin(pos(3,k)) 0 0 0 0; ...
                 -sin(pos(3,k))/a cos(pos(3,k))/a 0 0 0 0; ...
                 0 0 1 0 0 0; ...
                 0 0 0 cos(phid) -sin(phid) 0; ...
                 0 0 0 sin(phid) cos(phid) 0; ...
                 0 0 0 0 0 1];
            
            v{k} = K*xRefPonto;
            
            
            %impressão atualizada
            delete(Ho)
            axis([-4 4 -4 4 0 4]);  
            view([-40.0,20.0]);
            Ho=Robot_Plot_3D(pos(1,k),pos(2,k),pos(3,k),'g');hold on
            H1.XData = pos(1,k);
            H1.YData = pos(2,k);
            H2.XData = pos(1,2:k);
            H2.YData = pos(2,2:k);hold on
            H1d.XData = x(k);
            H1d.YData = y(k);
            H1d.ZData = z(k);hold on
            H2d.XData = x(1:k);
            H2d.YData = y(1:k);
            H2d.ZData = z(1:k);
            grid on;
            hold off;

            %velocidades do pioneer a serem ajustadas no compensador dinamico
            vD(1,k) = v{k}(1);
            vD(2,k) = v{k}(2);                       

            %Controlador dinâmico do Pioneer
            if k==1
                Vd_ponto(:,k)=(vD(:,k)-0)/sampleTime;
            else
                Vd_ponto(:,k)=(vD(:,k)-vD(:,k-1))/sampleTime;
            end

            vErr(:,k) = vD(:,k) - vRobot(:,k); % cálculo erros de velocidade

            C = [0 -theta(3)*vRobot(2,k);theta(3)*vRobot(2,k) 0];
            F = [theta(4) 0;0 theta(6)+(theta(5)-theta(3)*vRobot(1,k))];
            T_v = [lu 0;0 lw]*[tanh(vErr(1,k)*ku/lu);tanh(vErr(2,k)*kw/lw)];

            vRef(:,k)= T_v + H*Vd_ponto(:,k) + C*vD(:,k) + F*vD(:,k);  %Lei Controle dinâmica

           % Envia a velocidade para o robô
            escribirRobot(vRef(1,k),vRef(2,k));
            
            B.pSC.Ud(1) =  v{k}(4)*ganho; % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
            B.pSC.Ud(2) =  -v{k}(5)*ganho; % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
            B.pSC.Ud(3) =  v{k}(6)*ganho; % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
            B.pSC.Ud(4) = 0; % Angulo do drone [-1,1] (+) rotaciona para Direita em torno do Eixo Z 
            B.pSC.Ud(5) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda 
            B.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z 
                  
            % Envia Comando para o Drone
            B.rCommand;            
            drawnow;                              
            
            % Atualiza valor de k
            k = k+1;   
            
        end
        
     
%         %Print Loop
%         if toc(tp) > drawTime
%             tp = tic;
%                                    
%         end
  
        
    catch Exception     
          B.rLand;
          escribirRobot(0,0);
          disp("Pouso forçado via comando da estrutura Try Catch");
          disp(toStringJSON(Exception));
          break;
    end
end

disp(" ");
disp("Final da Programa .... Controlador OFF...")
disp(" ");


% Ao final, para o robô
escribirRobot(0,0);

% Send n times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    B.rLand
    pause(dLandMsg);
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Comando de Pouso ao final do programa");




% figure(2)
% plot(t(2:end),qTil(1:2,2:end),'LineWidth',2);hold on;
% plot(t(2:end),qTil(4:5,2:end),'LineWidth',2)
% grid on
% legend('Xf','Yf', 'rhof', 'Ângulo');
% title('Erros na formação');
% xlabel('tempo [s]');ylabel('Erro');

