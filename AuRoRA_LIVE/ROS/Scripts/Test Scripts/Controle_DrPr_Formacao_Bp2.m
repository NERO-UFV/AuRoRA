%% Tarefa 1

% Limpa as variï¿½vies
clear all; 
close all;
warning off;
clc;

% Carrega o diretï¿½rio corrente e subdiretï¿½rios 

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
    B = Bebop(1);        
    
    disp('### Load Class Success');
    
catch ME
    disp(' #### Load Class Issues ');
    disp('');
    disp(ME);
    rosshutdown;
    return;
end


%%
hg = figure(1); 
set(hg,'units','pix','pos',[800 100 1000 900],'PaperPositionMode','auto')
axis([-1 1 -1 1 -0.5 0.5])
grid on
drawnow

%% Inicializa a figura dos acelerï¿½metros 
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%% Pioneer  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define o tempo de amostragem (100ms)
sampleTime = 0.1;

%Define o tempo
t = 0:sampleTime:25; 

% Inicializa as variï¿½veis 
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

%Condiï¿½ï¿½es Iniciais de Posiï¿½ï¿½o do Drone
x(1) = 0;
y(1) = 0;
z(1) = 1;
phi(1) = 0;
phid = 0;

% Inicializa Parametros de configuraï¿½ï¿½o do teste
PhysMes = zeros(6,size(nn,2));
Angles = zeros(3,size(nn,2));
motor = zeros(4,size(nn,2));
k = 1;
idc =1;


%% Decolagem do Drone

% Assegura a decolagem vertical 
% Os valores de comando na decolagem sejam iguais a zero
B.pSC.Ud(1) = 0; % Esquerda/Direita [-1,1] (-) Move Drone para Esquerda
B.pSC.Ud(2) = 0; % Frente/Tras [-1,1] (-) Avanï¿½a - Move frente para baixo
B.pSC.Ud(3) = 0; % Velocidade Vertical [-1,1] (+) Eleva o drone
B.pSC.Ud(4) = 0; %
                 
%Inicio impressï¿½o do pioneer
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
qDes = [0 0 0 1 0 pi/2]';



% Ganho do Controledor do Drone Kp na saï¿½da de comando
% Reduzir este ganho para teste de novos parametros
ganho = 1;

% Tempo de Amostragem
tmax = 20; % Tempo Simulaï¿½ï¿½o em segundos
To = 1/30;
TIP = 0.1;

% =========================================================================
t  = tic; % Tempo de simulaï¿½ï¿½o
tc = tic; % Tempo de controle
tp = tic; % Tempo de plotagem e exibiï¿½ï¿½o
nLandMsg = 5;    % Numero de mensagens enviadas para pouso do drone
dLandMsg = 0.1 ; % Atraso de tempo entre messagens de pouso

countPrint = 0;
countWhile = 0;
countControl = 0;


% Envia Comando de Decolagem para o Drone e aguarda estabilizaï¿½ï¿½o de
% altitude padrï¿½o (8s)
B.rTakeOff;
pause(3);

disp(" ");
disp("Final da Estabilizacao .... Controlador ON...")
disp(" ");



%%
 try    
    while toc(t) < tmax
        countWhile = countWhile + 1;        
    
        if toc(tc) > To
            tc = tic;
            
            % Obter dados de voo
            B.rGetSensorData;            
            countControl = countControl + 1;
            
           
            % Atribuir variï¿½veis            
%             A.pPos.X(4) = A.pCom.cRawData(2)*pi/180; % Rotaï¿½ï¿½o em torno do eixo Y (+) Regra da Mao Direita
%             A.pPos.X(5) = A.pCom.cRawData(3)*pi/180; % Rotaï¿½ï¿½o em torno do eixo X (+) Regra da Mao Direita
%             A.pPos.X(6) = A.pCom.cRawData(4)*pi/180; % Rotaï¿½ï¿½o em torno do eixo Z (+) Regra da Mao Direita
            %SaveData(Files,A); % Save Data              
            
                                              
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                     DRONE
            
            % Capture Altura H e Velocities X,Y
            % Eixo Hz - Altura [-1,1] (+) O Drone estï¿½ em Voo
            % Eixo Vy - Esquerda/Direita [-1,1] (-) Move Drone para Esquerda
            % Eixo Vx - Frente/Tras [-1,1] (-) Avanï¿½a - Move frente para baixo
            % Eixo Vz - Velocidade Vertical [-1,1] (+) Eleva o drone                       
            % Angulo do drone [-1,1] (-) rotaciona para esquerda                                 
            
                       
            % Velocities
            for ii = 1:4                 
                    PhysMes(ii,k) = B.pPos.X(ii+6);           
                    trace(ii).YData = PhysMes(ii,end-size(nn,2)+1:end);                                                            
                    Nav(k,ii) = PhysMes(ii,k);
            end

            % Angles                       
            for ii = 1:3
                Angles(ii,k) = B.pPos.X(ii+9);
                tracet(ii+4).YData = Angles(ii,end-size(nn,2)+1:end);
                Nav(k,ii+4) = Angles(ii,k);
            end  
                                                 
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                     PIONEER
            
            
            % Leitura dos dados do pioneer                        
            [pos(1,k),pos(2,k),pos(3,k),vRobot(1,k),vRobot(2,k),~,~] = leerRobot();            
                            

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
            L  = .2*eye(6);
            kp = .1*eye(6);
%             kp = [.4, 0, 0, 0 ,  0   , 0;...
%                    0,.4, 0, 0 ,  0   , 0;...
%                    0, 0,.4, 0 ,  0   , 0;...
%                    0, 0, 0,0.1, 0   , 0;...
%                    0, 0, 0, 0 ,0.1 , 0;...
%                    0, 0, 0, 0 ,  0   ,0.1];
%                                 

            % Obs: Mauro    
            % Verificar este ponto, na transparï¿½ncia consta a soma de
            % qDesPonto
            % qRef = qDesPonto L*tanh(inv(L)*kp*qTil(:,k));
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

            %velocidades do pioneer a serem ajustadas no compensador dinamico
            vD(1,k) = v{k}(1);
            vD(2,k) = v{k}(2);
            
                        
            % Integracao da posicao 
            x(k+1) = x(k)+To*B.pPos.X(7);
            y(k+1) = y(k)+To*B.pPos.X(8);            
            z(k+1) = z(k)+To*B.pPos.X(9);

            myNavData(k,1) = x(k+1);
            myNavData(k,2) = y(k+1);
            myNavData(k,3) = z(k+1);                      
            
            x(k+1) = B.pPos.X(1);
            y(k+1) = B.pPos.X(2);
            z(k+1) = B.pPos.X(3);

            myNavData(k,4) = x(k+1);
            myNavData(k,5) = y(k+1);
            myNavData(k,6) = z(k+1);

            %fprintf('x(k+1) = %d ---- Pos(x) = %d \n',round(x(k+1),4),round(B.pPos.X,4));             
                                   
            phi(k+1) = 0;            

            %Controlador dinï¿½mico do Pioneer
            if k==1
                Vd_ponto(:,k)=(vD(:,k)-0)/To;
            else
                Vd_ponto(:,k)=(vD(:,k)-vD(:,k-1))/To;
            end

            vErr(:,k) = vD(:,k) - vRobot(:,k); % cï¿½lculo erros de velocidade

            C=[0 -theta(3)*vRobot(2,k);theta(3)*vRobot(2,k) 0];
            F=[theta(4) 0;0 theta(6)+(theta(5)-theta(3)*vRobot(1,k))];
            T_v=[lu 0;0 lw]*[tanh(vErr(1,k)*ku/lu);tanh(vErr(2,k)*kw/lw)];

            vRef(:,k)= T_v + H*Vd_ponto(:,k) + C*vD(:,k) + F*vD(:,k);  %Lei Controle dinï¿½mica

           % Envia a velocidade para o robï¿½
             escribirRobot(vRef(1,k),vRef(2,k));
            
           % [pos(1,k),pos(2,k),pos(3,k),vRobot(1,k),vRobot(2,k),~,~]           
           %xPosRobo = [0,0,0,vRef(1,k),vRef(2,k),0,0];

            % Envia Comandos Controlador    
            tt = toc(t);        
       
%           B.pSC.Ud(1) = v{k}(5)*ganho; % Esquerda/Direita [-1,1] (-) Move Drone para Esquerda            
%           B.pSC.Ud(2) = -v{k}(4)*ganho; % Frente/Tras [-1,1] (-) Avanï¿½a - Move frente para baixo
%           B.pSC.Ud(3) = v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
%           B.pSC.Ud(4) = 0; % Angulo do drone [-1,1] (-) rotaciona para esquerda 
%           B.pSC.Ud(5) = 0; % Angulo do drone [-1,1] (-) rotaciona para esquerda 
%           B.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (-) rotaciona para esquerda 
            
            B.pSC.Ud(1) = v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
            B.pSC.Ud(2) = -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
            B.pSC.Ud(3) = -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
            B.pSC.Ud(4) = 0; % Angulo do drone [-1,1] (+) rotaciona para Direita em torno do Eixo Z 
            B.pSC.Ud(5) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda 
            B.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z 
                      

            % Envia Comando para o Drone
            B.rCommand;
            
            
            % Atualiza valor de k
            k = k+1;   
            
        end
        
        if toc(tp) > TIP
            tp = tic;
            
%           countPrint = countPrint + 1;
                    
%             % Impressao atulizada
%             delete(Ho)
%             axis([-4 4 -4 4 0 4]);  
%             view([-40.0,20.0]);
%             Ho=Robot_Plot_3D(pos(1,k),pos(2,k),pos(3,k),'g');hold on
%             H1.XData = pos(1,k);
%             H1.YData = pos(2,k);
%             H2.XData = pos(1,2:k);
%             H2.YData = pos(2,2:k);hold on
%             H1d.XData = x(k);
%             H1d.YData = y(k);
%             H1d.ZData = z(k);hold on
%             H2d.XData = x(1:k);
%             H2d.YData = y(1:k);
%             H2d.ZData = z(1:k);
%             grid on;
            

        end 
             
    end
   
 catch Exception    
      B.rLand;          
      disp("Pouso forï¿½ado via comando da estrutura Try Catch");
      disp(toStringJSON(Exception));
end

% Ao final, para o robï¿½
escribirRobot(0,0);


% Send 3 times Commands 1 second delay to Drone Land
for i=1:3
    B.rLand
    pause(1);
end


% Close ROS Interface
RI.rDisconnect;
rosshutdown;


% Printing Figure
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


disp("Comando de Pouso ao final do programa");


figure(2)
plot(t(2:end),qTil(1:2,2:end),'LineWidth',2);hold on;
plot(t(2:end),qTil(4:5,2:end),'LineWidth',2)
grid on
legend('Xf','Yf', 'rhof', 'ï¿½ngulo');
title('Erros na formaï¿½ï¿½o');
xlabel('tempo [s]');ylabel('Erro');

