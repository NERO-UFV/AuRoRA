%% Caminho de Bèzier - Alexandre Caldeira/Iure Rosa 13/01/20
%   Implementando planejamento de caminho por 
% curva de Bèzier de 5º grau com 2 parâmetros
% "aproximados".

%% Boas práticas
close all
clearvars
clc

%% Look for root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
try
    cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
    addpath(genpath(pwd))
catch
    disp('Wrong root directory')
end

%% Abrir V-REP
% cd([pwd,'\V-Rep_estavel\Scene'])
% ! scene.ttt
% 
% disp('Inicie a simulação!')
% pause(10)

%% Carregando os objetos do cenário
V = VREP;
V.vConnect
V.vHandle('Pioneer_p3dx');


%% Definindo o Robô
P = Pioneer3DX;
P.pPar.a = 0;
P.pPar.alpha = (0)*(pi/180);
% P.rConnect; % Descomentar para realiazação de experimento
P.rSetPose([0 0 0 0]);

disp('Stopping Pioneer')
P.pSC.Ud = [0; 0];
V.vSendControlSignals(P,1);

% pause(1);

%% Inicializando variáveis para o controle
%Declarando o trace:
dados=[];
Rastro.Xd = [];
Rastro.X = [];
V.vGetSensorData(P,1);

%% Variáveis para o caminho:
%Definindo variaveis:
xs  = 0;                        ys = 0;               %Posição inicial
xf  = 2;                        yf = 3.5;               %Posição final
ths = 0;                        thf =(75)*(pi/180); %Orientação inicial/final

Xtil = [sqrt((xf-xs)^2 +(yf-ys)^2) ,ths-thf];

%Pontos de controle:
b0 = [xs;ys];                   b5 = [xf;yf];   

c0 = 2;                         c5 = 2;
d0 = c0.*[cos(ths),sin(ths)];   d5 = c5.*[cos(thf),sin(thf)];
b1 = b0 + [(1/5).*d0]';         b4 = b5 - [(1/5).*d5]';

%Nosso:
b2 = b1 + [1/2.*d0]';          b3 = b4 - [1/2.*d5]';         %Inicialização aleatória

%Nosso 2.0:


ControlPoints = [b0,b1,b2,b3,b4,b5]; %=> N = 6;

%Plot da curva
t = 0:0.1:1;
N = 6;

Sum = 0;
for i=0:N-1
%        - Curva de Bèzier:
% 
%                   ( N-1 )    * (1-t)^(N-1-k) * t^(k) *  b_k
%                   (  k  )
    Sum = Sum + nchoosek(N-1,i)*(1-t).^(N-1-i).*(t.^i).*ControlPoints(:,i+1);
end
Curve = Sum;

% Sum = 0;
% for j=0:N-2
% %        - Derivada da curva de Bèzier:
% % 
% %                     ( N-2 )   *   (1-t)^(N-2-j)  *  t^(j)   * b_j
% %                     (  j  )
%     Sum = Sum +N*nchoosek(N-2,j)*(1-t).^(N-2-j).*(t.^j).*(ControlPoints(:,j+1)-ControlPoints(:,j+2));
% end
% dCurve = Sum;



% Diferança dos pontos de controle:
D=[diff(ControlPoints(1,:));diff(ControlPoints(2,:))];

% Diferença dos pontos de controle:
% for j=0:N-2
% D=[D,-ControlPoints(:,j+1)+ControlPoints(:,j+2)];
% end

% Brandao:
% Sum = 0;
% for k=0:N-2
% %        - Derivada da curva de Bèzier:
%     Sum = Sum + nchoosek(N-1,k).*ControlPoints(:,k+1).*((1-t).^(N-2-k)).*(t.^(k-1)).*(-t.*(N-1)+k);
% end
% dCurve = Sum;

Sum = 0;
for j=0:N-3
%        - Derivada da curva de Bèzier:
%                                   (N-2-j)       (j)
%                     ( N-2 )  *(1-t)         *  t     * D_j
%                     (  j  )
    Sum = Sum + nchoosek(N-2,j)*(1-t).^(N-2-j).*(t.^j).*D(:,j+1);

end
Sum = Sum + D(:,5);
dCurve = Sum.*(N-1);

% for j=0:N-2
%        - Derivada da curva de Bèzier:
% 
%                     ( N-2 )   *   (1-t)^(N-2-j)  *  t^(j)   * db_j
%                     (  j  )
%     Sum = Sum + nchoosek(N-2,j)*(1-t).^(N-2-j).*(t.^j).*ControlPoints(:,j+1).*D(:,j+1);
%     
%     (ControlPoints(:,j+2)-ControlPoints(:,j+1));
%     
% 
%     +N*nchoosek(N-2,j)*(1-time).^(N-2-j).*(time.^j).*ControlPoints(:,j+1).*(ControlPoints(:,j+2)-ControlPoints(:,j+1));
% end
% dCurve = (N).*Sum

plot(xs,ys,'k*')
hold on
plot(xf,yf,'r*')
hold on
plot(Curve(1,:),Curve(2,:),'b--')
hold on
plot(ControlPoints(1,:),ControlPoints(2,:),':.')
hold on
plot(ControlPoints(1,:),ControlPoints(2,:),'ro')
axis([-1 4 -1 4])
grid on

% figure
% hold on
% plot(1:size(dCurve,2),dCurve(1,:),'g--')
% 
% figure
% hold on
% plot(1:size(dCurve,2),dCurve(2,:),'g--')
% 
% Vetores velocidade:
% hold on
% quiver(Curve(1,:),Curve(2,:),dCurve(1,:),dCurve(2,:))
% 
% figure
% plot(dados(:,29),.02.*(atan(dCurve(2,:)./dCurve(1,:))))
        
%% Rotina da simulação:
t=tic;  ta=tic;   tp = tic;         tc=tic; tcmax=30;
        tmax=35;
k1=0.6; k2=0.4;   it = 0;


while toc(t)<tmax
    if toc(ta)>0.1
        ta=tic;
        
        P.pPos.Xda     = P.pPos.Xd;
        it=it+1;
        
       
        time = toc(tc)/(tcmax);
        if toc(tc)>tcmax
            time=1;
        end
        %Pegar informação da posição e velocidade real do robô
        V.vGetSensorData(P,1);
        
        %% Cálculo da curva:
        N = 6;
        
        
        Sum = 0;
        for i=0:N-1
%        - Curva de Bèzier:
% 
%                             ( N-1 )    * (1-t)^(N-1-k) *    t^(k) *  b_k
%                             (  k  )
            Sum = Sum + nchoosek(N-1,i)*(1-time).^(N-1-i).*(time.^i).*ControlPoints(:,i+1);
        end
        Curve = Sum;
        
%         Sum = 0;
        for j=0:N-3
%                - Derivada da curva de Bèzier:
%                                           (N-2-j)       (j)
%                             ( N-2 )  *(1-t)         *  t     * D_j
%                             (  j  )
            Sum = Sum + nchoosek(N-2,j)*(1-time).^(N-2-j).*(time.^j).*D(:,j+1);
        end
        Sum = Sum + D(:,5);
        dCurve = Sum.*(N-1);

%         Derivada Brandão:
%         Sum = 0;
%         for k=0:N-2
%         %        - Derivada da curva de Bèzier:
%             Sum = Sum + nchoosek(N-1,k).*ControlPoints(:,k+1).*((1-time)^(N-2-k)).*(time.^(k-1)).*(-time.*N+k);
%         end
%         dCurve = Sum;

%         Sum = 0;
%         for j=0:N-2
%         %        - Derivada da curva de Bèzier:
%         % 
%         %                     ( N-2 )   *   (1-t)^(N-2-j)  *  t^(j)   * db_j
%         %                     (  j  )
%             Sum = Sum + nchoosek(N-2,j)*(1-time).^(N-2-j).*(time.^j).*ControlPoints(:,j+1).*(ControlPoints(:,j+2)-ControlPoints(:,j+1));
%             
%             %+N*nchoosek(N-2,j)*(1-time).^(N-2-j).*(time.^j).*ControlPoints(:,j+1).*(ControlPoints(:,j+2)-ControlPoints(:,j+1));
%         end
%         dCurve = (N).*Sum;
        
%         r=1;
%         Sum = 0;
%         D=[];
%         Paper:
%         for j=0:r
%             D = [D,nchoosek(r,j)*((-1)^(r-j))*(ControlPoints(:,j+2)-ControlPoints(:,j+1))];
%         end      
        
        %% Robot control
        %Posição do ponto desejado:
        P.pPos.Xd(1)= Curve(1);
        P.pPos.Xd(2)= Curve(2);

        %Velocidade do ponto desejado:
%         L_erro =   -0.0786
%         L_erro =   -0.0710 com Xd(6)=atan
%         L_erro =   -0.0490 sem Xd(6)=atan
%         L_erro =   -0.0121 aumentando distancia dos pontos intermediarios..... DPI
%         L_erro =   -0.0056 aumentando mais a DPI
%         L_erro =   -0.0346 DPI >1 => não mt bom
%         L_erro =   -0.0081

%        Hipotese ditancia do caminho inv prop a distancia dos pts interm

%         p = atan(dCurve(2)/dCurve(1))*(180/pi);
%         P.pPos.Xd(6)= p;
        P.pPos.Xd(10)= (0.3/8)*sqrt(dCurve(1)^2 + dCurve(2)^2);
        P.pPos.Xd(11)= 42.314;
        
        P.pPos.Xd(7)= 0*dCurve(1)*0.02;
        P.pPos.Xd(8)= 0*dCurve(2)*0.02;
        
        V.vGetSensorData(P,1);

        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
        % Controlador Cinemático modelo extendido
        vx = P.pPos.Xd(7) + 0.75*P.pPos.Xtil(1);
        vy = P.pPos.Xd(8) + 0.75*P.pPos.Xtil(2);
                
%         vx = dCurve(1)*.02;
%         vy = dCurve(2)*.02;
         
        if abs(P.pPar.alpha) < pi/2 && P.pPar.a > 0
            vw = (-sin(P.pPos.X(6))*vx + cos(P.pPos.X(6))*vy)/(P.pPar.a*cos(P.pPar.alpha));
            P.pSC.Ud(2) = vw;
        else              
            P.pPos.Xd(6)  = atan2(vy,vx);
            P.pPos.Xd(12) = (P.pPos.Xd(6)-P.pPos.Xda(6))/0.1;            
            P.pPos.Xtil(6) = P.pPos.Xd(6) - P.pPos.X(6);
            if abs(P.pPos.Xtil(6)) > pi
                if P.pPos.Xtil(6) > 0
                    P.pPos.Xtil(6) = - 2*pi + P.pPos.Xd(6) - P.pPos.X(6);
                else
                    P.pPos.Xtil(6) =   2*pi + P.pPos.Xd(6) - P.pPos.X(6);
                end
            end
            vw = 0*(P.pPos.Xd(12)) +1.5*P.pPos.Xtil(6);
        end
        
%         P.pSC.Ud(2) = .02*(atan(dCurve(2)/dCurve(1)));
        P.pSC.Ud(2) = vw;
%         P.pSC.Ud(1) = .02*sqrt(dCurve(1)^2 + dCurve(2)^2);
        P.pSC.Ud(1) = vx*cos(P.pPos.X(6)) + vy*sin(P.pPos.X(6)) ...
                           + P.pPar.a*sin(P.pPar.alpha)*vw;

        
        % Armazenar dados da simulação
%         dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];
        Rastro.Xd = [Rastro.Xd; P.pPos.Xd(1:2)'];  % formação desejada
        Rastro.X  = [Rastro.X; P.pPos.X(1:2)'];    % formação real
        
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];
        
        % Enviar sinais de controle para o robô
        V.vSendControlSignals(P,1);     
        
          %% Desenha o robô
            if toc(tp) > 0.1
                tp = tic;
                
                P.mCADdel
                P.mCADplot2D('r')   % Visualização 2D

                hold on
                plot(Rastro.Xd(:,1),Rastro.Xd(:,2),'r-');
                hold on;
                plot(Rastro.X(:,1),Rastro.X(:,2),'black');
                
                
                drawnow
            end
    end
end

disp('Stopping Pioneer')
P.pSC.Ud([1 2]) = 0;
V.vSendControlSignals(P,1);

%% Desconecta Matlab e V-REP
V.vDisconnect;

%% Comprimento:
dt = diff(dados(:,29));

Lo = sum(sqrt(((diff(dados(:,13))./diff(dados(:,29))).^2)+((diff(dados(:,14))./diff(dados(:,29))).^2)).*dt);

Lc = sum(sqrt(((diff(dados(:,10))./diff(dados(:,29))).^2)+((diff(dados(:,11))./diff(dados(:,29))).^2)).*dt);

Ld = sum(sqrt(((diff(dados(:,1))./diff(dados(:,29))).^2)+((diff(dados(:,2))./diff(dados(:,29))).^2)).*dt);

L_erro = Ld-Lo

figure
plot(dados(:,29),dados(:,10),'r--')
hold on
plot(dados(:,29),dados(:,19),'k')
%% Segundo paper PhD Shanmugavel
%
% Sum = 0; delta=0;
% r = 1; % Derivada de ordem 1
% for j=0:r 
%    Sum = Sum + nchoosek(r,j).*(ControlPoints(:,i+j)).*(-1)^(r-j)
% end
% delta= Sum;
% 
% Sum = 0;
% for j=0:N-r-1
% %        - Derivada da curva de Bèzier:
% % 
% %                             n!             *  ( N-r )    *   (1-t)^(n-r-k)   *  t^(j)   * b_j
% %                           (n-r)!              (  j  )
%     Sum = Sum +(factorial(N)/(factorial(N-r)))*nchoosek(N-1-r,j)*(1-time).^(N-1-r-j).*(time.^j).*ControlPoints(:,j+1);
% end
% dCurve = Sum;
% 
















