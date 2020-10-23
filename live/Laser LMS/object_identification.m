%% Line reactive navigation v2 (03/02/2020)
%% Boas práticas

% //TODO: Orientação vem do mapa, velocidade maxima desejada é parametro
% livre. Logo, colocar na forma de seguimento de caminho.

close all
clearvars
clc

%% Carregando os objetos do cenário
V = VREP;
V.vConnect
V.vHandle('Pioneer_p3dx');
H = HandlePushObj;

%% Definindo o Robô
P = Pioneer3DX;
P.pPar.a = 0.15;%0.3
P.pPar.alpha = 0;

% figure(2)
% subplot(121)
% hold on
% grid on
% title('Simulação');
% axis([-5 5 -5 5])
% P.mCADplot2D('r')   % Visualização 2D
% drawnow

disp('Stopping Pioneer')
P.pSC.Ud = [0; 0];
V.vSendControlSignals(P,1);

pause(1);
% [p,~] = V.vGetObjPosition('Disc');

%% Inicializando variáveis para o controle
%Declarando o trace:
dados=[];
V.vGetSensorData(P,1);

%Parâmetros Line
LinMap=zeros(8,2); Map=[]; hmed = []; Hist = [];
tp = tic;

%% Rotina da simulação:
t=tic;  ta=tic; tL = tic; tmax=60;

k1=0.8; k2=0.8; k3 = 1.5; it = 0;

%% Controle e trajetória
% J = JoyControl;
% J.pSNES = 1;
Pilot = 0;

r = 1; w = 30*2*pi/(tmax);
time = linspace(0,2*pi,100);
% plot(r.*abs((cos(time).^(1/2))).*sign(cos(time)),...
%      r.*abs((sin(time).^(1/2))).*sign(sin(time)),'k-')

% plus = plot(P.pPos.Xd(1),P.pPos.Xd(2),'r+','MarkerSize',7,'LineWidth',1.2);

% figure(2)
% s = plot(P.pPos.Xd(1),P.pPos.Xd(2),'r+','MarkerSize',7,'LineWidth',1.2);
% cdr1 = plot(P.pPos.Xd(1),P.pPos.Xd(2),'r+','MarkerSize',7,'LineWidth',1.2);
% cdr2 = plot(P.pPos.Xd(1),P.pPos.Xd(2),'r+','MarkerSize',7,'LineWidth',1.2);

idx = [];

while toc(t)<tmax*5
    %% Get Laser Data
    if toc(tL)>0.5
        tL = tic;
        %Line:
        Map = V.vGetLaserData(P,1); 
        if ~isempty(Map)
          D = diff(Map(:,3));
          dD = diff(D);
          
          if(max(abs(D))>0.3)
            idx = find(abs(D)> sum(abs(D))/181);
            idx((sign(D(idx))<0)) =  idx((sign(D(idx))<0)) + 1;
            idx(idx(1:end-1)==idx(2:end)) = [];
          else
            idx = [];
          end
        else
            idx = [];
        end
        
    end
    
    %% Robot control
    if toc(ta)>0.1
        ta=tic;  it=it+1;
        
        %Pegar informação da posição e velocidade real do robô
        V.vGetSensorData(P,1);
        
        % Planejamento:
        % //TODO: identificar objetos

        if ~isempty(Map)
            figure(2)
%             subplot(122)
            plot(Map(:,1),Map(:,2),'k.')
            hold on
            plot(Map(idx,1),Map(idx,2),'r.','LineWidth',2,'MarkerSize',15)
%             plot3(Map(:,1),Map(:,2),Map(:,3),'*')
%             hold on
%             plot3(Map(idx,1),Map(idx,2),Map(idx,3),'r.','LineWidth',2,'MarkerSize',15)
%             grid on
            %% Bounding boxes:
            if ~isempty(idx)                
                try
                   delete(r1)
                end
                dx = abs(diff(Map(idx,1)));
                dy = abs(diff(Map(idx,2)));
                for ii = 1:size(dx,1)
                    
                    
%                     for k = 1:size(idx,1)-1
%                         if abs(Map(idx(k),3)-Map(idx(k+1),3))<.1
%                             r1 = rectangle('Position',[Map(idx(ii+1),1) Map(idx(ii+1),2)  dx(ii) dy(ii)]');
%                             r1.EdgeColor = [1 0 0];
%                             r1.LineStyle = '--';
%                             r1.LineWidth = 1.2;
%                         end
%                     end
                    if ii+1 <size(dx,1)
                        SS = sum(abs(diff(Map(idx(ii):idx(ii+1),3))));
                        if SS<.15 && SS>0.08
    %                         (dx(ii)<.3 && dy(ii)<.3) && (dx(ii)>.1 && dy(ii)<.1)
    %                         &&(dy(ii+1)>0.8 && dy(ii+1)<1.2)
                            r1 = rectangle('Position',[Map(idx(ii+1),1) Map(idx(ii+1),2)  .5 .5]');
    %                         viscircles([Map(idx(ii),1),Map(idx(ii),2)],0.5);
                            r1.EdgeColor = [0 1 0];
                            r1.LineStyle = '--';
                            r1.LineWidth = 1.2;
                        end
                    end
%                     
%                     if dx(ii)<.3
%                         dx(ii) = .3;
%                     end
%                     if dy(ii)<.3
%                         dy(ii) = .3;
%                     end
                    
%                     if ii+1 <size(dx,1)
%                         if (dy(ii)<.1 && dy(ii+1)<.1) && dx(ii)<0.1
%     %                         (dx(ii)==.3 && dy(ii)==.3)&& (idx(ii)-idx(ii+1))<7
%     %                         &&(dy(ii+1)>0.8 && dy(ii+1)<1.2)
%                             r1 = rectangle('Position',[Map(idx(ii+1),1) Map(idx(ii+1),2)  .5 .5]');
%     %                         viscircles([Map(idx(ii),1),Map(idx(ii),2)],0.5);
%                             r1.EdgeColor = [1 0 0];
%                             r1.LineStyle = '--';
%                             r1.LineWidth = 1.2;
%                         end 
%                     end
                    if (SS>4)&&(SS<5.7)
%                         (dx(ii)>1 && dx(ii)<1.7)&& dy(ii)>0.3 && 
                            r2 = rectangle('Position',[Map(idx(ii),1) Map(idx(ii),2)  dx(ii) 1.3]');
                            r2.EdgeColor = [0 0 1];
                            r2.LineStyle = '--';
                            r2.LineWidth = 1.2;
                    end
                end
            end
            hold off
        end
        axis([-10 10 -10 10])
        
        % Trajetória desejada:
        % Circunferência:
        %P.pPos.Xd(1) = r*cos(w*toc(t)/tmax);
        %P.pPos.Xd(2) = r*sin(w*toc(t)/tmax);
        %P.pPos.Xd(7) = -r*w/tmax*sin(toc(t)/tmax);
        %P.pPos.Xd(8) = r*w/tmax*cos(toc(t)/tmax);

        % Squircle:
        P.pPos.Xd(1) = r*abs((cos(w*toc(t)/tmax)^(1/2)))*sign(cos(w*toc(t)/tmax));
        P.pPos.Xd(2) = r*abs((sin(w*toc(t)/tmax)^(1/2)))*sign(sin(w*toc(t)/tmax));
        P.pPos.Xd(7) = r*w/tmax*abs((sin(w*toc(t)/tmax)^(1/2)))*sign(sin(w*toc(t)/tmax));
        P.pPos.Xd(8) = r*w/tmax*abs((cos(w*toc(t)/tmax)^(1/2)))*sign(cos(w*toc(t)/tmax));
        
        % Controle:        
%         P.pPos.Xd = P.pPos.X;
        %adicionar pov no JoyControl!
%         [P,tmax,Pilot] = pSafetyControl(P,J,Pilot,tmax);
        P = fKinematicControllerExtended(P,[k1 k2 k3]);
       
        % Armazenar dados da simulação
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];

        % Enviar sinais de controle para o robô
%         V.vSendControlSignals(P,1);        
        Hist = [P.pPos.Xc,Hist];
        
        % Sub-rotina para plotar
%         if toc(tp) > 0.1
%             figure(2)
%             subplot(121)
%             delete(plus)
%             plus = plot(P.pPos.Xd(1),P.pPos.Xd(2),'r+','MarkerSize',7,'LineWidth',1.2);
%             
%             tp = tic;
%             % Plot da simulação
%             P.mCADdel
%             P.mCADplot2D('r')   % Visualização 2D
            drawnow
%         end
        
    end
end

if Pilot == 0
    disp('Press START to quit.')
    J.pDigital(5) = 1;
    while ~J.pDigital(end)
        if toc(ta)>0.1
            ta=tic;
            V.vGetSensorData(P,1); 
            [P,tmax,Pilot] = pSafetyControl(P,J,Pilot,tmax);
            P = fKinematicControllerExtended(P);
            V.vSendControlSignals(P,1);  
            
            % Sub-rotina para plotar
%             figure(2)
%             if toc(tp) > 0.1
%                 delete(plus)
%                 plus = plot(P.pPos.Xd(1),P.pPos.Xd(2),'r+','MarkerSize',7,'LineWidth',1.2);

%                 tp = tic;
                % Plot da simulação
%                 P.mCADdel
%                 P.mCADplot2D('r')   % Visualização 2D
%                 drawnow
%             end
        end
    end
end

%% Desconecta Matlab e V-REP
P.pSC.Ud([1 2]) = 0;
V.vSendControlSignals(P,1);
V.vDisconnect;

%% Gráficos
% figure(1)
% subplot(121)
% plot(Map(:,3),'k.','MarkerSize',15)
% plot(Map(:,3),'Marker','.','Color', [0,0.447,0.741],'MarkerSize',15)
% hold on
% plot(idx,Map(idx,3),'r.','MarkerSize',15)
% grid on
for k = 1:size(idx,1)-1
    if abs(idx(k+1)-idx(k))<5
        plot(idx([k,k+1]),Map(idx([k,k+1]),3),'y--')
        if abs(Map(idx(k),3)-Map(idx(k+1),3))<.1
            plot(idx([k,k+1]),Map(idx([k,k+1]),3),'c--')
        end
    end
    
    
    
    if abs(Map(idx(k),3)-Map(:,3))<.2
%         (norm(Map(idx(k),3)-Map(:,3))<0.15)
%         plot(idx([k,k+1]),Map(idx([k,k+1]),3),'r--')
        try
            delete(r)
        end
        dx = abs(Map(idx(k+1),1)-Map(idx(k),1));
        dy = abs(Map(idx(k+1),2)-Map(idx(k),2));
        if dx<.3
            dx = .3;
        end
        if dy<.3
            dy = .3;
        end
        r = rectangle('Position',[Map(idx(k),1) Map(idx(k),2)  dx dy]');
        r.EdgeColor = [1 0 0];
        r.LineStyle = '--';
        r.LineWidth = 1.2;
    end
    
    
%     if sum(diff(Map(idx(k):idx(k+1),3)))<.05
%         abs(idx(k+1)-idx(k))>8 &&
%         abs(idx(k+1)-idx(k))<11
%         plot(idx([k,k+1]),Map(idx([k,k+1]),3),'r--')
%     end
    
end


% subplot(122)
% plot(abs(D(:)),'k.','MarkerSize',15)
% plot(abs(D(:)),'Marker','.','Color', [0,0.447,0.741],'MarkerSize',15)
%[0.301,0.745,0.933])
% hold on
% plot(idx,abs(D(idx)),'r.','MarkerSize',15)
% grid on