try
    P.rDisconnect
end

clear all
close all
clc

try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Carregando Classes

% Robots
P = Pioneer3DX(1);
P.rConnect;

% Obstáculos
% NumObs = 6;
% Obstaculos = zeros(1,NumObs);
% for i = 1:NumObs
%     A{i} = ArDrone(i + 10);
% end

% Joystick
% J = JoyControl;

% Create OptiTrack object and initialize
% OPT = OptiTrack;
% OPT.Initialize;

% ROS
% ....


%% CRIANDO O GRAFO
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% Propriedades do grafo quadrado
Dist = 0.6;
Tamanho = 5;
Deslocamento = Dist + Tamanho*Dist/2;
Chance = 0;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% Iniciando as variáveis do grafo
Obstaculo = randi(100,Tamanho)/100 < Chance;
Vertices = zeros(Tamanho^2 - sum(sum(Obstaculo)),3);
Nomes = zeros(Tamanho);
matrizAdjacencia = zeros(Tamanho^2 - sum(sum(Obstaculo)));
Custos = inf(Tamanho^2 - sum(sum(Obstaculo)));

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% Criando o pré grafo
cont = 1;
for j = 1:Tamanho
    for i = 1:Tamanho
        if Obstaculo(j,i) == 0
            Nomes(j,i) = cont;
        end
        cont = cont + 1;
    end
end

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% Selecionando a posição inicial e final
% while true
%     Sinit = randi(Tamanho^2);
%     if find(Sinit==Nomes) ~= 0
%         [X,Y] = find(Sinit==Nomes);
%         Sinit_pos = [0 X Y];
%         Sg = Sinit;
%         break
%     end
% end
% while true
%     Sg = randi(Tamanho^2);
%     if find(Sg == Nomes) ~= 0 
%         if Sg ~= Sinit
%             [X,Y] = find(Sg==Nomes);
%             Sg_pos = [0 X Y];
%             if distanciaManhattan(Sinit_pos,Sg_pos) >= Tamanho
%                 break
%             end
%         end
%     end
% end

Sinit = 1;
Sg = Tamanho^2;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% Criação do grafo
% G = (V,a), V = (a,a')
f = figure(1);
hold on
% grid on
axis([0-Deslocamento (Tamanho+2)*Dist-Deslocamento...
    0-Deslocamento (Tamanho+2)*Dist-Deslocamento])
axis equal
% axis ij
for j = 1:Tamanho+1
    for i = 1:Tamanho+1
        if i < Tamanho+1
            plot([i*Dist-Deslocamento (i+1)*Dist-Deslocamento],...
                [j*Dist-Deslocamento j*Dist-Deslocamento],...
                '-k','LineWidth', 0.8); % Horizontal
        end
        if j < Tamanho+1
            plot([i*Dist-Deslocamento i*Dist-Deslocamento],...
                [j*Dist-Deslocamento (j+1)*Dist-Deslocamento],...
                '-k','LineWidth', 0.8); % Vertical
        end
        if i < Tamanho+1 && j < Tamanho+1 && Obstaculo(j,i) == 0
            Vertices(Nomes(j,i),:) = [Nomes(j,i) (i+.5)*Dist-Deslocamento (j+.5)*Dist-Deslocamento];
            Mapa.Plot.Texto(Nomes(j,i)) =...
                text(Vertices(Nomes(j,i),2)-.25*Dist,Vertices(Nomes(j,i),3)-.25*Dist,...
                num2str(Nomes(j,i)),...
                'HorizontalAlignment','center',...
                'VerticalAlignment','middle');
            Mapa.Plot.Custo(Nomes(j,i)) =...
                text(Vertices(Nomes(j,i),2)+.25*Dist,Vertices(Nomes(j,i),3)+.25*Dist,...
                '\infty',...
                'HorizontalAlignment','center',...
                'VerticalAlignment','middle');
            if i < Tamanho && Obstaculo(j,i+1) == 0 % Horizontal
                Mapa.Plot.Aresta.Horizontal(Nomes(j,i)) =...
                    plot([Vertices(Nomes(j,i),2) (Vertices(Nomes(j,i),2)+1*Dist)],...
                    [Vertices(Nomes(j,i),3) Vertices(Nomes(j,i),3)],'--b',...
                    'LineWidth',1.3);
                matrizAdjacencia(Nomes(j,i),Nomes(j,i+1)) = 1;
                matrizAdjacencia(Nomes(j,i+1),Nomes(j,i)) = 1;
                Custos(Nomes(j,i),Nomes(j,i+1)) = 1;
                Custos(Nomes(j,i+1),Nomes(j,i)) = 1;
            end
            if j < Tamanho && Obstaculo(j+1,i) == 0 % Vertical
                Mapa.Plot.Aresta.Vertical(Nomes(j,i)) =...
                    plot([Vertices(Nomes(j,i),2) Vertices(Nomes(j,i),2)],...
                    [Vertices(Nomes(j,i),3) Vertices(Nomes(j,i),3)+1*Dist],'--b',...
                    'LineWidth',1.3);
                matrizAdjacencia(Nomes(j,i),Nomes(j+1,i)) = 1;
                matrizAdjacencia(Nomes(j+1,i),Nomes(j,i)) = 1;
                Custos(Nomes(j,i),Nomes(j+1,i)) = 1;
                Custos(Nomes(j+1,i),Nomes(j,i)) = 1;
            end
            Mapa.Plot.Vertice(Nomes(j,i)) =...
                plot(Vertices(Nomes(j,i),2),Vertices(Nomes(j,i),3),'ob',...
                'LineWidth',1.3,...
                'MarkerFaceColor','w');
            cont = cont + 1;
%         elseif i < Tamanho+1 && j < Tamanho+1
%             Arestas(cont,:) = [cont i+.5 j+.5];
%             cont = cont + 1;
        end
    end
end
Mapa.Plot.Vertice(Sinit).MarkerEdgeColor = 'k';
Mapa.Plot.Vertice(Sinit).MarkerFaceColor = 'k';
Mapa.Plot.Vertice(Sg).MarkerEdgeColor = 'r';
Mapa.Plot.Vertice(Sg).MarkerFaceColor = 'r';

%% ALGORITMO DE BUSCA
Sinit = Vertices(Sinit,:);
Sg = Vertices(Sg,:);
disp(Sg)
try
    [Caminho,FECHADO] = LPAstar_3(Sinit,Sg,Nomes,Vertices);
    clc
    Caminho = wrev(Caminho);
    disp(Caminho)
catch
    error('CAMINHO INEXISTENTE')
end
% disp(Caminho)
P.rSetPose([Sinit(2) Sinit(3) 0 0]);

%% Robots initial pose

% Xo = [0 0 0 0];
% P.rSetPose(Xo);    % define pose do robô
% P.pPos.X(1:2) = [0 0];

% idP = getID(OPT,P);            % pioneer ID on optitrack
% rb = OPT.RigidBody;            % read optitrack data
% P = getOptData(rb(idP),P);    % get pioneer data

%% SIMULAÇÃO
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% Iniciando as variaveis
tmax = 8;
ta = 0.1;
Dados = [];
contD = 1;

Destino = 0;
it = 1;
Rep = 0;
Repc = 0;
Troca = 0;

ts = tic;
tp = tic;
to = tic;

pause
pause(5)
set(f,'WindowButtonDownFcn',@clickFncfDD)

t = tic;

% for cc = 1:size(Caminho,2)-1
cc = 1;
Passou = Caminho(cc);
while cc < size(Caminho,2)
while toc(t) < tmax
% %- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%     % Obstaculo com OptiTrack
%     if toc(to) > 1
%         to = tic;
%         contObs = 0;
%         for i = 1:NumObs
%             
% %- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%             % Detectando Obstaculo
%             idA{i} = getID(OPT,A{i}) + contObs;            % pioneer ID on optitrack
%             contObs = contObs + 1;
%             if Obstaculos(1,i) ~= 1
%                 rb = OPT.RigidBody;            % read optitrack data
%                 A{i} = getOptData(rb(idA{i}),A{i});    % get pioneer data
% 
% %- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                 % Avaliando o Obstaculo
%                 DetectandoObstaculo(i)
%                 disp(obsPoint)
%                 if ~isempty(obsPoint)
%                     Obstaculos(1,i) = 1;
%                 end
%             end
% %- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%             try
%                 nomeClick = Nomes(obsPoint(1,2),obsPoint(1,1));
%                 delete(Mapa.Plot.Vertice(nomeClick))
%                 delete(Mapa.Plot.Texto(nomeClick))
%                 delete(Mapa.Plot.Custo(nomeClick))
%                 [sucClick,posSucClick] = mapaSucessor(Vertices(nomeClick),Nomes,Vertices);
%                 for i=1:size(sucClick,1)
%                     if posSucClick{i,2} == 'V'
%                         if posSucClick{i,1} == -1
%                             delete(Mapa.Plot.Aresta.Vertical(sucClick(i,1)))
%                         else
%                             delete(Mapa.Plot.Aresta.Vertical(nomeClick))
%                         end
%                     elseif posSucClick{i,2} == 'H'
%                         if posSucClick{i,1} == -1
%                             delete(Mapa.Plot.Aresta.Horizontal(sucClick(i,1)))
%                         else
%                             delete(Mapa.Plot.Aresta.Horizontal(nomeClick))
%                         end
%                     end
%                 end
%                 plot(Vertices(nomeClick,2),Vertices(nomeClick,3),...
%                     'Marker','x',...
%                     'MarkerEdgeColor','k',...
%                     'LineWidth', 1,...
%                     'MarkerSize',120);
%                 Nomes(obsPoint(1,2),obsPoint(1,1)) = 0;
%                 Vertices(nomeClick,:) = [0 0 0];
% %- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                 % Algoritmo de busca
%                 Satu = Vertices(Caminho(cc),:);
%                 temp_cc = Caminho(cc);
%                 Sg = Vertices(Sg(1),:);
%                 pp = find(nomeClick == Passou);
%                 if size(pp,2) == 0
%                 if nomeClick ~= 0
%                     valor = nomeClick;
%                     while size(valor,1) ~= 0
%                         valor_t = FECHADO(find(valor(end) == FECHADO(:,4)),1);
%                         FECHADO(find(valor(end) == FECHADO(:,1)),:) = [];
%                         valor = valor(1:(end-1),:);
%                         valor((end+1):(end+size(valor_t,1)),1) = valor_t;
%                         if size(valor,1) ~= 0
%                             Mapa.Plot.Custo(valor(end)).String = '\infty';
%                         end
%                     end
%                 end
%                 try
%                     Caminho_temp = Caminho;
%                     [Caminho,FECHADO] = LPAstar_2(Sinit,Sg,Nomes,Vertices,FECHADO);
% %                     disp(FECHADO)
%                     disp(Caminho)
%                 catch
%                     error('CAMINHO INEXISTENTE')
%                 end
%                 cc = find(temp_cc == Caminho);
%                 if size(cc,2) == 0
%                     try
%                         Atalho = DijkstraCaminho(Satu,Caminho,Nomes,Vertices);
%                         disp(Atalho)
%                         Caminho = caminhoAtualizado(Caminho,Atalho);
%                         disp(Caminho)
%                         cc = find(temp_cc == Caminho);
%                     catch
%                         error('CAMINHO INEXISTENTE')
%                     end
%                 end
%                 end
%                 obsPoint = [];
%             end
%             drawnow
%         end
%     end
    if toc(ts) > ta
        try
            nomeClick = Nomes(obsPoint(1,2),obsPoint(1,1));
            delete(Mapa.Plot.Vertice(nomeClick))
            delete(Mapa.Plot.Texto(nomeClick))
            delete(Mapa.Plot.Custo(nomeClick))
            [sucClick,posSucClick] = mapaSucessor(Vertices(nomeClick),Nomes,Vertices);
            for i=1:size(sucClick,1)
                if posSucClick{i,2} == 'V'
                    if posSucClick{i,1} == -1
                        delete(Mapa.Plot.Aresta.Vertical(sucClick(i,1)))
                    else
                        delete(Mapa.Plot.Aresta.Vertical(nomeClick))
                    end
                elseif posSucClick{i,2} == 'H'
                    if posSucClick{i,1} == -1
                        delete(Mapa.Plot.Aresta.Horizontal(sucClick(i,1)))
                    else
                        delete(Mapa.Plot.Aresta.Horizontal(nomeClick))
                    end
                end
            end
            plot(Vertices(nomeClick,2),Vertices(nomeClick,3),...
                'Marker','x',...
                'MarkerEdgeColor','k',...
                'LineWidth', 1,...
                'MarkerSize',50);
            Nomes(obsPoint(1,2),obsPoint(1,1)) = 0;
            Vertices(nomeClick,:) = [0 0 0];
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
            % Algoritmo de busca
            if nomeClick ~= 0
                valor = nomeClick;
                while size(valor,1) ~= 0
                    valor_t = FECHADO(find(valor(end) == FECHADO(:,4)),1);
                    FECHADO(find(valor(end) == FECHADO(:,1)),:) = [];
                    valor = valor(1:(end-1),:);
                    valor((end+1):(end+size(valor_t,1)),1) = valor_t;
                    if size(valor,1) ~= 0
                        Mapa.Plot.Custo(valor(end)).String = '\infty';
                    end
                end
            end
            Satu = Vertices(Caminho(cc),:);
            temp_cc = Caminho(cc);
            Sg = Vertices(Sg(1),:);
            try
                Caminho_temp = Caminho;
                [Caminho,FECHADO] = LPAstar_2(Sinit,Sg,Nomes,Vertices,FECHADO);
%                 disp(FECHADO)
                Caminho = wrev(Caminho);
                disp(Caminho)
            catch
                error('CAMINHO INEXISTENTE')
            end
            cc = find(temp_cc == Caminho);
%             if size(cc,2) == 0
%                 try
%                     Atalho = DijkstraCaminho(Satu,Caminho,Nomes,Vertices);
%                     disp(Atalho)
%                     Caminho = caminhoAtualizado(Caminho,Atalho);
%                     disp(Caminho)
%                     cc = find(temp_cc == Caminho);
%                 catch
%                     error('CAMINHO INEXISTENTE')
%                 end
%             end
            obsPoint = [];
        end
        drawnow
        ts = tic;
    end
    if toc(tp) > ta
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%         % Get data from robot and Optitrack
%         t_control = tic;
%         Rede.mReceiveMsg;
%         if length(Rede.pMSG.getFrom)>1
%             P.pSC.U  = Rede.pMSG.getFrom{2}(29:30);       % current velocities (robot sensors)
%             PX       = Rede.pMSG.getFrom{2}(14+(1:12));   % current position (robot sensors)
%         end
%         rb = OPT.RigidBody;             % read optitrack
%         P = getOptData(rb(idP),P);
        
        % Pegando os dados do robo
        P.rGetSensorData;
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Trajetória
        P.pPos.Xd(1) = Vertices(Caminho(cc),2) +...
            toc(t)/tmax*(Vertices(Caminho(cc+1),2) -...
            Vertices(Caminho(cc),2));
        P.pPos.Xd(2) = Vertices(Caminho(cc),3) +...
            toc(t)/tmax*(Vertices(Caminho(cc+1),3) -...
            Vertices(Caminho(cc),3));
        P.pPos.Xd(7) = (Vertices(Caminho(cc+1),2) -...
            Vertices(Caminho(cc),2))/tmax;
        P.pPos.Xd(8) = (Vertices(Caminho(cc+1),3) -...
            Vertices(Caminho(cc),3))/tmax;
%         plot(Xt,Yt,'.g','MarkerSize',15)
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Pegando os dados do robo
%         P.rGetSensorData;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%        
        % Controlador Dinâmico        
        % Ganhos pré-definidos
        cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];
%         cgains = [ 0.10  0.10  0.75  0.75  0.75  0.75  0.10  0.05 ];
        
        P = fDynamicController(P,cgains);     % Pioneer Dynamic Compensator

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Erro
%         P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Enviando sinais para o robo
%         Rede.mSendMsg(P);
        P.rSendControlSignals;
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % PLOT DA SIMULAÇÃO
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Plot do robo
        P.mCADdel
        P.mCADplot(1,'r')
        try
            delete(h)
        end
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Matriz de dados
        Dados = [Dados;
            [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Plot do destino
        h(1) = plot(Dados(:,1), Dados(:,2),'--r','LineWidth',2);
        h(2) = plot(Dados(:,13), Dados(:,14),'-k','LineWidth',2);
        h(3) = plot([P.pPos.Xd(1) Vertices(Caminho(cc+1),2)],...
            [P.pPos.Xd(2) Vertices(Caminho(cc+1),3)],'-g','LineWidth',1);
        for i = cc+1:(size(Caminho,2)-1)        
            h(i+3) = plot([Vertices(Caminho(i),2) Vertices(Caminho(i+1),2)],...
                [Vertices(Caminho(i),3) Vertices(Caminho(i+1),3)],'-g','LineWidth',1);
        end
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        % Detectando a existencia do caminho    
        Fechou = find(Sg(1) == FECHADO(:,1));
        if size(Fechou,1) == 0
            P.pSC.Ud = [0  ;  0];
            for ii = 1:500
                Rede.mSendMsg(P);
            end
            error('CAMINHO INEXISTENTE')
        end
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
        drawnow
        tp = tic;
    end
end
Passou(end+1) = Caminho(cc);
Mapa.Plot.Vertice(Caminho(cc+1)).MarkerEdgeColor = 'k';
Mapa.Plot.Vertice(Caminho(cc+1)).MarkerFaceColor = 'k';
t = tic;
cc = cc + 1;
end

%% Send control signals
P.pSC.Ud = [-0.2  ;  0];
for ii = 1:50
    Rede.mSendMsg(P);
end

%% Send control signals
P.pSC.Ud = [0  ;  0];
for ii = 1:500
    Rede.mSendMsg(P);
end
