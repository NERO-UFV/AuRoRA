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

%% Propriedades do grafo quadrado
Tamanho = 5;
Chance = 0;

%% Definindo o Robo
P = Pioneer3DX;
P.rConnect;
% pause(5)

%% Iniciando as variáveis do grafo
Obstaculo = randi(100,Tamanho)/100 < Chance;
Vertices = zeros(Tamanho^2 - sum(sum(Obstaculo)),3);
Nomes = zeros(Tamanho);
matrizAdjacencia = zeros(Tamanho^2 - sum(sum(Obstaculo)));
Custos = inf(Tamanho^2 - sum(sum(Obstaculo)));

%% Criando o pré grafo
cont = 1;
for j = 1:Tamanho
    for i = 1:Tamanho
        if Obstaculo(j,i) == 0
            Nomes(j,i) = cont;
        end
        cont = cont + 1;
    end
end

%% Selecionando a posição inicial e final
while true
    Sinit = randi(Tamanho^2);
    if find(Sinit==Nomes) ~= 0
        [X,Y] = find(Sinit==Nomes);
        Sinit_pos = [0 X Y];
        Sg = Sinit;
        break
    end
end
while true
    Sg = randi(Tamanho^2);
    if find(Sg == Nomes) ~= 0 
        if Sg ~= Sinit
            [X,Y] = find(Sg==Nomes);
            Sg_pos = [0 X Y];
            if distanciaManhattan(Sinit_pos,Sg_pos) >= Tamanho
                break
            end
        end
    end
end
Sinit = 1;
Sg = Tamanho^2;

%% Criação do grafo
% G = (V,a), V = (a,a')
f = figure(1);
hold on
grid on
axis([0 Tamanho+2 0 Tamanho+2])
axis equal
% axis ij
for j = 1:Tamanho+1
    for i = 1:Tamanho+1
        if i < Tamanho+1 && j < Tamanho+1 && Obstaculo(j,i) == 0
            Vertices(Nomes(j,i),:) = [Nomes(j,i) i+.5 j+.5];
            Mapa.Plot.Texto(Nomes(j,i)) =...
                text(Vertices(Nomes(j,i),2)-.25,Vertices(Nomes(j,i),3)-.25,...
                num2str(Nomes(j,i)),...
                'HorizontalAlignment','center',...
                'VerticalAlignment','middle');
            if i < Tamanho && Obstaculo(j,i+1) == 0 % Horizontal
                Mapa.Plot.Aresta.Horizontal(Nomes(j,i)) =...
                    plot([Vertices(Nomes(j,i),2) Vertices(Nomes(j,i),2)+1],...
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
                    [Vertices(Nomes(j,i),3) Vertices(Nomes(j,i),3)+1],'--b',...
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
        if i < Tamanho+1
            plot([i i+1],[j j],'-k','LineWidth', 2); % Horizontal
        end
        if j < Tamanho+1
            plot([i i],[j j+1],'-k','LineWidth', 2); % Vertical
        end
    end
end
Mapa.Plot.Vertice(Sinit).MarkerEdgeColor = 'k';
Mapa.Plot.Vertice(Sinit).MarkerFaceColor = 'k';
Mapa.Plot.Vertice(Sg).MarkerEdgeColor = 'r';
Mapa.Plot.Vertice(Sg).MarkerFaceColor = 'r';


%% Algoritmo de busca
Sinit = Vertices(Sinit,:);
Sg = Vertices(Sg,:);
try
    Caminho = Dijkstra(Sinit,Sg,Nomes,Vertices);
    Caminho = Astar(Sinit,Sg,Nomes,Vertices);
catch
    error('CAMINHO INEXISTENTE')
end
% disp(Caminho)
P.rSetPose([Sinit(2) Sinit(3) 0 0]);

%% Simulação para tirar obstáculo
% Iniciando as variaveis
tmax = 10;
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

pause
set(f,'WindowButtonDownFcn',@clickFunctionfloor)

t = tic;

% for cc = 1:size(Caminho,2)-1
cc = 1;
while cc < size(Caminho,2)
while toc(t) < tmax
    if toc(ts) > ta
        try
            nomeClick = Nomes(clickPoint(1,2),clickPoint(1,1));
            delete(Mapa.Plot.Vertice(nomeClick))
            delete(Mapa.Plot.Texto(nomeClick))
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
                'LineWidth', 2,...
                'MarkerSize',50);
            Nomes(clickPoint(1,2),clickPoint(1,1)) = 0;
            Vertices(nomeClick,:) = [0 0 0];
            % Algoritmo de busca
            Satu = Vertices(Caminho(cc),:);
            temp_cc = Caminho(cc);
            Sg = Vertices(Sg(1),:);
            try
                Caminho = Astar(Satu,Sg,Nomes,Vertices);
            catch
                error('CAMINHO INEXISTENTE')
            end
%             disp(Caminho)
            cc = find(temp_cc == Caminho);
            clickPoint = [];
        end
        drawnow
        ts = tic;
    end
    if toc(tp) > ta
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
        
         %% Pegando os dados do robo
        P.rGetSensorData;
        
        %% Erro
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
        
        %% Controlador
        % Constantes do controlador
        Ka = 0.8; 
%         Ka (Robo) = 0.2;
%         Ka = 0.8;       
%         Ka = 0.65;
        Kb = Ka;
%         G1 = 0.5;   
%         G1 (Robo) = 0.2;
%         G1 = 0.2;
        G1 = 1.06;
        G2 = G1;
        G = [G1 0;
             0 G2];
        % Definindo a Função de Controle
        A = [P.pPos.Xtil(1)/sqrt(P.pPos.Xtil(1)^2 + Ka^2);
             P.pPos.Xtil(2)/sqrt(P.pPos.Xtil(2)^2 + Kb^2)];
        % Definindo o Controlador
        K = [cos(P.pPos.X(6)) -P.pPar.a*sin(P.pPos.X(6));
             sin(P.pPos.X(6)) P.pPar.a*cos(P.pPos.X(6))];
        P.pSC.Ud = K\(P.pPos.Xd([7,8]) + G*A);

        %% Controlador de Orientação
        % Calculando o Ângulo necessário
%         Psir = atan2((P.pPos.Xd(2)-P.pPos.Xc(2)),(P.pPos.Xd(1)-P.pPos.Xc(1)));
%         % Iniciando o Controle
%         ANG = abs(P.pPos.Xc(6) - Psir)*180/pi;
% %         disp(ANG)
%         % Corrigindo a singularidade caso Psi desejado seja 180 graus      
%         if ANG == 180
%             P.pSC.Ud(1) = 0;
%             P.pSC.Ud(2) = 1;
%         end
%         % O primeiro limite para o controlador sera de 5 graus
%         if (ANG > 5 && abs(P.pSC.Ud(2)) > 0.1) && Rep == 0
%             P.pSC.Ud(1) = 0;
%         elseif Rep == 0
%             Rep = 1;
%             Troca = 1;
%         end
%         if Rep == 1
%             P.pSC.Ud(2) = 0;
%         end
%         % Caso Psi desejado seja maior que 10 graus, ele ira corrigir        
%         if ANG > 10 && Rep == 1
%             Rep = 0;
%             Troca = 1;
%         end
        
        %% Enviando sinais para o robo
        P.rSendControlSignals;
        
        %% Plot da simulação
        % Plot do robo
        P.mCADdel
        P.mCADplot(1,'r')
        try
            delete(h)
        end
        
        % Matriz de dados
        Dados = [Dados;
            [P.pPos.Xd' P.pPos.X' P.pPos.Xtil' P.pSC.Ud' P.pSC.Ur' P.pSC.U' toc(t)]];
        % Plot do destino
        h(1) = plot(Dados(:,1), Dados(:,2),'--r','LineWidth',2);
        h(2) = plot(Dados(:,13), Dados(:,14),'-k','LineWidth',2);
        
        drawnow
        tp = tic;
    end
end
Mapa.Plot.Vertice(Caminho(cc+1)).MarkerEdgeColor = 'k';
Mapa.Plot.Vertice(Caminho(cc+1)).MarkerFaceColor = 'k';
t = tic;
cc = cc + 1;
end

P.rDisconnect