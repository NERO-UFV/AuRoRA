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
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

%% Propriedades do grafo quadrado
Dist = 0.6;
Tamanho = 10;
Chance = 0.3;
Obstaculos = 0;

% Obstaculos = [14 34 54 74 94 95 96 97 116 117 136 137 156 157 176 177 196 ...
%     197 195 194 193 192 211 230 250 270 290 310 331 332 333 334 335 336 337 ...
%     289 288 287 307 326 325 324 344 388 389 368 369 188 168 148 128 108 88 ...
%     68 48 132 131 130 129 128 127 126 125 124 123 241 242 243 244 245 225 ...
%     41 42 43 44 64 273 274 275 276 277 278 279 280]; % TAMANHO 20

% Obstaculos = [47 48 63 64 79 80 95 96 111 112 127 128 256 240 224 206 205 ...
%     204 203 202 190 189 188 187 186 174 173 172 171 170 138 139 140 106 107 ...
%     108 90 91 92 74 75 76 58 59 60 42 43 44 209 210 211 212 193 194 195 196 ...
%     247 248 231 232 215 216 199 200 183 184 18 19 20 21 22 23 34 35 36 37 ...
%     38 39 82 83 98 99 114 115 130 131 146 147 86 87 102 103 118 119 ...
%     134 135 150 151]; % TAMANHO 16

% Obstaculos = [8 29 39 69 70 23 16 55 56 45 46 43 42 41 91 81 83 84 85 ...
%     86]; % TAMANHO 10

Obstaculos = [44 54 64 65 66 67 68 58 48]; % TAMANHO 10

% Obstaculos = [16 26 36 46 56 66 76 38 48 58 68 78]; % TAMANHO 10

%% Definindo o Robo
P = Pioneer3DX;
% P.rConnect;

% pause(5)

%% Iniciando as variáveis do grafo

Obstaculo = randi(100,Tamanho)/100 < Chance;
Vertices = zeros(Tamanho^2 - sum(sum(Obstaculo)),3);
Nomes = zeros(Tamanho);
MapaAnt = zeros(Tamanho);
matrizAdjacencia = zeros(Tamanho^2 - sum(sum(Obstaculo)));
Custos = inf(Tamanho^2 - sum(sum(Obstaculo)));

%% Criando o pré grafo

cont = 1;
for j = 1:Tamanho
    for i = 1:Tamanho
%         if Obstaculo(j,i) == 0
        if sum(Obstaculos == cont) == 0
            Nomes(j,i) = cont;
        end
        MapaAnt(j,i) = cont;
        cont = cont + 1;
    end
end

%% Selecionando a posição inicial e final

Sinit = 1; % Tamanho 5 10
Sinit = 19;
% Sinit = 33;
% Sinit = 119; % Tamanho 20
% Sinit = 16; % Tamanho 16
Sg = Tamanho^2; % Tamanho 5 10
Sg = 81;
% Sg = 69;
% Sg = 166; % Tamanho 20
% Sg = 241; % Tamanho 16

%% Criação do grafo
% G = (V,a), V = (a,a')
f = figure(1);
hold on
grid off
axis([0 (Tamanho+2)*Dist 0 (Tamanho+2)*Dist])
axis equal
% axis ij
for j = 1:Tamanho+1
    for i = 1:Tamanho+1
        if i < Tamanho+1 %&& j < Tamanho/2+2
            plot([i*Dist (i+1)*Dist],[j*Dist j*Dist],...
                '-k','LineWidth', 0.8); % Horizontal
%                 'FaceAlpha',0.4,'EdgeAlpha',0.4); % Horizontal

%             plot([i*Dist*2-Dist (i+1)*Dist*2-Dist],[j*Dist*2-Dist j*Dist*2-Dist],...
%                 '-k','LineWidth', 0.8); % Horizontal
% %                 'FaceAlpha',0.4,'EdgeAlpha',0.4); % Horizontal

        end
        if j < Tamanho+1 %&& i < Tamanho/2+2
            plot([i*Dist i*Dist],[j*Dist (j+1)*Dist],...
                '-k','LineWidth', 0.8); % Vertical
%                 'FaceAlpha',0.4,'EdgeAlpha',0.4); % Vertical

%             plot([i*Dist*2-Dist i*Dist*2-Dist],[j*Dist*2-Dist (j+1)*Dist*2-Dist],...
%                 '-k','LineWidth', 0.8); % Vertical
% %                 'FaceAlpha',0.4,'EdgeAlpha',0.4); % Vertical

        end
        if i < Tamanho+1 && j < Tamanho+1 && Nomes(j,i) ~= 0 %Obstaculo(j,i) == 0
            Vertices(Nomes(j,i),:) = [Nomes(j,i) (i+.5)*Dist (j+.5)*Dist];
            VerticesAnt(Nomes(j,i),:) = [Nomes(j,i) (i+.5)*Dist (j+.5)*Dist];
            Mapa.Plot.Texto(Nomes(j,i)) =...
                text(Vertices(Nomes(j,i),2)-.25*Dist,Vertices(Nomes(j,i),3)-.25*Dist,...
                num2str(Nomes(j,i)),...
                'HorizontalAlignment','center',...
                'VerticalAlignment','middle',...
                'Visible','off');
            Mapa.Plot.Obstaculo(Nomes(j,i)) =...
                plot(Vertices(Nomes(j,i),2),Vertices(Nomes(j,i),3),...
                'Marker','x',...
                'MarkerEdgeColor','k',...
                'LineWidth', 1,...
                'MarkerSize',50,...
                'Visible','off');
            Mapa.Plot.Custo(Nomes(j,i)) =...
                text(Vertices(Nomes(j,i),2)+.25*Dist,Vertices(Nomes(j,i),3)+.25*Dist,...
                '\infty',...
                'HorizontalAlignment','center',...
                'VerticalAlignment','middle',...
                'Visible','off');
            if i < Tamanho &&  Nomes(j,i+1) ~= 0 %Obstaculo(j,i+1) == 0 % Horizontal
                Mapa.Plot.Aresta.Horizontal(Nomes(j,i)) =...
                    plot([Vertices(Nomes(j,i),2) (Vertices(Nomes(j,i),2)+1*Dist)],...
                    [Vertices(Nomes(j,i),3) Vertices(Nomes(j,i),3)],'--b',...
                    'LineWidth',1.3,'Visible','off');
                matrizAdjacencia(Nomes(j,i),Nomes(j,i+1)) = 1;
                matrizAdjacencia(Nomes(j,i+1),Nomes(j,i)) = 1;
                Custos(Nomes(j,i),Nomes(j,i+1)) = 1;
                Custos(Nomes(j,i+1),Nomes(j,i)) = 1;
            end
            if j < Tamanho &&  Nomes(j+1,i) ~= 0 %Obstaculo(j+1,i) == 0 % Vertical
                Mapa.Plot.Aresta.Vertical(Nomes(j,i)) =...
                    plot([Vertices(Nomes(j,i),2) Vertices(Nomes(j,i),2)],...
                    [Vertices(Nomes(j,i),3) Vertices(Nomes(j,i),3)+1*Dist],'--b',...
                    'LineWidth',1.3,'Visible','off');
                matrizAdjacencia(Nomes(j,i),Nomes(j+1,i)) = 1;
                matrizAdjacencia(Nomes(j+1,i),Nomes(j,i)) = 1;
                Custos(Nomes(j,i),Nomes(j+1,i)) = 1;
                Custos(Nomes(j+1,i),Nomes(j,i)) = 1;
            end
            Mapa.Plot.Vertice(Nomes(j,i)) =...
                plot(Vertices(Nomes(j,i),2),Vertices(Nomes(j,i),3),'ob',...
                'LineWidth',1.3,...
                'MarkerFaceColor','w','Visible','off');
            cont = cont + 1;
%         elseif i < Tamanho+1 && j < Tamanho+1
%             Arestas(cont,:) = [cont i+.5 j+.5];
%             cont = cont + 1;
        elseif i < Tamanho+1 && j < Tamanho+1 && Nomes(j,i) == 0
            VerticesObs(MapaAnt(j,i),:) = [MapaAnt(j,i) (i+.5)*Dist (j+.5)*Dist];
            Mapa.Plot.Obstaculo(MapaAnt(j,i)) =...
                plot(VerticesObs(MapaAnt(j,i),2),VerticesObs(MapaAnt(j,i),3),...
                'Marker','x',...
                'MarkerEdgeColor','k',...
                'LineWidth', 1,...
                'MarkerSize',70,...
                'Visible','on');
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
disp(Sg)
try
    [Caminho,FECHADO] = LPAstar(Sinit,Sg,Nomes,Vertices);
    clc
    disp(Caminho)
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
set(f,'WindowButtonDownFcn',@clickFunctionfloorDist)

t = tic;

% for cc = 1:size(Caminho,2)-1
cc = 1;
while cc < size(Caminho,2)
while toc(t) < tmax
    if toc(ts) > ta
        try
            nomeClick = Nomes(clickPoint(1,2),clickPoint(1,1));
            if Nomes(clickPoint(1,2),clickPoint(1,1)) ~= 0
                Mapa.Plot.Vertice(nomeClick).Visible = 'off';
                Mapa.Plot.Texto(nomeClick).Visible = 'off';
%                 Mapa.Plot.Custo(nomeClick).Visible = 'off';
                [sucClick,posSucClick] = mapaSucessor(Vertices(nomeClick),Nomes,Vertices);
                for i=1:size(sucClick,1)
                    if posSucClick{i,2} == 'V'
                        if posSucClick{i,1} == -1
                            Mapa.Plot.Aresta.Vertical(sucClick(i,1)).Visible = 'off';
                        else
                            Mapa.Plot.Aresta.Vertical(nomeClick).Visible = 'off';
                        end
                    elseif posSucClick{i,2} == 'H'
                        if posSucClick{i,1} == -1
                            Mapa.Plot.Aresta.Horizontal(sucClick(i,1)).Visible = 'off';
                        else
                            Mapa.Plot.Aresta.Horizontal(nomeClick).Visible = 'off';
                        end
                    end
                end
                Mapa.Plot.Obstaculo(nomeClick).Visible = 'on';
%                 Mapa.Plot.Obstaculo(nomeClick) =...
%                     plot(Vertices(nomeClick,2),Vertices(nomeClick,3),...
%                     'Marker','x',...
%                     'MarkerEdgeColor','k',...
%                     'LineWidth', 1,...
%                     'MarkerSize',50);
                Nomes(clickPoint(1,2),clickPoint(1,1)) = 0;
                Vertices(nomeClick,:) = [0 0 0];
            elseif false
                nomeClick = MapaAnt(clickPoint(1,2),clickPoint(1,1));
                Mapa.Plot.Vertice(nomeClick).Visible = 'on';
                Mapa.Plot.Texto(nomeClick).Visible = 'on';
%                 Mapa.Plot.Custo(nomeClick).Visible = 'on';
                Mapa.Plot.Obstaculo(nomeClick).Visible = 'off';
                Nomes(clickPoint(1,2),clickPoint(1,1)) = nomeClick;
                Vertices(nomeClick,:) = VerticesAnt(nomeClick,:);
                [sucClick,posSucClick] = mapaSucessor(Vertices(nomeClick),Nomes,Vertices);
                for i=1:size(sucClick,1)
                    if posSucClick{i,2} == 'V'
                        if posSucClick{i,1} == -1
                            Mapa.Plot.Aresta.Vertical(sucClick(i,1)).Visible = 'on';
                        else
                            Mapa.Plot.Aresta.Vertical(nomeClick).Visible = 'on';
                        end
                    elseif posSucClick{i,2} == 'H'
                        if posSucClick{i,1} == -1
                            Mapa.Plot.Aresta.Horizontal(sucClick(i,1)).Visible = 'on';
                        else
                            Mapa.Plot.Aresta.Horizontal(nomeClick).Visible = 'on';
                        end
                    end
                end
%                 disp(Nomes)
            end
            if sum(nomeClick == Caminho) ~= 0
                % Algoritmo de busca
                Satu = Vertices(Caminho(cc),:);
                temp_cc = Caminho(cc);
                Sg = Vertices(Sg(1),:);
                try
                    Caminho_temp = Caminho;
                    [Caminho,FECHADO] = LPAstar(Satu,Sg,Nomes,Vertices);
    %                 disp(FECHADO)
                    disp(Caminho)
                catch
                    error('CAMINHO INEXISTENTE')
                end
                cc = find(temp_cc == Caminho);
                clickPoint = [];
            end
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
%         h(1) = plot(Dados(:,1), Dados(:,2),'--r','LineWidth',2);
        h(2) = plot(Dados(:,13), Dados(:,14),'-k','LineWidth',2);
        h(3) = plot([P.pPos.Xd(1) Vertices(Caminho(cc+1),2)],...
            [P.pPos.Xd(2) Vertices(Caminho(cc+1),3)],'-g','LineWidth',1);
        for i = cc+1:(size(Caminho,2)-1)        
            h(i+3) = plot([Vertices(Caminho(i),2) Vertices(Caminho(i+1),2)],...
                [Vertices(Caminho(i),3) Vertices(Caminho(i+1),3)],'-g','LineWidth',1);
        end
        
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