clear all
close all
clc

%% Propriedades do grafo quadrado
Tamanho = 5;
Chance = 0;

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
            cont = cont + 1;
        end
    end
end

%% Selecionando a posição inicial e final
Sinit = randi(Tamanho^2 - sum(sum(Obstaculo)));
Sg = Sinit;
while Sg == Sinit
    Sg = randi(Tamanho^2 - sum(sum(Obstaculo)));
end
Sinit = 12;
Sg = 5;

%% Criação do grafo
% G = (V,a), V = (a,a')
figure(1)
hold on
grid on
axis([0 Tamanho+2 0 Tamanho+2])
axis equal
axis ij
for j = 1:Tamanho+1
    for i = 1:Tamanho+1
        if i < Tamanho+1 && j < Tamanho+1 && Obstaculo(j,i) == 0
            Vertices(Nomes(j,i),:) = [Nomes(j,i) i+.5 j+.5];
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
% Usando Dijkstra
% Caminho = Dijkstra(Sinit,Sg,Nomes,Vertices);
% Usando A*
Caminho = Astar(Sinit,Sg,Nomes,Vertices);
% Caminho = [1 2 7 12 17 18 13 8 3 4 9 14 15];
% Caminho = [1 2 3 4 5 10 15];

%% Trajetória
tmax = 0.5;
ta = 0.01;
ts = tic;

% Inicializando a trajetória
ct = tmax;
Dados = [];
contD = 1;

pause
t = tic;
% Tempo real
for cc = 1:size(Caminho,2)-1
while toc(t) < tmax
    if toc(ts) > ta
        % Trajetória
        Xt = Vertices(Caminho(cc),2) +...
            toc(t)/tmax*(Vertices(Caminho(cc+1),2) -...
            Vertices(Caminho(cc),2));
        Yt = Vertices(Caminho(cc),3) +...
            toc(t)/tmax*(Vertices(Caminho(cc+1),3) -...
            Vertices(Caminho(cc),3));
        dXt = (Vertices(Caminho(cc+1),2) -...
            Vertices(Caminho(cc),2))/tmax;
        dYt = (Vertices(Caminho(cc+1),3) -...
            Vertices(Caminho(cc),3))/tmax;
%         plot(Xt,Yt,'.g','MarkerSize',15)
        ts = tic;
        Dados(:,contD) = [Xt; Yt; dXt; dYt];
        contD = contD + 1;
        
        plot(Dados(1,:),Dados(2,:),'-k','LineWidth',2)
        
        drawnow
    end
end
Mapa.Plot.Vertice(Caminho(cc+1)).MarkerEdgeColor = 'k';
Mapa.Plot.Vertice(Caminho(cc+1)).MarkerFaceColor = 'k';
t = tic;
end