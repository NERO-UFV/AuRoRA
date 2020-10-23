clear all
close all
clc

%% Propriedades do grafo quadrado
Tamanho = 8;
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
        end
        cont = cont + 1;
    end
end

%% Criação do grafo
% G = (V,a), V = (a,a')
f = figure(1);
hold on
grid on
axis([0 Tamanho+2 0 Tamanho+2])
axis equal
axis ij
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

%% Simulação para tirar obstáculo
tmax = 5;
ta = 0.1;
tclock = 0.5;
clock_temp = 0;
ts = tic;
tc = tic;
pause
set(f,'WindowButtonDownFcn',@clickFunctionfloor)

t = tic;
while toc(t) < tmax
    % Função
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
            clickPoint = [];
        end
        drawnow
        ts = tic;
    end
    
    % Imprime o tempo decorrido
    if toc(tc) > tclock
%         clc
%         fprintf('Tempo atual: %2.1f/%2.1f', clock_temp, tmax)
        clock_temp = clock_temp + tclock;
        tc = tic;
    end
end
if round(clock_temp) == tmax
%     clc
%     fprintf('Tempo atual: %2.1f/%2.1f\n', clock_temp, tmax)
end