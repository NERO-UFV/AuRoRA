clear all
close all
clc

%% Propriedades do mapa
Mapa.Tamanho = 25;                           % Tamanho
Mapa.Posicao.Estados = 0.5;                 % Posição dos estados
Mapa.Posicao.Arestas = 0;                   % Posição das arestas
Mapa.ChanceObstaculo = .25;                 % Chance de gerar obstáculo
Mapa.Valor.Estado = ones(Mapa.Tamanho);     % Define o valor de cada estado do mapa
Mapa.Valor.Aresta = ones(Mapa.Tamanho+1);   % Define o valor de cada aresta do mapa
Mapa.Estados = {};

%% Posição inicial e final
Pos.Inicial = randi(Mapa.Tamanho,1,2);
Pos.Final = randi(Mapa.Tamanho,1,2);

%% Construção do mapa
Chance = randi(100,Mapa.Tamanho)/100;
for i = 1:Mapa.Tamanho
    for j = 1:Mapa.Tamanho
        if Chance(i,j) < Mapa.ChanceObstaculo &&...
                i ~= Pos.Inicial(1) && i ~= Pos.Final(1) &&...
                i ~= Pos.Inicial(2) && i ~= Pos.Final(2)
            Mapa.Valor.Estado(i,j) = 0;
            Mapa.Estados{i,j} = NaN;
        else
            Mapa.Estados{i,j} = [i j];
        end
    end
end

%% Plot do mapa
% Iniciando as variaveis do plot
% Mapa.Plot.Estado = zeros(Mapa.Tamanho);
% Mapa.Plot.Aresta.Horizontal = zeros(Mapa.Tamanho,Mapa.Tamanho-1);
% Mapa.Plot.Aresta.Vertical = zeros(Mapa.Tamanho-1,Mapa.Tamanho);

% Iniciando o plot
figure(1)
hold on
grid on
axis([(min(Mapa.Posicao.Estados,Mapa.Posicao.Arestas)-1) (Mapa.Tamanho+1)...
    (min(Mapa.Posicao.Estados,Mapa.Posicao.Arestas)-1) (Mapa.Tamanho+1)])
axis equal
for i = 1:(Mapa.Tamanho+1)
    for j = 1:(Mapa.Tamanho+1)
        % Plot dos estados
        if i < Mapa.Tamanho+1 && j < Mapa.Tamanho+1
            % Obstáculo
            if Mapa.Valor.Estado(i,j) == 0
                Mapa.Plot.Estado(i,j) = plot(i-1+Mapa.Posicao.Estados,j-1+Mapa.Posicao.Estados,...
                    'Marker','x',...
                    'MarkerEdgeColor','k',...
                    'LineWidth', 2,...
                    'MarkerSize',25);
            end
            % Posição Inicial
            if i == Pos.Inicial(1) && j == Pos.Inicial(2)
                Mapa.Plot.Estado(i,j) = plot(i-1+Mapa.Posicao.Estados/2,j-1+Mapa.Posicao.Estados/2,...
                    'Marker','o',...
                    'MarkerEdgeColor','k',...
                    'MarkerFaceColor','k',...
                    'MarkerSize',5);
                text(i-1+Mapa.Posicao.Estados,j-1+Mapa.Posicao.Estados,'S',...
                    'HorizontalAlignment','center',...
                    'VerticalAlignment','middle',...
                    'FontSize',15,...
                    'FontWeight','bold')
            end
            % Posição Final
            if i == Pos.Final(1) && j == Pos.Final(2)
%                 Mapa.Plot.Estado(i,j) = plot(i-1+Mapa.Posicao.Estados,j-1+Mapa.Posicao.Estados,...
%                     'Marker','s',...
%                     'MarkerEdgeColor','r',...
%                     'LineWidth', 2,...
%                     'MarkerSize',10);
%                     'MarkerFaceColos','k',...
                text(i-1+Mapa.Posicao.Estados,j-1+Mapa.Posicao.Estados,'G',...
                    'HorizontalAlignment','center',...
                    'VerticalAlignment','middle',...
                    'FontSize',15,...
                    'FontWeight','bold')
            end
        end
        % Plot das arestas
        if i < Mapa.Tamanho+1
            Mapa.Plot.Aresta.Horizontal(i,j) = plot([i-1+Mapa.Posicao.Arestas i+Mapa.Posicao.Arestas],...
                [j-1+Mapa.Posicao.Arestas j-1+Mapa.Posicao.Arestas],'-k','LineWidth', 2);
        end
        if j < Mapa.Tamanho+1
            Mapa.Plot.Aresta.Vertical(i,j) = plot([i-1+Mapa.Posicao.Arestas i-1+Mapa.Posicao.Arestas],...
                [j-1+Mapa.Posicao.Arestas j+Mapa.Posicao.Arestas],'-k','LineWidth', 2);
        end
    end
end

%% Valor Heurístico do Estado
for i = 1:Mapa.Tamanho
    for j = 1:Mapa.Tamanho
        if Mapa.Valor.Estado(i,j) ~= 0
            Mapa.Valor.Heuristico(i,j) = distanciaManhattan([i j],Pos.Final);
            text(i-1+(Mapa.Posicao.Estados+1)/2,j-1+(Mapa.Posicao.Estados)/2,...
                num2str(Mapa.Valor.Heuristico(i,j)),...
                'HorizontalAlignment','center',...
                'VerticalAlignment','middle',...
                'FontSize',10)
        else
            Mapa.Valor.Heuristico(i,j) = inf;
        end
    end
end

Caminho = Dijkstra(Pos.Inicial,Pos.Final,Mapa);
% pause
% 
% for i = 1:Mapa.Tamanho
%     for j = 1:Mapa.Tamanho
%         Mapa.Plot.Estado(i,j).MarkerEdgeColor = 'b';
%         Mapa.Plot.Estado(i,j).MarkerFaceColor = 'b';
%         drawnow
%         t = tic;
%         while toc(t) < 1
%         end
%     end
% end