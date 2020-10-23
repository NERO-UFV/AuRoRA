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

%% Propriedades do grafo quadrado
Dist = 0.6;
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
        end
        cont = cont + 1;
    end
end

%% Selecionando a posição inicial e final
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

%% Criação do grafo
% G = (V,a), V = (a,a')
f = figure(1);
hold on
grid on
axis([0 (Tamanho+2)*Dist 0 (Tamanho+2)*Dist])
axis equal
% axis ij
for j = 1:Tamanho+1
    for i = 1:Tamanho+1
        if i < Tamanho+1
            plot([i*Dist (i+1)*Dist],[j*Dist j*Dist],...
                '-k','LineWidth', 0.8); % Horizontal
%                 'FaceAlpha',0.4,'EdgeAlpha',0.4); % Horizontal
        end
        if j < Tamanho+1
            plot([i*Dist i*Dist],[j*Dist (j+1)*Dist],...
                '-k','LineWidth', 0.8); % Vertical
%                 'FaceAlpha',0.4,'EdgeAlpha',0.4); % Vertical
        end
        if i < Tamanho+1 && j < Tamanho+1 && Obstaculo(j,i) == 0
            Vertices(Nomes(j,i),:) = [Nomes(j,i) (i+.5)*Dist (j+.5)*Dist];
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

%% Algoritmo de busca
Sinit = Vertices(Sinit,:);
Sg = Vertices(Sg,:);
disp(Sg)
try
    [Caminho,FECHADO] = LPAstar_2_E(Sinit,Sg,Nomes,Vertices);
    clc
    disp(Caminho)
catch
    error('CAMINHO INEXISTENTE')
end
% disp(Caminho)

%% Simulação para tirar obstáculo
% Iniciando as variaveis

pause
for i = 1:(size(Caminho,2)-1)        
    h(i) = plot([Vertices(Caminho(i),2) Vertices(Caminho(i+1),2)],...
        [Vertices(Caminho(i),3) Vertices(Caminho(i+1),3)],'-g','LineWidth',1);
    drawnow
end
set(f,'WindowButtonDownFcn',@clickFunctionfloorDist)

while true
    try
        nomeClick = Nomes(clickPoint(1,2),clickPoint(1,1));
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
        Nomes(clickPoint(1,2),clickPoint(1,1)) = 0;
        Vertices(nomeClick,:) = [0 0 0];
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
        
        try
            [Caminho,FECHADO] = LPAstar_2_E(Sinit,Sg,Nomes,Vertices,FECHADO);
            disp(Caminho)
            try
                delete(h)
            end
        catch
            error('CAMINHO INEXISTENTE')
        end
        clickPoint = [];
        for i = 1:(size(Caminho,2)-1)        
            h(i) = plot([Vertices(Caminho(i),2) Vertices(Caminho(i+1),2)],...
                [Vertices(Caminho(i),3) Vertices(Caminho(i+1),3)],'-g','LineWidth',1);
            drawnow
        end
    end
    drawnow
    ts = tic;
end