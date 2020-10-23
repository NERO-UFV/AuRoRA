clear all
close all
clc

try
    fclose(instrfindall);
catch
end
%% CRIANDO O GRAFO
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% Propriedades do grafo quadrado
Dist = 1;
Tamanho = 5;
% Deslocamento = Dist;
Deslocamento = Dist + Tamanho*Dist/2;
Desl = [Deslocamento - .5;
        Deslocamento - .2];
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
%         if Obstaculo(j,i) == 0
            Nomes(j,i) = cont;
%         end
        cont = cont + 1;
    end
end

Adjacencia = zeros(Tamanho^2);
Obstacle = zeros(Tamanho^2);
for j = 1:Tamanho
    for i = 1:Tamanho
        if j+1 <= Tamanho && Nomes(i,j+1) ~= 0
            Adjacencia(Nomes(i,j),Nomes(i,j+1)) = 1;
        end
        if j-1 >= 1 && Nomes(i,j-1) ~= 0
            Adjacencia(Nomes(i,j),Nomes(i,j-1)) = 1;
        end
        if i+1 <= Tamanho && Nomes(i+1,j) ~= 0
            Adjacencia(Nomes(i,j),Nomes(i+1,j)) = 1;
        end
        if i-1 >= 1 && Nomes(i-1,j) ~= 0
            Adjacencia(Nomes(i,j),Nomes(i-1,j)) = 1;
        end
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

Sinit = 8;
Sg = 22;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% Criação do grafo
% G = (V,a), V = (a,a')
f = figure(1);
hold on
grid on
axis([0-Desl(1) (Tamanho+2)*Dist-Desl(1)...
    0-Desl(2) (Tamanho+2)*Dist-Desl(2)])
axis equal
% axis ij
for j = 1:Tamanho
    for i = 1:Tamanho
        Vertices(Nomes(j,i),:) = [Nomes(j,i) (i+.5)*Dist-Desl(1) (j+.5)*Dist-Desl(2)];
    end
end
% Obstacle
Obstacle(1,2) = 1;
Obstacle(2,7) = 1;
Obstacle(4,9) = 1;
Obstacle(6,11) = 1;
Obstacle(7,8) = 1;
Obstacle(8,13) = 1;
Obstacle(8,9) = 1;
Obstacle(4,9) = 1;
Obstacle(9,10) = 1;
Obstacle(12,17) = 1;
Obstacle(14,19) = 1;
Obstacle(16,17) = 1;
Obstacle(17,22) = 1;
Obstacle(19,24) = 1;
Obstacle(19,20) = 1;
Obstacle(22,23) = 1;


Adjacencia = Adjacencia - (Obstacle + Obstacle');
Obstacle = Obstacle + Obstacle';

cont = 1;
for j = 1:Tamanho+1
    for i = 1:Tamanho+1
        if (i == 1 || i == Tamanho+1) && j <= Tamanho
            plot([i*Dist-Desl(1) i*Dist-Desl(1)],...
                 [j*Dist-Desl(2) (j+1)*Dist-Desl(2)],...
                 '-k');
        end
        if (j == 1 || j == Tamanho+1) && i <= Tamanho
            plot([i*Dist-Desl(1) (i+1)*Dist-Desl(1)],...
                 [j*Dist-Desl(2) j*Dist-Desl(2)],...
                 '-k');
        end
%         Mapa.Plot.Parede(Nomes(j,i)) =
        if i <= Tamanho && j <= Tamanho
            for k = find(Obstacle(Nomes(j,i),:)==1)
                if k > cont
                    if Vertices(Nomes(j,i),2) ~= Vertices(k,2)
                        Mapa.Plot.Parede(Nomes(j,i),k) = plot([(Vertices(Nomes(j,i),2)+Vertices(k,2))/2 ...
                                                               (Vertices(Nomes(j,i),2)+Vertices(k,2))/2],...
                                                               [Vertices(Nomes(j,i),3)-.5*Dist Vertices(Nomes(j,i),3)+.5*Dist],...
                                                               '-k');
                    else
                        Mapa.Plot.Parede(Nomes(j,i),k) = plot([Vertices(Nomes(j,i),2)-.5*Dist Vertices(Nomes(j,i),2)+.5*Dist],...
                                                              [(Vertices(Nomes(j,i),3)+Vertices(k,3))/2 ...
                                                              (Vertices(Nomes(j,i),3)+Vertices(k,3))/2],...
                                                              '-k');
                    end
                end
            end
            if true
                for k = find(Adjacencia(Nomes(j,i),:)==1)
                    if k > cont
                        Mapa.Plot.Arestas(Nomes(j,i),k) = plot([Vertices(Nomes(j,i),2) Vertices(k,2)],...
                                                               [Vertices(Nomes(j,i),3) Vertices(k,3)],...
                                                               '--b');
                    end
                end
                Mapa.Plot.Vertice(Nomes(j,i)) = plot((i+.5)*Dist-Desl(1), (j+.5)*Dist-Desl(2),...
                                                     'ob','MarkerFaceColor','w');
                Mapa.Plot.Text(Nomes(j,i)) = text((i+.25)*Dist-Desl(1), (j+.25)*Dist-Desl(2),...
                                                  num2str(Nomes(j,i)),...
                                                  'HorizontalAlignment','center',...
                                                  'VerticalAlignment','middle',...
                                                  'FontWeight','bold');
                Mapa.Plot.Custo(Nomes(j,i)) = text((i+.75)*Dist-Desl(1), (j+.75)*Dist-Desl(2),...
                                                   '\infty',...
                                                   'HorizontalAlignment','center',...
                                                   'VerticalAlignment','middle');
            end
            cont = cont + 1;
        end
    end
end
%%
Mapa.Plot.Vertice(Sinit).MarkerEdgeColor = 'k';
Mapa.Plot.Vertice(Sinit).MarkerFaceColor = 'k';
Mapa.Plot.Vertice(Sg).MarkerEdgeColor = 'r';
Mapa.Plot.Vertice(Sg).MarkerFaceColor = 'r';

%% ALGORITMO DE BUSCA
Sinit = Vertices(Sinit,:);
Sg = Vertices(Sg,:);
disp(Sg)
try
%     [Caminho,FECHADO] = LPAstar_Adj(Sinit,Sg,Nomes,Vertices,Adjacencia);
    Caminho = DijkstraAdj(Sinit,Sg,Nomes,Vertices,Adjacencia);
    clc
    disp(Caminho)
catch
    error('CAMINHO INEXISTENTE')
end
% disp(Caminho)
% P.rSetPose([Sinit(2) Sinit(3) 0 0]);

plot([Vertices(Caminho(1:end-1),2) Vertices(Caminho(2:end),2)],...
     [Vertices(Caminho(1:end-1),3) Vertices(Caminho(2:end),3)],...
     '-r')
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
    
P = Pioneer3DX(1); % Pioneer3DX Simulado
P.pPar.a = 0;
P.pPar.alpha = 0;
pgains = 10*[0.1 0.1 1];

P.rSetPose([Sinit(2) Sinit(3) 0 -pi/2]);
                    P.rGetSensorData;
Mapa.Plot.Custo(Sinit(1)).String = num2str(0);
P.mCADplot(1,'r');

pause
t_transicao = 2;
ta = tic;
dt = tic;

open(1,:) = [Sinit 0 0];
closed = [Sinit 0 0];

while size(open,1) ~= 0
    n = open(1,:);
    open = open(2:end,:);
    if n(1) == Sg(1)
        break
    end
    suc = mapaSucessorAdj(n,Adjacencia,Vertices);
    for i = 1:size(suc,1)
        c = [suc(i,:) n(1)];
        t = tic;
        if sum(c(:,1) == closed(:,1)) == 0
            Mapa.Plot.Custo(c(1)).String = num2str(n(1,end) + 1);
            open(end + 1,:) = [c n(1,end) + 1];
            closed(end + 1,:) = [c n(1,end) + 1];
            
            missao = [];
            missao(1) = closed(end-1,1);
            missao(2) = c(1);
            missao_temp = missao;
            while Adjacencia(missao_temp(1),missao_temp(2)) ~= 1
                missao_temp(1) = closed(closed(:,1)==missao_temp(1),4);
                missao_temp(2) = closed(closed(:,1)==missao_temp(2),4);
                if missao_temp(1) == missao_temp(2)
                    missao = [missao(1:end/2) missao_temp(1) missao((end/2+1):end)];
                    break
                else
                    missao = [missao(1:end/2) missao_temp missao((end/2+1):end)];
                end
            end
            mm = 1;
%             disp(missao)    
            t = tic;
            while mm < size(missao,2)
                if toc(ta) > 0.1
                    ta = tic;
                    
                    P.pPos.Xda = P.pPos.Xd;
                    P.pPos.Xd(1:2) = [Vertices(missao(mm),2)+(Vertices(missao(mm + 1),2)-Vertices(missao(mm),2))*toc(t)/t_transicao;
                        Vertices(missao(mm),3)+(Vertices(missao(mm + 1),3)-Vertices(missao(mm),3))*toc(t)/t_transicao];
                    
                    P.pPos.Xd(7:8) = [(Vertices(missao(mm + 1),2)-Vertices(missao(mm),2))/t_transicao;
                                      (Vertices(missao(mm + 1),3)-Vertices(missao(mm),3))/t_transicao];
                    
                    P.rGetSensorData;
                    P = fKinematicControllerExtended(P,pgains);
                    P.rSendControlSignals;
                    
                    P.mCADdel
                    P.mCADplot(1,'r')
                    
                    drawnow
                end
                if toc(t) > t_transicao
                    t = tic;
                    mm = mm + 1;
                end
            end
        end
        pos = find(c(:,1) == closed(:,1));
        if sum(c(:,1) == closed(:,1)) ~= 0 && closed(pos,end) > n(1,end) + 1
            closed(pos,end) = n(1,end) + 1;
        end
        open = sortrows(open,5);
    end
end

