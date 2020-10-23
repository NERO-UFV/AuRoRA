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
        Deslocamento - .3];
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

try
    % Load Classes
    
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
%     % Inicializando o OptiTrack
    OPT = OptiTrack;    % Criando o OptiTrack
    OPT.Initialize;     % Iniciando o OptiTrack

    % Iniciando os Robôs
    B{1} = Bebop(1,'B1'); % Bebop 1
    B{2} = Bebop(2,'B2'); % Bebop 2
    A{3} = ArDrone(3); % Drone virtual
    A{4} = ArDrone(4); % Drone virtual
    
    DRONES = [1 1 0 0];    % Drones ativos
    
    P{1} = RPioneer(1,'P1'); % Pìoneer3DX Experimento
    P{2} = RPioneer(2,'P2'); % Pìoneer3DX Experimento
    P{3} = RPioneer(3,'P3'); % Pìoneer3DX Experimento
%     P{1} = Pioneer3DX(1); % Pioneer3DX Simulado
%     P{2} = Pioneer3DX(2); % Pioneer3DX Simulado
%     P{3} = Pioneer3DX(3); % Pioneer3DX Simulado
    for ii = 1:3
        P{ii}.rDisableMotors;
    end

    PIONEER = [1 1 0]; % Pioneer ativos
    
    % Pegando o ID dos corpos rigidos no OptiTrack
    idB{1} = getID(OPT,B{1},1);
    idB{2} = getID(OPT,B{2},2);
    idP{1} = 1;
    idP{2} = 2;
    idP{3} = 3;
    
    % Iniciando a formação triangular
    TF{1} = TriangularFormationBaju(1);
    TF{1}.pPar.R = {1 1 3; 'P' 'B' 'A'};
    
    TF{2} = TriangularFormationBaju(2);
    TF{2}.pPar.R = {2 2 4; 'P' 'B' 'A'};
    
    TF{3} = TriangularFormationBaju(3);
    TF{3}.pPar.R = {3 0 0; 'P' '-' '-'};
    
    FORMACAO = [1 1 0];
    
    % Joystick
    J = JoyControl;
    
    disp('################### Load Class Success #######################');
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end



