%%%% Simulação Catamaran %%%%

clearvars, close, clc

addpath(genpath(pwd))

%% Representação do Barco

boat = Catamara;

% Setting Position
% boat.pPos.X([1 2]) = [10,50];
boat.pPesoVirtual = -0.25;
% boat.pPos.X([1 2 3]) = [50,50,0.25-2]; % CADload/1000
boat.pPos.X([1 2 3]) = [100, 101, 0]; % [25 60 0]; % CADload/100

% Pontos de Contato 
% Pontos pertencentes ao catamara antigo
% (?) A fazer: 
%   - Implementar metodo automático para selecionar pontos
%   - Detectar pontos frente/tras/esquerda/direita automaticamente

p1 = [-8.7668         0    0.4510]';
p2 = [-8.4516   -6.9723   -0.2276]';
p3 = [-8.4516    6.9723   -0.2276]';
p4 = [-2.0698   -8.8348    0.7670]';
p5 = [-2.0698    8.8348    0.7670]';
p6 = [ 3.1767   -9.4437    0.8877]';
p7 = [ 3.1767    9.4437    0.8877]';
p8 = [ 9.06195        0   -1.1997]';
originalPdC = [p1 p2 p3 p4 p5 p6 p7 p8];

originalfrente = [-8.7668   -8.4516   -8.4516;
                0   -6.9723    6.9723;
           0.4510   -0.2276   -0.2276];
     
originaltras = [ 3.1767    3.1767    9.0619;
        -9.4437    9.4437         0;
         0.8877    0.8877   -1.1997];
    
originaldireita = [-8.4516  -2.0698 3.1767;
            6.9723   8.8348 9.4437;
           -0.2276   0.7670 0.8877];

originalesquerda = [ -8.4516    3.1767   -2.0698;  
            -6.9723    -9.4437   -8.8348;
            -0.2276    0.8877     0.7670];
        
%% Definindo a Figura que irá rodar a simulação

f1 = figure('Name','Simulação: Catamarã','NumberTitle','off');
f1.Position = [9 2 930 682];

figure(f1);

ax = gca;
ax.FontSize = 12;
xlabel({'$$x$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$y$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
zlabel({'$$z$$ [m]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
view(3)
view(45,30)
grid on
hold on
grid minor
light;

% axis([-1 1 -1 1 0 1])
axis tight

set(gca,'Box','on');
set(gca, 'Color', 'none')

% First Plot
hold on

boat.mCADplot();
hold off

pause

%% Inicializando superficie:

n=200; % limites (X,Y) em cm(?)
H=zeros(n,n)-2; % gera area 

% hold on
% mesh(H);
% hold off
axis([1 n 1 n -15 25]);


% Energia cinetica e potencial
Ekin=[]; Epot=[];

oldH=H; % ?
newH=H; % ?
i = 2:n-1; % Vetor de 2 a 99 com passo 1;
j = 2:n-1; % Vetor de 2 a 99 com passo 1;
hold on
h=surf(newH); % ?
hold off


% Estilizando superficie
lighting phong;
material shiny;
lightangle(-45,30)
light('Position',[-10 20 10]);
% axis([1 n 1 n -15 5]);
% axis([40 60 40 60 -15 5]);

% Inicializando variaveis (add_Catamara)

% globOH = 0; % ?

% Colocar Catamara na superfície:
[H,globOH] = add_Catamara(boat.pPos.X(1),boat.pPos.X(2),n,H,boat.pPesoVirtual);

%t = tic; % temporizador de interação onda-catamara

while true
    % Reseta velocidades angulares desejadas em rolagem e arfagem
    boat.pSC.Ud(1:2) = [0 0]; 
    
    [aceleracoes,newH]=Wave(n,i,j,0.05,12,0.2,H,oldH,0,0,0);
                          % n,i,j,dt,c,k,H,oldH,fix,cont,connect
    
    %boat.pPos.X(3) = newH(boat.pPos.X(1),boat.pPos.X(2));
    boat.pPos.X(3) = mean(newH((-10 + boat.pPos.X(1)):(10 + boat.pPos.X(1)),...
                               (-10 + boat.pPos.X(2)):(10 + boat.pPos.X(2))),'all');
    
    % Atualiza posicao dos pontos de contato 
    PdC(1:3,:) = boat.pPos.X(1:3)+originalPdC(1:3,:);
    frente(1:3,:) = boat.pPos.X(1:3)+originalfrente(1:3,:);
    tras(1:3,:) = boat.pPos.X(1:3)+originaltras(1:3,:);
    esquerda(1:3,:) = boat.pPos.X(1:3)+originalesquerda(1:3,:);
    direita(1:3,:) = boat.pPos.X(1:3)+originaldireita(1:3,:);
    
    %% Para cada ponto de contato do barco, calcula diferença entre 
    % a altura do ponto de contato e da agua. Encontra primeiro indice
    % da menor diferença (i,j) na matriz da superficie da agua e usa 
    % como ponto na agua de contato com o barco.
    % 
    % (?) A fazer:
    %   - Ineficiente, nao pega os pontos corretos, IMPORTANTE CORRIGIR
    %   - Problema: como selecionar os pontos da agua mais próximos dos PdC
    %   do barco?
    
    indices_PdC = [];
    for ponto = 1:size(PdC,2) 
        dif = abs(newH-PdC(3,ponto));
        indice_linear = find(dif == min(min(dif)),1);
        [indices_PdC(1,ponto) indices_PdC(2,ponto)] = find(newH(indice_linear)== newH,1,'first');
    end
    
    %% Mostra Pontos de Contato (pontos da agua que tocam no barco)
    %
    % (?) A fazer:
    %   - Metodo de selecionar os pontos esta ruim, pontos selecionados
    %   estao distantes do barco. IMPORTANTE CORRIGIR
    %   - Melhorar implementacao de visualizacao dos pontos
 
    hold on
    try
        delete(pontosAgua)
%         delete(pontosBarco)
    end
    
    % PdC da agua (calculados)
    pontosAgua = plot3(indices_PdC(1,:),indices_PdC(2,:),newH(indices_PdC),'o','LineWidth', 12,'MarkerSize', 10);
    % PdC do barco (manualmente selecionados)
%     pontosBarco = plot3(PdC(1,:),PdC(2,:),PdC(3,:),'o','LineWidth', 12,'MarkerSize', 10);

    
    %% Direcionando o barco
    % Similar ao algoritmo dos pontos de contato. 
    %
    % (?) A fazer:
    %   - Ao inves de utilizar a maior aceleracao em algum ponto de
    %   contato, calcular resultante de todas (?)
    %   - Pode ser melhorado.
    
    aceleracoes = aceleracoes(indices_PdC);
    resultante = find(aceleracoes == max(aceleracoes),1,'first');
%     resultante = find(aceleracoes > 0, 1,'last');
    
    if sum(sum(PdC(resultante))) == 0
        boat.pSC.Ud(1:2) = [0 0]; 
    else
        if find(frente== PdC(resultante))
            %ir para tras
            boat.pSC.Ud(1) = 0.5;
        
        elseif find(PdC(resultante)==tras)
            %ir para frente
            boat.pSC.Ud(1) = -0.5;

        elseif find(PdC(resultante)==esquerda)
            %ir para direta
            boat.pSC.Ud(2) = 0.5;

        elseif find(PdC(resultante)==direita)
            %ir para esquerda
            boat.pSC.Ud(2) = -0.5;
        end
    end
         
    sDynamicModel(boat);
    
    
    % Teoricamente codigo que altera a agua a partir dos pontos de contato.
    % Nao funcionou como esperado, repensar estrategia.
%     if toc(t)>1
%        [H,globOH] = add_Catamara(PdC(1,:),PdC(2,:),n,H,-0.0001);
%        t= tic;
%     end
     
    
    boat.mCADplot();
        
    set(h,'zdata',newH);

    oldH=H;
    H=newH;
    
    pause(0.05);
    
end


 
 
 


 
 
 
