close all
clear
clc

try
    fclose(instrfindall);
end
% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


P = Pioneer3DX;
P.rConnect;
P.rSetPose([2.5 2.5 0 0]);

% Parâmetros de Navegação
DOBS_TAN  = 0.85;
DOBS_A    = 0.65;
DOBS_TRAN = 1;
FLAG_DESVIO = false;

% Parâmetros Sonar
maxSonarDist = 5.0; % [m]

Medidas = ones(1,361)*maxSonarDist;

% Criar grade de ocupação
og = robotics.OccupancyGrid(25,25,10);
og.GridLocationInWorld = [-5 -5];


% Parâmetros Desvio Tangencial
sTE = TangentialEscape;
dref = 0.50;
k1 = [0.25 0;0 0.35];
k2 = [1 0;0 1];
xq = []; yq = []; grid = []; curva = []; poly = [];

prog = true;
pos_dest = [18 18;2.5 2.5];
pos_index = 1;
%%
%%% codigo para armazenar dados de navegação %%%
%POS_X = cell(1,size(pos_dest,1));
%M_SENSOR = cell(1,size(pos_dest,1));
%POS_A_ESTRELA = cell(1,size(pos_dest,1));
while pos_index <= size(pos_dest,1)
    Xdest = pos_dest(pos_index,1); Ydest = pos_dest(pos_index,2);
    P.pPos.Xd([1 2 6]) = [Xdest Ydest 0]';
    d = 0;
    ts = tic;
    P.pPos.Xtil(1:2) = [inf,inf];
    mov_aestrela = [];
    grid1 = [];
    grid1.og = og;
    grid1.mod = floor(og.occupancyMatrix+0.5);
    [xq,yq,mov_aestrela,tem_caminho,grid,curva] = ProcuraCaminho(world2grid(og,[P.pPos.X(1) P.pPos.X(2)]),world2grid(og,[P.pPos.Xd(1) P.pPos.Xd(2)]),grid1);
    if tem_caminho
        dobs = DOBS_A;
        rota = ConstruirCaminhoSeguimento([xq yq]);
        t_rota = size(rota,2);
    else
        dobs = DOBS_TAN;
    end
    
    while norm([Xdest-P.pPos.X(1) Ydest-P.pPos.X(2)]) > 0.15
        P.rGetSensorData;
        SonarData = P.rGetSonarData;
        insertRay(og,[P.pPos.X(1) P.pPos.X(2) P.pPos.X(6)],SonarData(2,:),SonarData(1,:),maxSonarDist);
        if tem_caminho
            if norm(P.pPos.X(1:2)-[rota(1,t_rota) rota(2,t_rota)]') < 0.1
                tem_caminho = false;
                dobs = DOBS_TAN;
                P.pPos.Xd(1:2) = [Xdest Ydest];
                fprintf('parar de seguir caminho\n');
            else
                dtemp = min(SonarData.medidas(2,:));
                if dtemp <= DOBS_A
                    FLAG_DESVIO = true;
                    fprintf('iniciando desvio apesar do A* \n');
                end
                if FLAG_DESVIO && dtemp < DOBS_TRAN
                    P.pPos.Xd(1:2) = [Xdest Ydest];
                    fprintf('continuar desviando...\n');
                else
                    FLAG_DESVIO = false;
                    P.pPos.Xd = SeguirCaminho(rota,P.pPos.X);
                end
            end
        end
        
        % Desvio Tangencial
        P = sTE.sFindVirtualGoal(P,SonarData);
        P = fControladorCinematico(P);
        
        if toc(ts) > 0.1
            P.rSendControlSignals;
            ts = tic;
        end
        
        show(og)
        drawnow;
    end
    P.pSC.Ud = [0; 0];
    P.rSendControlSignals;
    pos_index=pos_index+1;
    %%% codigo para salvar dados de navegação %%%
    %POS_X{pos_index} = rastroRobo;
    %rastroRobo = [];
    %if tem_caminho
    %    POS_A_ESTRELA{pos_index} = [xq yq];
    %end
    %M_SENSOR{pos_index} = rastroSensor;
end

P.pSC.Ud = [0; 0];
P.rSendControlSignals;