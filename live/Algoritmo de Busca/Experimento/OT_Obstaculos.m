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

%% Carregando Classe

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

%% Criando o Obstáculo
% Obstáculos
NumObs = 6;
Obstaculos = zeros(1,NumObs);
for i = 1:NumObs
    A{i} = ArDrone(i + 10);
end

%% Propriedades do grafo quadrado
Dist = 0.6;
Tamanho = 5;
Deslocamento = Dist + Tamanho*Dist/2;
Chance = 0;

%% Detectando Obstaculo
contObs = 0;
for i = 1:NumObs
    idA{i} = getID(OPT,A{i}) + contObs;            % pioneer ID on optitrack
    rb = OPT.RigidBody;            % read optitrack data
    A{i} = getOptData(rb(idA{i}),A{i});    % get pioneer data
%     disp(A{i}.pPos.X(1:3))
    contObs = contObs + 1;
end
%% Pegando valores
Obstaculos = zeros(1,NumObs);
T = tic;
while true
    if toc(T) > 1
        T = tic;
        contObs = 0;
        for i = 1:NumObs
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
            % Detectando Obstaculo
            idA{i} = getID(OPT,A{i}) + contObs;            % pioneer ID on optitrack
            contObs = contObs + 1;
            if Obstaculos(1,i) ~= 1
                rb = OPT.RigidBody;            % read optitrack data
                A{i} = getOptData(rb(idA{i}),A{i});    % get pioneer data
            
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
            % Avaliando o Obstaculo
                DetectandoObstaculo(i)
                disp(obsPoint)
                if ~isempty(obsPoint)
                    Obstaculos(1,i) = 1;
                end
            end
        end
    end
    if sum(Obstaculos == 0) == 0
        break
    end
end
