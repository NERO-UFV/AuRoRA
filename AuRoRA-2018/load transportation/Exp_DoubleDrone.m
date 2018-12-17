%% Simulação de Voo Cooperativo com carga em 3 dimensões

% Inicialização
close all; clear; clc

try
    fclose(instrfindall);
catch
end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

% Drones initialization
A{1} = ArDrone;
A{2} = ArDrone;

A{1}.pPar.uSat(1:2) = A{1}.pPar.uSat(1:2)*2.0;
A{1}.pPar.uSat(3)   = A{1}.pPar.uSat(3)*1.0;
A{1}.pPar.uSat(4)   = A{1}.pPar.uSat(4)*1.0;
A{2}.pPar.uSat      = A{1}.pPar.uSat;

A{1}.pPar.D = [0 0 0 0 0 0]';
A{2}.pPar.D = [0 0 0 0 0 0]';

gains = [1.2 2.6 1.2 2.6 2.2 15;
         2 15 2 15 1 2];

OptData = CreateOptDataFile(A{1},A{2},'DoubleTransportation','Log_LoadTransportation');
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%%
rb = OPT.RigidBody;
A{1} = getOptData(rb(2),A{1});
A{2} = getOptData(rb(1),A{2});

A{1}.pPos.Xa(1:6) = A{1}.pPos.X(1:6);
A{2}.pPos.Xa(1:6) = A{2}.pPos.X(1:6);
disp(' ')
disp(['Posição Drone 1:  ' num2str(round(A{1}.pPos.X(1),1)) ' / ' num2str(round(A{1}.pPos.X(2),1)) ' / ' num2str(round(A{1}.pPos.X(3),1))])
disp(['Posição Drone 2:  ' num2str(round(A{2}.pPos.X(1),1)) ' / ' num2str(round(A{2}.pPos.X(2),1)) ' / ' num2str(round(A{2}.pPos.X(3),1))])

%%
A{1}.pPar.ip = '192.168.1.1';
% A{1}.pPar.ip = '192.168.1.30';
A{1}.rConnect;

A{2}.pPar.LocalPortControl = 5558;
A{2}.pPar.LocalPortState = 5552;
A{2}.pPar.ip = '192.168.1.30';
% A{2}.pPar.ip = '192.168.1.1';
A{2}.rConnect;

%% Initialize classes
% Connect Joystick
J = JoyControl;

%%
Xd1 = [+0.0   0.6   1.1   0  20.0;
       +0.5   0.6   1.1   0  30.0;
];

Xd2 = [+0.0   -0.6   1.1   0  20.0;
       -0.5   -0.6   1.1   0  30.0;
];

%%
% rTakeOffMultiples(A{1},A{2});
for k = 1:10
    %%
    % Set the reference for horizontal plane
    fprintf(A{1}.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'FTRIM', A{1}.pCom.SequenceNumber, ''));
    A{1}.pCom.SequenceNumber = A{1}.pCom.SequenceNumber + 1;    % Send Takeoff command

    A{1}.rGetStatusRawData;
    if A{1}.pCom.cStatus(1)==1
        Sig1 = sprintf('AT*REF=%d,290718464\r',A{1}.pCom.SequenceNumber);%(00010001010101000000001100000000)
        disp('Emergency State')
    else 
       Sig1 = sprintf('AT*REF=%d,290718208\r',A{1}.pCom.SequenceNumber);%(00010001010101000000001000000000)
    end
    fprintf(A{1}.pCom.controlChannel, Sig1);
    A{1}.pCom.SequenceNumber = A{1}.pCom.SequenceNumber + 1;
    %%
    % Set the reference for horizontal plane
    fprintf(A{2}.pCom.controlChannel, ...
        sprintf( 'AT*%s=%i,%s\r', 'FTRIM', A{2}.pCom.SequenceNumber, ''));
    A{2}.pCom.SequenceNumber = A{2}.pCom.SequenceNumber + 1;    % Send Takeoff command

        A{2}.rGetStatusRawData;
    if A{2}.pCom.cStatus(1)==1
        Sig2 = sprintf('AT*REF=%d,290718464\r',A{2}.pCom.SequenceNumber);%(00010001010101000000001100000000)
        disp('Emergency State')
    else 
       Sig2 = sprintf('AT*REF=%d,290718208\r',A{2}.pCom.SequenceNumber);%(00010001010101000000001000000000)
    end
    fprintf(A{2}.pCom.controlChannel, Sig2);
    A{2}.pCom.SequenceNumber = A{2}.pCom.SequenceNumber + 1;
end
disp('Holding')
pause(5)
disp(' ')
disp('Released')
%% Time variables
time = 15;  % target hover time  [s]
tsim = 30;    % Simulation time [s]
cont = 1;     % counter
angulos = [];
XX = [];      % position data
YY = [];      % position data
tout = 100;   % maximum simulation duration [s]
tc = tic;     % drone frequency
tp = tic;     % graph refresh rate
tt = tic;     % trajectory time
t = tic;      % simulation current time
td = tic;
ta = [];
dt = 1/30;
kopt = 0;   % counter
kdrone = 0; % counter
%% Simulation loop
while toc(t) < tsim
    if toc(tc) > A{1}.pPar.Ts
        ta = [ta toc(tc)];
        tc = tic;
        
        %% Desired Position
        if toc(t) > Xd1(cont,5)
            cont = cont + 1;
        end
        
        A{1}.pPos.Xd(1) = Xd1(cont,1);   % x
        A{1}.pPos.Xd(2) = Xd1(cont,2);   % y
        A{1}.pPos.Xd(3) = Xd1(cont,3);   % z
        A{1}.pPos.Xd(6) = 0; % Psi
        
        A{2}.pPos.Xd(1) = Xd2(cont,1);   % x
        A{2}.pPos.Xd(2) = Xd2(cont,2);   % y
        A{2}.pPos.Xd(3) = Xd2(cont,3);   % z
        A{2}.pPos.Xd(6) = 0; % Psi
        
        %%
        % ----------------------------------------------------------
        % Controlador
        % ----------------------------------------------------------
        %  Get current rigid body information from optitrack
        rb = OPT.RigidBody;
        
        if rb(1).isTracked
            dta = toc(td);
            if dta > 2/30
                dta = 1/30;
            end
            A{2} = getOptData(rb(1),A{2});
            td = tic;
            kopt = kopt+1;
        end
        
        if rb(2).isTracked
            dta = toc(td);
            if dta > 2/30
                dta = 1/30;
            end
            A{1} = getOptData(rb(2),A{1});
            td = tic;
            kopt = kopt+1;
        end

        %%
        A{1} = cUnderActuatedControllerMexido(A{1},gains);
        A{2} = cUnderActuatedControllerMexido(A{2},gains);

        A{1} = J.mControl(A{1}); % joystick command (priority)
        pause(.0001);
        A{2} = J.mControl(A{2});
%         % Save data ------------------------------------------
        XX = [XX [A{1}.pPos.Xd; A{1}.pPos.X; A{1}.pSC.Ud; A{1}.pSC.U; toc(t)]];
        YY = [YY [A{2}.pPos.Xd; A{2}.pPos.X; A{2}.pSC.Ud; A{2}.pSC.U; toc(t)]];
        %  SaveOptData(OptData,A,L ,toc(t));
        % ----------------------------------------------------------------
        A{1}.rCommand;
        pause(.0001);
        A{2}.rCommand;
    end
end
%% Close Data Files
A{1}.rLand;
A{2}.rLand;
%%
CloseOptDataFile(OptData);


%% Plot Results
% Roll and pitch angles
figure(1)
subplot(311),plot(XX(end,:),XX([4 16],:)'*180/pi); grid;
% subplot(311),plot(YY(end,:),YY([4 16],:)'*180/pi); grid;
subplot(312),plot(XX(end,:),XX([5 17],:)'*180/pi); grid;
% subplot(312),plot(YY(end,:),YY([5 17],:)'*180/pi); grid;
subplot(313),plot(XX(end,:),XX([6 18],:)'*180/pi); grid;
% subplot(313),plot(YY(end,:),YY([6 18],:)'*180/pi); grid;
legend('\psi_{Des}[^o]','\psi_{Atu}[^o]')

%%
% Trajectory 2D
figure(2)
plot(XX([1,13],:)',XX([2,14],:)'); grid;
plot(YY([1,13],:)',YY([2,14],:)'); grid;


% Trajectory 3D
figure(3)
plot3(XX([1,13],:)',XX([2,14],:)',XX([3,15],:)'); grid
plot3(YY([1,13],:)',YY([2,14],:)',YY([3,15],:)'); grid
%% X and Y
figure(4)
subplot(311),plot(XX(end,:),XX([1 13],:)'); grid;
subplot(312),plot(XX(end,:),XX([2 14],:)'); grid;
subplot(313),plot(XX(end,:),XX([3 15],:)'); grid;

legend('_{Des}','_{Atu}')

figure(5)
subplot(311),plot(YY(end,:),YY([1 13],:)'); grid;
subplot(312),plot(YY(end,:),YY([2 14],:)'); grid;
subplot(313),plot(YY(end,:),YY([3 15],:)'); grid;
legend('_{Desj}','_{Atu}')
%%
% velocities
figure(5)
subplot(311),plot(XX(end,:),XX([7 19],:)'); grid;
axis([0 XX(end,end) -1 1])
subplot(312),plot(XX(end,:),XX([8 20],:)'); grid;
axis( [0 XX(end,end) -1 1])
subplot(313),plot(XX(end,:),XX([9 21],:)'); grid;
legend('dz_{Des}','dz_{Atu}')
axis([0 XX(end,end) -1 1])

disp('!!! Fim !!!')

%%



try
A{1}.rLand;
end
try
A{2}.rLand;
end




