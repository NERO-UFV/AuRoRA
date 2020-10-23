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
    2 15 2 15 1 3];

% Load initialization
L = Load;
L.pPos.X(1:3) = [0 0 0];
L.pPos.X(4:6) = [0 0 0];
L.pPar.m = 0.1;

% Cables
C{1} = Cable;
C{2} = Cable;

C{1}.pPar.l = 1.0;
C{2}.pPar.l = 1.0;

OptData = CreateOptDataFile(A{1},A{2},L,'DoubleTransportation','Log_LoadTransportation');
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%%
rb = OPT.RigidBody;
A{1} = getOptData(rb(2),A{1});
A{2} = getOptData(rb(1),A{2});
L = getOptData(rb(3),L);

A{1}.pPos.Xa(1:6) = A{1}.pPos.X(1:6);
A{2}.pPos.Xa(1:6) = A{2}.pPos.X(1:6);
L.pPos.Xa(1:6) = L.pPos.X(1:6);
disp(' ')
disp(['Posição Drone 1:  ' num2str(round(A{1}.pPos.X(1),2)) ' / ' num2str(round(A{1}.pPos.X(2),2)) ' / ' num2str(round(A{1}.pPos.X(3),2))])
disp(['Posição Drone 2:  ' num2str(round(A{2}.pPos.X(1),2)) ' / ' num2str(round(A{2}.pPos.X(2),2)) ' / ' num2str(round(A{2}.pPos.X(3),2))])
disp(['Posição Carga:  '   num2str(round(L.pPos.X(1),2)) ' / ' num2str(round(L.pPos.X(2),2)) ' / ' num2str(round(L.pPos.X(3),2))])

%%
% A{1}.pPar.ip = '192.168.1.1';
A{1}.pPar.ip = '192.168.1.30';
A{1}.rConnect;

A{2}.pPar.LocalPortControl = 5558;
A{2}.pPar.LocalPortState = 5552;
% A{2}.pPar.ip = '192.168.1.30';
A{2}.pPar.ip = '192.168.1.1';
A{2}.rConnect;

%% Initialize classes
% Connect Joystick
J = JoyControl;

%% Desired Values
Alt = 1.1;

Xd1 = [-0.5   0   Alt   0  30.0;
];

Xd2 = [+0.5   0   Alt   0  30.0;
];


% Xd1 = [+0.0   0.6   Alt   0  15.0;
%        +0.5   0.6   Alt   0  25.0;
%        -1.   0.6   Alt   0  35.0;
%        -1.   0.0   Alt   0  45.0;
%         0.   0.0   Alt   0  60.0;
% ];
%
% Xd2 = [+0.0   -0.6   Alt   0  15.0;
%        +0.5   -0.6   Alt   0  25.0;
%        -1.0   -0.6   Alt   0  35.0;
%         0.0    -0.0   Alt   0  45.0;
%         1.0    -0.0   Alt   0  60.0;
% ];

% Xd1 = [-0.6   0.0   Alt   0  20.0;
%        -0.6   0.6   Alt   0  25.0;
%        -0.6   -0.7   Alt   0  30.0;
% ];
%
% Xd2 = [+0.6   0.0   Alt   0  20.0;
%        +0.6   0.6   Alt   0  25.0;
%        +0.6   -0.7   Alt   0  30.0;
% ];


% Xd1 = [-0.6    0.0   Alt   0  15.0;
%        -0.6   -0.9   Alt   0  20.0;
%        -1.5    0.7   Alt   0  25.0;
%         0.5    0.7   Alt   0  35.0;
%         0.5   -0.6   Alt   0  35.0;
% ];
%
% Xd2 = [+0.6    0.0   Alt   0  15.0;
%        +0.6   -0.9   Alt   0  20.0;
%        -0.5    0.7   Alt   0  25.0;
%         1.5    0.7   Alt   0  35.0;
%         1.5   -0.6   Alt   0  35.0;
% ];

% Frente trás
% Xd1 = [-0.6   0.0   Alt   0  5.0;
%         0.5   0.0   Alt   0  10.0;
%        -1.5   0.0   Alt   0  20.0;
%     ];
% 
% Xd2 = [+0.6   0.0   Alt   0  5.0;
%         1.5   0.0   Alt   0  10.0;
%        -0.5   0.0   Alt   0  20.0;
%     ];

% tsim = Xd1(end,end);    % Simulation time [s]

a = .8;
b = .7;
c = .1;
w = .05;


tsim = 40;
%%
% rTakeOffMultiples(A{1},A{2});
for k = 1:5
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
pause(7)
disp(' ')
disp('Released')
%% Time variables

cont = 1;     % counter
angulos = [];
XX = []; YY = []; CC = []; % position data
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

        if toc(t) > 10
        tt = toc(t);
        A{2}.pPos.Xd(1) = a*sin(2*pi*w*tt) +.5;            % x
        A{2}.pPos.Xd(7) = a*2*pi*w*cos(2*pi*w*tt);     % dx
        A{2}.pPos.Xd(2) = b*sin(2*pi*2*w*tt);          % y
        A{2}.pPos.Xd(8) = b*2*pi*2*w*cos(2*pi*2*w*tt); % dy
        A{2}.pPos.Xd(3) = 1.2;                         % z
        A{2}.pPos.Xd(9) = 0;
        
        A{1}.pPos.Xd(1) = A{2}.pPos.Xd(1) -1;            % x
        A{1}.pPos.Xd(7) = a*2*pi*w*cos(2*pi*w*tt);     % dx
        A{1}.pPos.Xd(2) = b*sin(2*pi*2*w*tt);          % y
        A{1}.pPos.Xd(8) = b*2*pi*2*w*cos(2*pi*2*w*tt); % dy
        A{1}.pPos.Xd(3) = 1.2;                         % z
        A{1}.pPos.Xd(9) = 0;
        
%         if toc(t) > Xd1(cont,5)
%             cont = cont + 1;
%         end
% 
%         A{1}.pPos.Xd(1) = Xd1(cont,1);   % x
%         A{1}.pPos.Xd(2) = Xd1(cont,2);   % y
%         A{1}.pPos.Xd(3) = Xd1(cont,3);   % z
%         A{1}.pPos.Xd(6) = 0; % Psi
%         
%         A{2}.pPos.Xd(1) = Xd2(cont,1);   % x
%         A{2}.pPos.Xd(2) = Xd2(cont,2);   % y
%         A{2}.pPos.Xd(3) = Xd2(cont,3);   % z
%         A{2}.pPos.Xd(6) = 0; % Psi
        
        else
            A{2}.pPos.Xd(1) = .5;
            A{2}.pPos.Xd(3) = 1.3;
            A{1}.pPos.Xd(1) = -.5;
            A{1}.pPos.Xd(3) = 1.3;
        end
        
        
%% Positioning
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
            td = tic; kopt = kopt+1;
        end
        
        if rb(2).isTracked
            dta = toc(td);
            if dta > 2/30
                dta = 1/30;
            end
            A{1} = getOptData(rb(2),A{1});
            td = tic; kopt = kopt+1;
        end
        
        if rb(3).isTracked
            dta = toc(td);
            if dta > 2/30
                dta = 1/30;
            end
            L = getOptData(rb(3),L);
            td = tic; kopt = kopt+1;
        end
        %                 % Filtro de sinal
        L.pPos.X = L.pPos.Xa*0.3 + L.pPos.X*0.7;
        %
        %% % % Alpha, Beta, Tração e Comprimento do cabo
        
        for nv = 1:2
            C{nv}.pPos.Xa = C{nv}.pPos.X;
            C{nv}.pPos.X(1) = atan(norm(norm(A{1}.pPos.X(1)- L.pPos.X(1)))/norm(A{1}.pPos.X(2)- L.pPos.X(2)));
            C{nv}.pPos.X(2) = abs(atan2(A{nv}.pPos.X(1)-L.pPos.X(1),A{nv}.pPos.X(3)-L.pPos.X(3)));
            C{nv}.pPos.X(4) = norm(A{nv}.pPos.X(1:3)-L.pPos.X( 1:3));
        end
        
        if L.pPos.X(3) < .02
            C{1}.pPos.X(3) = 0;
            C{2}.pPos.X(3) = 0;
        else
            C{1}.pPos.X(3) = L.pPar.m*L.pPar.g*sin(C{2}.pPos.X(2))/(sin(C{1}.pPos.X(2)+C{2}.pPos.X(2)));
            C{2}.pPos.X(3) = L.pPar.m*L.pPar.g*sin(C{1}.pPos.X(2))/(sin(C{1}.pPos.X(2)+C{2}.pPos.X(2)));
        end
        
        Cabo1 = [C{1}.pPos.X(3)*sin(C{1}.pPos.X(2))*cos(C{1}.pPos.X(1));
                 -C{1}.pPos.X(3)*sin(C{1}.pPos.X(2))*sin(C{1}.pPos.X(1));
                 C{1}.pPos.X(3)*cos(C{1}.pPos.X(2))];
        
        Cabo2 = [C{2}.pPos.X(3)*sin(C{2}.pPos.X(2))*cos(C{2}.pPos.X(1));
                 -C{2}.pPos.X(3)*sin(C{2}.pPos.X(2))*sin(C{2}.pPos.X(1));
                 C{2}.pPos.X(3)*cos(C{2}.pPos.X(2))];
             
        Cabo1(3) = 0;
        Cabo2(3) = 0;
        
%         Repulsion = sRepulsionField(A{1},A{2});
%         A{1}.pPar.D(4:6) =  -Repulsion';
%         A{2}.pPar.D(4:6) =  Repulsion';    

%         A{1}.pPar.D(4:6) =  Cabo1;
%         A{2}.pPar.D(4:6) =  Cabo2; 
% 
        A{1}.pPar.D(1:3) = Cabo1;
        A{2}.pPar.D(1:3) = Cabo2;
 
       % disp([Cabo1 Cabo2])
        
        %%
        A{1} = cUnderActuatedControllerMexido(A{1},gains);
        A{2} = cUnderActuatedControllerMexido(A{2},gains);
        
        A{1} = J.mControl(A{1}); % joystick command (priority)
        A{2} = J.mControl(A{2});
        %% Save data ------------------------------------------
        XX = [XX [A{1}.pPos.Xd; A{1}.pPos.X; A{1}.pSC.Ud; A{1}.pSC.U; toc(t)]];
        YY = [YY [A{2}.pPos.Xd; A{2}.pPos.X; A{2}.pSC.Ud; A{2}.pSC.U; toc(t)]];
        CC = [CC [L.pPos.X; toc(t)]];
        SaveOptData(OptData,A{1},A{2},L,toc(t));
        %% ----------------------------------------------------------------
        A{1}.rCommand;
        A{2}.rCommand;
        
    end
end


%% Close Data Files

CloseOptDataFile(OptData);
A{1}.rLand;
A{2}.rLand;

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
% % Trajectory 2D
% figure(2)
% plot(XX([1,13],:)',XX([2,14],:)'); grid;
% plot(YY([1,13],:)',YY([2,14],:)'); grid;
%
%
% % Trajectory 3D
figure(3)
plot3(XX([1,13],:)',XX([2,14],:)',XX([3,15],:)'); hold on
plot3(YY([1,13],:)',YY([2,14],:)',YY([3,15],:)'); hold on
plot3(CC(1,:)',CC(2,:)',CC(3,:)'); grid
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

%%
% Control Signals
figure(6)
subplot(411),plot(XX(end,:),XX([25 29],:)'); grid;
axis([0 XX(end,end) -1 1])
subplot(412),plot(XX(end,:),XX([26 30],:)'); grid;
axis( [0 XX(end,end) -1 1])
subplot(413),plot(XX(end,:),XX([27 31],:)'); grid;
axis( [0 XX(end,end) -1 1])
subplot(414),plot(XX(end,:),XX([28 32],:)'); grid;
axis([0 XX(end,end) -1 1])

figure(7)
subplot(411),plot(YY(end,:),YY([25 29],:)'); grid;
axis([0 YY(end,end) -1 1])
subplot(412),plot(YY(end,:),YY([26 30],:)'); grid;
axis( [0 YY(end,end) -1 1])
subplot(413),plot(YY(end,:),YY([27 31],:)'); grid;
axis( [0 YY(end,end) -1 1])
subplot(414),plot(YY(end,:),YY([28 32],:)'); grid;
axis([0 YY(end,end) -1 1])


disp('!!! Fim !!!')
 
%%


try A{1}.rLand; end; try A{2}.rLand; end; 







