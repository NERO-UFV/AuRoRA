%% Testar modelo dinâmico do ArDrone em uma trajetória

% Inicialização
close all
clear
clc

try
    fclose(instrfindall);
catch
end

% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_Optitrack')
Arq = fopen(['trajetoria_espiral_drone_' NomeArq '.txt'],'w');
cd(PastaAtual)


%% Initialize classes
% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Connect Joystick
J = JoyControl;

% Robot initialization
A = ArDrone;
A.rConnect;
A.rTakeOff;
% A.rGetSensorCalibration;

%% Window definition
figure(1)
axis([-3 3 -3 3 0 3])
grid on
A.mCADplot
drawnow
pause(5)

%% Variables Initialization
a = 1;          % "raio" do circulo em x
b = 1;         % "raio" do circulo em y
c = 1;
% w = 0.05;
% nvoltas = 1;
% tsim = pi*nvoltas/(w*2);

angulos = [];
XX = [];             % position data
kopt = 0;   % counter
kdrone = 0; % counter

% Time variables
tsim = 60;    % Simulation duration [s]
tout = 100;     % maximum simulation duration [s]
tc = tic;      % drone frequency
tp = tic;      % graph refresh rate
tt = tic;      % trajectory time
t = tic;       % simulation current time
td = tic;
ta = [];
dt = 1/30;

%% Simulation loop
while toc(t) < tsim
    if toc(tc) > 1/30
        ta = [ta toc(tc)];
        tc = tic;
        
        % Trajetória desejada
        % espiral
        w = 5/tsim; % 0.02
        tt = toc(t);
        A.pPos.Xda = A.pPos.Xd;
        
        perc = (tsim-tt)/tsim;
        
        A.pPos.Xd(1) =  perc*a*cos(2*pi*w*tt) ;            % x
        A.pPos.Xd(7) = -perc*a*2*pi*w*sin(2*pi*w*tt) - a*cos(2*pi*w*tt)/tsim;     % dx
        A.pPos.Xd(2) = perc*b*sin(2*pi*w*tt);          % y
        A.pPos.Xd(8) = perc*b*2*pi*w*cos(2*pi*w*tt) - b*sin(2*pi*w*tt)/tsim; % dy
        A.pPos.Xd(3) = 0.75+c*tt/tsim;      % z
        A.pPos.Xd(9) = c/tsim;
        %         A.pPos.Xd(6) = atan2(A.pPos.Xd(8),A.pPos.Xd(7));
        %        if abs(A.pPos.Xd(6) - A.pPos.Xda(6)) > pi
        %             if A.pPos.Xda(6) < 0
        %                 A.pPos.Xda(6) =  2*pi + A.pPos.Xda(6);
        %             else
        %                 A.pPos.Xda(6) = -2*pi + A.pPos.Xda(6);
        %             end
        %        end
        %         A.pPos.Xd(12) = (A.pPos.Xd(6) - A.pPos.Xda(6))/dt;
        % ----------------------------------------------------------
        
        %  Get current rigid body information from optitrack
        rb = OPT.RigidBody;
        try
            if rb.isTracked
                dt = toc(td);
                if dt > 2/30
                    dt = 1/30;
                end
                A = getOptData(rb,A);
                A.pPos.X(7:9) = (A.pPos.X(1:3)-A.pPos.Xa(1:3))/dt;
                A.pPos.X(10:11) = (A.pPos.X(4:5)-A.pPos.Xa(4:5))/dt;
                if abs(A.pPos.X(6) - A.pPos.Xa(6)) > pi
                    if A.pPos.Xa(6) < 0
                        A.pPos.Xa(6) =  2*pi + A.pPos.Xa(6);
                    else
                        A.pPos.Xa(6) = -2*pi + A.pPos.Xa(6);
                    end
                end
                A.pPos.X(12) = (A.pPos.X(6)-A.pPos.Xa(6))/dt;
                
                A.pPos.Xa = A.pPos.X;
                td = tic;
                kopt = kopt+1;
            end
        catch
            kdrone = kdrone+1;
        end
        
        A.rGetAngles;
        A = cUnderActuatedController(A);
        A = J.mControl(A);           % joystick command (priority)
        
        % Save data (.txt file) ------------------------------------------
        % Control signal
        lim = (15*pi/180);
        % U = [phi theta dz dpsi]
        A.pSC.U = [A.pPos.X(4)/lim; A.pPos.X(5)/lim; A.pPos.X(9); A.pPos.X(12)/(100*pi/180)];
        % text file
        fprintf(Arq,'%6.6f\t',[A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' toc(t)]);
        fprintf(Arq,'\n\r');
        % plot variable
        XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; A.pSC.U; toc(t)]];
        % ----------------------------------------------------------------
        
        A.rSendControlSignals;
       
    end
    
    % Draw robot
    if toc(tp) > 0.3
        tp = tic;
        A.mCADplot; hold on
        plot3(XX(1,:),XX(2,:),XX(3,:),'k-','LineWidth',0.5);
        plot3(XX(13,:),XX(14,:),XX(15,:),'g-','LineWidth',1);
        drawnow
    end
    
    % Timeout
    if toc(t)>tout
        disp('Time out. Ending simulation...');
        break
    end
    
end

% Land drone
if A.pFlag.Connected == 1
    A.rLand;
    %     A.rDisconnect;
end

kopt
kdrone
% % Close txt file
fclose(Arq);

%% Plot results
%
% figure
% plot(ta)

% Roll and pitch angles
figure
subplot(311),plot(XX(end,:),XX([4 16],:)'*180/pi)
legend('\phi_{Des}[^o]','\phi_{Atu}[^o]')
grid
subplot(312),plot(XX(end,:),XX([5 17],:)'*180/pi)
legend('\theta_{Des}[^o]','\theta_{Atu}[^o]')
grid
subplot(313),plot(XX(end,:),XX([6 18],:)'*180/pi)
legend('\psi_{Des}[^o]','\psi_{Atu}[^o]')
grid

% Trajectory 2D
figure
% axis equal
plot(XX([1,13],:)',XX([2,14],:)')
% axis([-1.5 1.5 -1.5 1.5])


% Trajectory 3D
figure
% axis equal
plot3(XX([1,13],:)',XX([2,14],:)',XX([3,15],:)')
% axis([-1.5 1.5 -1.5 1.5])
grid on


% X and Y
figure
subplot(311),plot(XX(end,:),XX([1 13],:)')
legend('x_{Des}','x_{Atu}')
grid
subplot(312),plot(XX(end,:),XX([2 14],:)')
legend('y_{Des}','y_{Atu}')
grid
subplot(313),plot(XX(end,:),XX([3 15],:)')
legend('z_{Des}','dz_{Atu}')
grid

% dX and dY
figure
subplot(311),plot(XX(end,:),XX([7 19],:)')
legend('dx_{Des}','dx_{Atu}')
grid
subplot(312),plot(XX(end,:),XX([8 20],:)')
legend('dy_{Des}','dy_{Atu}')
grid
subplot(313),plot(XX(end,:),XX([9 21],:)')
legend('dz_{Des}','dz_{Atu}')
grid


