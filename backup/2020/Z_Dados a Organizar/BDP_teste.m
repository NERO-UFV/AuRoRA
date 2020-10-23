%% Inicialização

close all
clear
clc

try
    fclose(instrfindall);
catch
end

% Search root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;
%OPT.Initialize('192.168.0.1');
% Robot initialization
P = Pioneer3DX;
Bola = Pioneer3DX;
P.pPar.a = 0.005;
Bola.pPar.a = 0.005;

% open serial port
s = serial('COM3');
fopen(s)

%% detect rigid body ID from optitrack
idP = getID(OPT,P,1);         % pioneer ID on optitrack
idP2 = getID(OPT,Bola,2);         % drone ID on optitrack

% initial positions
rb = OPT.RigidBody;         % read optitrack data
Bola = getOptData(rb(idP2),Bola);  % get ardrone data
P = getOptData(rb(idP),P);  % get pioneer data
P.pPar.Ts = 1/60;

%% Parâmetros
% thetad  = deg2rad(90);        % angulo desejado
erroPMax = (.03);%1cm
erroMax = deg2rad(20); % erro angular máximo
%%
data = [];
vel  = 0;
vmin = 45;
tp   = tic;
t    = tic;
vl   = 116;
vr   = 116;
%% Simulation loop
while toc(t)<4
    
    if toc(tp)>1/60
        tp = tic;
        %         %  Get current rigid body information from optitrack
                rb = OPT.RigidBody;
        %         Bola = getOptData(rb(idP2),Bola);  % get ardrone data
                P = getOptData(rb(idP),P);  % get pioneer data
        %
        %
        %         thetad  = atan2((Bola.pPos.X(2)-P.pPos.X(2)),(Bola.pPos.X(1)-P.pPos.X(1)));
        %         erroA = (P.pPos.Xc(6) - thetad);
        %         erroP = sqrt((Bola.pPos.X(2)-P.pPos.X(2))^2+(Bola.pPos.X(1)-P.pPos.X(1))^2);
        %
        %         %     if abs(erro) > erroMax && (erro<0)
        %         if abs(erroA) > erroMax
        %
        %             % orientação
        %             vel = vmin +(abs(P.pPos.X(6) - thetad))*10;
        %
        %             pacote=(['B' 'D' 22  round(vel) 128+round(vel) 116 116 116 116 'P']);
        %             fprintf(s,pacote);
        %             pause(.1)
        %             pacote=(['B' 'D' 22 16 16 16 16 16 16  'P']);
        %             fprintf(s,pacote);
        %             pause(.1)
        %
        %         elseif abs(erroP) > erroPMax
        %            % posição
        
        pacote=(['B' 'D' 22  vl vr 116 116 116 116 'P']);
        fprintf(s,pacote);
        %             pause(.1)
        %             pacote=(['B' 'D' 22 vmin vmin 16 16 16 16  'P']);
        %             fprintf(s,pacote);
        %             pause(.08)
        
        
        
    end
    %     if abs(erro) > erroMax && (erro>0)
    %
    %         vel = vmin +(abs(P.pPos.X(6) - thetad))*10;
    %
    % %         pacote=(['B' 'D' 22  round(vel+128) 16 116 116 116 116 'P']);
    % pacote=(['B' 'D' 22 vmin 16 116 116 116 116 'P']);
    %
    % fprintf(s,pacote);
    %         pause(0.1)
    %          pacote=(['B' 'D' 22  16 16 16 16 16 16  'P']);
    %         fprintf(s,pacote);
    %         pause(0.08)
    %     end
    
    %         if abs(erro) <= erroMax
    %             fprintf(s,['B' 'D' 22 16 16 16 16 16 16 'P']);
    %         end
    
    % save data: [robot positions(12) (left velocity) (right velocity) (current time)]
    data = [data [P.pPos.X ; vl ; vr ; toc(t)]];
    
    
%     disp('Angulo')
%     (P.pPos.X(6))
%     
%     disp('PWM');
%     vel
%     
%     disp('Erro');
%     erroA
%     disp('Erro');
%     erroP
%     disp('thetad');
%     thetad

% Draw robot
if toc(tp) > 0.1
    
    P.mCADdel
    Bola.mCADdel
    
    tp = tic;
    P.mCADplot(0.2,'k');
    Bola.mCADplot(0.2,'b');
    axis([-1 1 -1 1])
    drawnow
    grid on
end


end
% save('C:\Users\NERO_2\Dropbox\AuRoRA 2018\DataFiles\Log_bdp\linearPWM80_3','data');
% close serial port
fclose(s);

plot(data(end,:),data(7,:))
ylabel('Velocidade x [m/s]')
grid on
figure;
plot(data(end,:),data(8,:))
ylabel('Velocidade y')
grid on

figure;
plot(data(end,:),data(12,:))
ylabel('Velocidade angular')
grid on

