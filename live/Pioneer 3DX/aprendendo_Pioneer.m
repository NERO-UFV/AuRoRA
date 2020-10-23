clear; close all; clc;
% Close all the open connections
try
    fclose(instrfindall);
catch
end

%% Look for root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

% Criando uma variável para representar o Robô
P = Pioneer3DX;
% Coectar com o Robô real ou com MobileSim
P.rConnect

%% Initial Position
% Xo = input('Digite a posição inicial do robô ([x y z psi]): ');
Xo = [0 0 0 0];

P.rSetPose(Xo);         % define pose do robô

%%

t = tic;
t_inc = tic;

Tmax = 15;
Tinc = .100;

u = [];
ud = [];

w = [];
wd = [];

time = [];

figure();
subplot(211);
p1 = plot(toc(t),P.pSC.Ud(1));
hold on;
grid on;
p2 = plot(toc(t),P.pSC.U(1));
axis([0 Tmax -2 2]);
xlabel('Time [s]');
ylabel('Linear Velocity [m/s]');
lg1 = legend('u_{d}','u');

subplot(212);
p3 = plot(toc(t),P.pSC.Ud(2));
hold on;
grid on;
p4 = plot(toc(t),P.pSC.U(2));
axis([0 Tmax -2 2]);
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');
lg2 = legend('w_{d}','w');

while toc(t) < Tmax
    
    if toc(t_inc) > Tinc
        t_inc = tic;
        
        P.rGetSensorData;
        
        if toc(t) < Tmax/2
            P.pSC.Ud = [0 ; .5];
        else
            P.pSC.Ud = [0 ; 0];
        end
               
        ud = [ud ; P.pSC.Ud(1)];
        u = [u ; P.pSC.U(1)];
        
        wd = [wd ; P.pSC.Ud(2)];
        w = [w ; P.pSC.U(2)];
        
        time = [time ; toc(t)];
        
        try
        catch ERRRO
            delete (p1);
            delete (p2);
            delete (p3);
            delete (p4);
            delete (lg1);
            delete (lg2);
        end
        
        subplot(211);
        p1 = plot(time,ud,'r--');
        p2 = plot(time,u,'b-');
        lg1 = legend('u_{d}','u');
        
        subplot(212);
        p3 = plot(time,wd,'r--');
        p4 = plot(time,w,'b-');    
        lg2 = legend('w_{d}','w');
        drawnow;
        
        P.rSendControlSignals;
        
    end
end
