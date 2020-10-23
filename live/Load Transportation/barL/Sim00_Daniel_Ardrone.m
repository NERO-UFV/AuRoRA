clear all; close all; clc;

%% Modelo do Robô
A = ArDrone(1);

% A.pPar.A = diag([1,1,1,1]);
% A.pPar.B = diag([1,1,1,1]);
invA = inv(A.pPar.A);
invAB = inv(A.pPar.A)*A.pPar.B;

%% Constantes de Controle
Kd = diag([1 1 1 1]);
Kp = diag([1 1 1 1]);

%% Inicialização das variáveis
T_exp = 60;
t_exp = tic;
T_loop = 1/30;
t_loop = tic;
T_plot = 0.1;
t_plot = tic;
t = tic;
ti = tic;

fig = figure();
axis([-6 6 -6 6 0 6]);
xlabel('[x]','FontWeight','bold')
ylabel('[y]','FontWeight','bold')
zlabel('[z]','FontWeight','bold')
grid on
hold on

rX = 5;
rY = 5;
rZ = 2;
z0 = 3;
T = 15;
w = 2*pi/T;
plotdrone = plot3([A.pPos.X(1) A.pPos.X(1)],[A.pPos.X(2) A.pPos.X(2)],[A.pPos.X(3) A.pPos.X(3)],'r*');
plottrajetoria = plot3([A.pPos.Xd(1) A.pPos.Xd(1)],[A.pPos.Xd(2) A.pPos.Xd(2)],[A.pPos.Xd(3) A.pPos.Xd(3)],'b*');

data = [];

% Caminho
path.k = 1;
path.n = 100;
path.x = linspace(-2,2,path.n);
path.y = path.x.^3 - path.x;

A.pPos.X([1 2]) = [-1;-2];
[path.min,path.k] = min(sqrt((A.pPos.X(1) - path.x).^2 + (A.pPos.X(2) - path.y).^2) + (A.pPos.Xd(3) - A.pPos.X(3)).^2);

    if path.k == 1
        path.k = path.k + 1;
    end

A.pPos.Xd([1 2 3 6]) = [path.x(path.k);
    path.y(path.k);
    z0;
    0];

vmax = 1;
path.beta = atan2((path.y(path.k) - path.y(path.k-1)),(path.x(path.k) - path.x(path.k-1)));
path.dbeta = 0;
path.advance = tic;
path.on = 0;

while toc(t_exp) < T_exp
    if toc(t_loop) > T_loop
        t_atual = toc(t);
        t_loop = tic;
        
        %% Trajetória desejada
        %% Caminho
        
        path.min = min(sqrt((A.pPos.X(1) - path.x(path.k)).^2 + (A.pPos.X(2) - path.y(path.k)).^2) + (A.pPos.Xd(3) - A.pPos.X(3)).^2);
        
        
        %%
        if (path.min < 0.05 || path.on == 1 && toc(path.advance) > 0.2) && path.k < path.n
            path.advance = tic;
            path.on = 1;
            path.k = path.k + 1;
            disp(path.k)
            
            path.betaa = path.beta;
            path.beta = atan2((path.y(path.k) - path.y(path.k-1)),(path.x(path.k) - path.x(path.k-1)));
            path.dbeta = (path.beta - path.betaa);
        
            A.pPos.Xd([7 8 9 12]) = ...
            [(vmax/(1+200*abs(path.dbeta)))*cos(path.beta);
            (vmax/(1+200*abs(path.dbeta)))*sin(path.beta);
            0;
            0];
        
        end
        
        if path.k == 100
            A.pPos.Xd([7 8 9 12]) = [0;0;0;0];
        end
        

        
        A.pPos.Xd([1 2 3 6]) = [path.x(path.k);
            path.y(path.k);
            z0;
            path.beta];
                
           
        
   
        
        %% Data
        data = [data; A.pPos.X([1 2 3 6])' A.pPos.Xd([1 2 3 6])' A.pPos.Xd([7 8 9 12])' path.dbeta t_atual];
        
        %% Plot
        if toc(t_plot) > T_plot
            t_plot = tic;
            delete(plotdrone)
            delete(plottrajetoria)
            A.mCADplot;
            plotdrone_rastro = plot3([A.pPos.X(1) A.pPos.X(1)],[A.pPos.X(2) A.pPos.X(2)],[A.pPos.X(3) A.pPos.X(3)],'r.','MarkerSize',5);
            plotdrone = plot3([A.pPos.X(1) A.pPos.X(1)],[A.pPos.X(2) A.pPos.X(2)],[A.pPos.X(3) A.pPos.X(3)],'r*');
            plottrajetoria_rastro = plot3([A.pPos.Xd(1) A.pPos.Xd(1)],[A.pPos.Xd(2) A.pPos.Xd(2)],[A.pPos.Xd(3) A.pPos.Xd(3)],'b.','MarkerSize',5);
            plottrajetoria = plot3([A.pPos.Xd(1) A.pPos.Xd(1)],[A.pPos.Xd(2) A.pPos.Xd(2)],[A.pPos.Xd(3) A.pPos.Xd(3)],'b*');
            drawnow
        end
        
    end
end


