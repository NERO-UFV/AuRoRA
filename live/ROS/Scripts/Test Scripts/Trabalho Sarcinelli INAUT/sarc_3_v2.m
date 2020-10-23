%% Comandos Iniciais

clear all; close all; clc;

plt = 0;
a = 1;

% Plotagem em tempo real
T = 0.01;
T_plot = 1;
tmax = 10;

%% Parâmetros iniciais

% Ganhos Dinâmicos
Ku = diag([4.72 6.23 2.65 2.38]);

Kv = diag([0.28 0.53 2.58 1.52]);

Ks = diag([1.00 1.00 1.00 1.00]);

Ky = diag([3.10 3.10 3.10 3.10]);

Kd = 100*diag([1.00 1.00 1.00 1.00]);

GAMA = diag([1  1  1  1  1  1  1  1]);
THETA = [1;  1;  1;  1;  1;  1;  1;  1];
% THETA = [0.08;  0.11;  0.13;  0.11;  0.15;  1.14;  0.52;  0.28];
% THETA_est = [0;  0;  0;  0;  0;  0;  0;  0];
THETA_est = [1;  1;  1;  1;  1;  1;  1;  1];

dXr_max = diag([1.5 0.9375 0.2 0.2094]);

% UAV 1
% dX_max = diag([1.60 1.10 .25 0.25]);

% UAV 2
dX_max = diag([3.20 2.20 1.50 1.00]);

% UAV 3
% dX_max = diag([3.00; 2.00; 1.00; .25]);

Kc = dX_max - dXr_max;

X = [0; 0; 0; 0];
dX = [0; 0; 0; 0];
ddX = [0; 0; 0; 0];

Xb = [0; 0; 0; 0];
dXb = [0; 0; 0; 0];
ddXb = [0; 0; 0; 0];

Ub = [0; 0; 0; 0];
Uc_ant = [0; 0; 0; 0];
H = [];
H_THETA_est = [];

fig(1)= plot3(0,0,0,'ko','LineWidth',5);
axis([-6 6 -8 8 0 2]);
xlabel('Eixo x [m]','FontSize',12,'FontWeight','bold');
ylabel('Eixo y [m]','FontSize',12,'FontWeight','bold');
zlabel('Eixo z [m]','FontSize',12,'FontWeight','bold');
hold on;
grid on;
fig(2) = plot3(0,0,0,'r--','LineWidth',2);
fig(3) = plot3(0,0,0,'b','LineWidth',2);
legend([fig(2),fig(3)],'X_{d}','X','Location','northwest');
drawnow;

pause(2);

t = tic;
ta = tic;
tplot = tic;

%% Laço de Simulação

while toc(t) < tmax
    if toc(ta) > T
        ta = tic;
        a = a*(-1);
        
        % Referência de Trajetória
        p = 3.75;
        w = 0.4;
        
        Xr = [p*sin(w*toc(t))+0.1;
            1.25*p*cos(0.5*w*toc(t))-0.2;
            1+0.5*sin(w*toc(t));
            pi/6*sin(w*toc(t))];
        
        dXr = [w*p*cos(w*toc(t));
            -0.5*w*1.25*p*sin(0.5*w*toc(t));
            w*0.5*cos(w*toc(t));
            w*pi/6*cos(w*toc(t))];
        
        ddXr = [-w*w*p*sin(w*toc(t));
            -(0.5*w)^2*1.25*p*cos(0.5*w*toc(t));
            -w*w*0.5*sin(w*toc(t));
            -w*w*pi/6*sin(w*toc(t))];
        
        
        F = [ cos(Xb(4))   -sin(Xb(4))   0    0; ...
            sin(Xb(4))      cos(Xb(4))   0    0;
            0                 0        1    0;
            0                 0        0    1];
        
        % Modelo Dinâmico
        ddX = F*Ku*Ub - Kv*dX; % Global
        dX = dX + ddX*T;
        X = X + dX*T;
        
        ddXb = Ku*Ub - Kv*dXb; % Drone
        dXb = dXb + ddXb*T;
        Xb = Xb + dXb*T;
        
        Xtil = Xr - X; % Global
        
        % Controlador Cinemático
        Uc = F\(dXr + Kc*tanh(Ky*Xtil));
        dUc = (Uc - Uc_ant)/T;
        Uc_ant = Uc;
        
        Ub = Uc;
                
        % Controlador Adaptativo
        g11 = dUc(1) + Kd(1)*(Uc(1)-dXb(1));
        g15 = dXb(1);
        g22 = dUc(2) + Kd(2)*(Uc(2)-dXb(2));
        g26 = dXb(2);
        g33 = dUc(3) + Kd(3)*(Uc(3)-dXb(3));
        g37 = dXb(3);
        g44 = dUc(4) + Kd(4)*(Uc(4)-dXb(4));
        g48 = dXb(4);
        
        G = [ g11   0   0   0   g15   0   0   0 ;...
            0   g22   0   0   0   g26   0   0  ;...
            0   0   g33   0   0   0   g37   0  ;...
            0   0   0   g44   0   0   0   g48 ];
        
        D = diag([THETA_est(1) THETA_est(2) THETA_est(3) THETA_est(4)]);
        SIGMA = dUc + Kd*(Uc - dXb);
        NETA = diag([dXb(1) dXb(2) dXb(3) dXb(4)])*...
            [THETA_est(5); THETA_est(6); THETA_est(7); THETA_est(8)];
        
        dTHETA_est = GAMA\G'*(Uc - dXb);
        THETA_est = THETA_est + dTHETA_est*T;
        THETA_error = THETA_est - THETA;
        
        
        %         Ud = D*SIGMA + NETA + G*THETA_error;
        Ud = G*THETA_est;
        %         Ub = Uc;
        
        H = [H; [Xr' dXr' X' dX' Xtil' Ub' toc(t) a]];
        %         H_THETA_est = [H_THETA_est; THETA_est'];
        
        if toc(tplot) > T_plot
            tplot = tic;
            try %#ok<TRYNC>
                delete (fig);
            end
            fig(1)= plot3(X(1),X(2),X(3),'ko','LineWidth',5);
            fig(2) = plot3(H(:,1),H(:,2),H(:,3),'r--','LineWidth',2);
            fig(3) = plot3(H(:,9),H(:,10),H(:,11),'b','LineWidth',2);
            legend([fig(2),fig(3)],'X_{d}','X','Location','northwest');
            drawnow;
        end
    end
end

%% Plotagem dos Resultados

if plt == 1
    %% Erros lineares em x,y,z
    figure(2);
    plot(H(:,25),(H(:,1)-H(:,9)),'k','LineWidth',2);
    hold on;
    grid on;
    plot(H(:,25),(H(:,2)-H(:,10)),'r','LineWidth',2);
    plot(H(:,25),(H(:,3)-H(:,11)),'b','LineWidth',2);
    legend('x_{error}','y_{error}','z_{error}');
    axis([0 tmax -1 5]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Erros lineares em x,y,z','FontSize',12,'FontWeight','bold');
    
    %% Erro angular psi
    figure(3);
    plot(H(:,25),(H(:,4)-H(:,12)),'k','LineWidth',2);
    hold on;
    grid on;
    legend('psi_{error}');
    axis([0 tmax -1 1]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Erro angular psi','FontSize',12,'FontWeight','bold');
    
    %% Velocidade linear dx
    figure(4);
    plot(H(:,25),H(:,5),'r','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,13),'k','LineWidth',2);
    plot(H(:,25),dX_max(1,1)*ones(length(H),1),'--b','LineWidth',2);
    plot(H(:,25),-dX_max(1,1)*ones(length(H),1),'--b','LineWidth',2);
    grid on;
    legend('dXr','dX','dX_{max}');
    axis([0 tmax -4 4]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Velocidades no eixo X','FontSize',12,'FontWeight','bold');
    
    %% Velocidade linear dy
    figure(5);
    plot(H(:,25),H(:,6),'r','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,14),'k','LineWidth',2);
    plot(H(:,25),dX_max(2,2)*ones(length(H),1),'--b','LineWidth',2);
    plot(H(:,25),-dX_max(2,2)*ones(length(H),1),'--b','LineWidth',2);
    grid on;
    legend('dYr','dY','dY_{max}');
    axis([0 tmax -4 4]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Velocidades no eixo Y','FontSize',12,'FontWeight','bold');
    
    %% Velocidade linear dz
    figure(6);
    plot(H(:,25),H(:,7),'r','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,15),'k','LineWidth',2);
    plot(H(:,25),dX_max(3,3)*ones(length(H),1),'--b','LineWidth',2);
    plot(H(:,25),-dX_max(3,3)*ones(length(H),1),'--b','LineWidth',2);
    grid on;
    legend('dZr','dZ','dZ_{max}');
    axis([0 tmax -4 4]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Velocidade no eixo Z','FontSize',12,'FontWeight','bold');
    
    %% Velocidade angular
    figure(7);
    plot(H(:,25),H(:,8),'r','LineWidth',2);
    hold on;
    plot(H(:,25),H(:,16),'k','LineWidth',2);
    plot(H(:,25),dX_max(4,4)*ones(length(H),1),'--b','LineWidth',2);
    plot(H(:,25),-dX_max(4,4)*ones(length(H),1),'--b','LineWidth',2);
    grid on;
    legend('dpsir','dpsi','dpsi_{max}');
    axis([0 tmax -1 1]);
    xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
    ylabel('Velocidade angular','FontSize',12,'FontWeight','bold');
    
end


