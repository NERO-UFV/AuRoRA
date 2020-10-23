%% Comandos Iniciais

clear all; close all; clc;

%% Parâmetros iniciais

% Ganhos Dinâmicos
Ku = diag([4.72 6.23 2.65 2.38]);

Kv = diag([0.28 0.53 2.58 1.52]);

Ks = diag([1.00 1.00 1.00 1.00]);

Ky = diag([3.10 3.10 3.10 3.10]);

dXr_max = diag([1.5 0.9375 0.2 0.2094]);

% UAV 1
% dX_max = diag([1.60 1.10 0.25 0.25]);

% UAV 2
% dX_max = diag([3.20 2.20 1.50 0.50]);

% UAV 3
dX_max = diag([3.00; 2.00; 1.00; .25]);

Kc = dX_max - dXr_max;

T = 0.075;
T_plot = 0.1;
tmax = 60;

X = [0; 0; 0; 0];
dX = [0; 0; 0; 0];
ddX = [0; 0; 0; 0];

Xb = [0; 0; 0; 0];
dXb = [0; 0; 0; 0];
ddXb = [0; 0; 0; 0];

U = [0; 0; 0; 0];
Uc_ant = [0; 0; 0; 0];
H = [];

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

% Cinemática Inversa

t = tic;
ta = tic;
tt = tic;
tt_ant = tic;
tplot = tic;

%% Laço de Simulação

while toc(t) < tmax
    tt = tic;
    if toc(ta) > T
        ta = tic;
        
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
        
        F = [ cos(Xb(4))   -sin(Xb(4))   0    0; ...
            sin(Xb(4))      cos(Xb(4))   0    0;
            0                 0        1    0;
            0                 0        0    1];
        
        ddX = F*Ku*U - Kv*dX;
        dX = dX + ddX*T;
        X = X + dX*T;
        
        ddXb = Ku*U - Kv*dXb;
        dXb = dXb + ddXb*T;
        Xb = Xb + dXb*T;
        
        Xtil = Xr - X;
        
        Uc = F\(dXr + Kc*tanh(Ky*Xtil));
        dUc = (F*Uc - Uc_ant)/T;
        Uc_ant = F*Uc;
        
        Sr = dX_max*tanh(F*Uc);
        Chr = F*Uc;
        Chr = diag([cosh(Chr(1)) cosh(Chr(2)) cosh(Chr(3)) cosh(Chr(4))]);
        dSr = dX_max/(Chr^(2))*dUc;
        
        S = dX_max*tanh(dX);
        
        Stil = dX_max*tanh((Sr - S));
        
        H = [H; [Xr' dXr' X' dX' Xtil' U' toc(t)]];
        
        Ud = (F*Ku)\(dSr+Ks*Stil) + (F*Ku)\Kv*Sr;
        
        U = Ud;
        
        
        if toc(tplot) > T_plot
            tplot = tic;
            try
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

% Velocidades lineares dx,dy,dz
figure(4);
plot(H(:,25),H(:,5),'r','LineWidth',2);
hold on;
plot(H(:,25),H(:,13),'k','LineWidth',2);
plot(H(:,25),dX_max(1,1)*ones(length(H),1),'--k','LineWidth',2);
plot(H(:,25),-dX_max(1,1)*ones(length(H),1),'--k','LineWidth',2);
grid on;
legend('dXr','dX','dX_{max}');
axis([0 tmax -4 4]);
xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
ylabel('Velocidades no eixo X','FontSize',12,'FontWeight','bold');
%%
figure(5);
plot(H(:,25),H(:,6),'r','LineWidth',2);
hold on;
plot(H(:,25),H(:,14),'k','LineWidth',2);
plot(H(:,25),dX_max(2,2)*ones(length(H),1),'--k','LineWidth',2);
plot(H(:,25),-dX_max(2,2)*ones(length(H),1),'--k','LineWidth',2);
grid on;
legend('dYr','dY','dY_{max}');
axis([0 tmax -4 4]);
xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
ylabel('Velocidades no eixo Y','FontSize',12,'FontWeight','bold');

%%
figure(6);
plot(H(:,25),H(:,7),'r','LineWidth',2);
hold on;
plot(H(:,25),H(:,15),'k','LineWidth',2);
plot(H(:,25),dX_max(3,3)*ones(length(H),1),'--k','LineWidth',2);
plot(H(:,25),-dX_max(3,3)*ones(length(H),1),'--k','LineWidth',2);
grid on;
legend('dZr','dZ','dZ_{max}');
axis([0 tmax -4 4]);
xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
ylabel('Velocidade no eixo Z','FontSize',12,'FontWeight','bold');

%%
figure(7);
plot(H(:,25),H(:,8),'r','LineWidth',2);
hold on;
plot(H(:,25),H(:,16),'k','LineWidth',2);
plot(H(:,25),dX_max(4,4)*ones(length(H),1),'--k','LineWidth',2);
plot(H(:,25),-dX_max(4,4)*ones(length(H),1),'--k','LineWidth',2);
grid on;
legend('dWr','dW','dW_{max}');
axis([0 tmax -1 1]);
xlabel('Tempo [s]','FontSize',12,'FontWeight','bold');
ylabel('Velocidade angular','FontSize',12,'FontWeight','bold');

