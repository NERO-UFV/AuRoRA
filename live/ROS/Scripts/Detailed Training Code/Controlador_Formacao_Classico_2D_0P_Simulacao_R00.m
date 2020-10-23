%% 3D Line Formation Pioneer-Drone
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [ xf yf zf rho alfa beta ]
% Initial Comands

clear
close all
clc


%% Variable initialization
formacao_hist = [];
X1_hist = [];
X2_hist = [];
X1d_hist = [];
X2d_hist = [];

X1 = zeros(12,1);
X2 = zeros(12,1);

dXb1 = zeros(4,1);
dXb2 = zeros(4,1);

U1 = zeros(6,1);
U2 = zeros(6,1);

X1(1:3) = [0 0 1]';
X2(1:3) = [0 9 1]';

X1(6) = pi/2;
X2(6) = pi/2;

Vd1A = 0;
Vd2A = 0;

% Time variables initialization
T_CONTROL = 0.02; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 20;

L1 = diag([1 1 1 1 1 1]);
L2 = diag([1 1 .5 .5 .5 .5]);
K = diag([.8 .8 .8 .8]);
Ku = diag([0.8417 0.8354 3.966 9.8524]);
Kv = diag([0.18227 0.17095 4.001 4.7295]);

t  = tic;
t_control = tic;
t_der = tic;
t_sim = tic;

% Formação
T = 20;
w = 2*pi/T;
A = [2 2 1]';
C = [3 3 3]';

rX = 2;           % [m]
rY = 2;           % [m]            % [s]
Tf = 2*T;            % [s]        % [rad/s]

figure
   
while toc(t) < T_MAX

    if toc(t_control) > T_CONTROL             

        t_control = tic;
   
%         qd = [0 4 5 2 0 pi/3]';
%         dqd = [0 0 0 0 0 0]';
        
%       Trajetoria formato cirtular
%         qd = [C(1) + A(1)*cos(w*toc(t)) C(2) + A(2)*sin(w*toc(t)) C(3) + A(3)*sin(w*toc(t)) 2 0 pi/3]';
%         dqd = [-A(1)*w*sin(w*toc(t)) A(2)*w*cos(w*toc(t)) A(3)*w*cos(w*toc(t)) 0 0 0]';
                
        % Trajetoria em formato de Pringles
        qd = [C(1) + A(1)*cos(w*toc(t)) C(2) + A(2)*sin(w*toc(t)) C(3) + A(3)*sin(2*w*toc(t)) 2 0 pi/3]';
        dqd = [-A(1)*w*sin(w*toc(t)) A(2)*w*cos(w*toc(t)) A(3)*2*w*cos(2*w*toc(t)) 0 0 0]';
        
        % Trajetoria em formato de Lemniscata
%         t_traj = toc(t);
%         a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
%         tp = a*Tf; 
% 
%         qd = [C(1) + rX*sin(w*tp) C(2) + rY*sin(2*w*tp) C(3) + 0.5*sin(w*tp) 5 0 pi/3]';
%         dqd = [w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp) 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp) w*0.5*cos(w*tp) 0 0 0]';

        
        
        rho = norm(X2(1:3) - X1(1:3));
        alpha = atan2(X2(2) - X1(2),X2(1) - X1(1));
        beta = atan2(X2(3) - X1(3),norm(X2(1:2) - X1(1:2)));
        q = [X1(1) X1(2) X1(3) rho alpha beta]';

        qtil = qd - q;

        dqref = dqd + L1*tanh(L2*qtil);

        J_inv = [1 0 0           0                       0                          0          ;...
                 0 1 0           0                       0                          0          ;...
                 0 0 1           0                       0                          0          ;...
                 1 0 0 cos(alpha)*cos(beta) -rho*sin(alpha)*cos(beta) -rho*cos(alpha)*sin(beta);...
                 0 1 0 sin(alpha)*cos(beta)  rho*cos(alpha)*cos(beta) -rho*sin(alpha)*sin(beta);...
                 0 0 1       sin(beta)                   0                    rho*cos(beta)    ];

        dXref = J_inv*dqref;

        F1 = [  cos(X1(6))   -sin(X1(6))     0     0; % Cinemática direta
                sin(X1(6))    cos(X1(6))     0     0;
                    0             0          1     0;
                    0             0          0     1];

        F2 = [  cos(X2(6))   -sin(X2(6))     0     0; % Cinemática direta
                sin(X2(6))    cos(X2(6))     0     0;
                    0             0          1     0;
                    0             0          0     1];

        dXd1 = [dXref(1:3); 0];
        dXd2 = [dXref(4:6); 0];

        Vd1 = F1\dXd1;
        Vd2 = F2\dXd2;

        %Derivando a velocidade desejada
        dVd1 = (Vd1 - Vd1A)/toc(t_der);
        dVd2 = (Vd2 - Vd2A)/toc(t_der);
        t_der = tic;

        Vd1A = Vd1;
        Vd2A = Vd2;

        %Velocidade do robô em seu próprio eixo
        Vb1 = F1\([X1(7:9);X1(12)]);
        Vb2 = F2\([X2(7:9);X2(12)]);

        Ud1 = Ku\(dVd1 + K*(Vd1 - Vb1) + Kv*Vb1);
        Ud2 = Ku\(dVd2 + K*(Vd2 - Vb2) + Kv*Vb2);

        U1 = [Ud1(1:3)' 0 0 Ud1(4)]';
        U2 = [Ud2(1:3)' 0 0 Ud2(4)]';

        
        % SIMULADOR
        dXb1 = F1\[X1(7:9)' X1(12)]';
        dXb2 = F2\[X2(7:9)' X2(12)]';
        
        ddX1 = Ku*[U1(1:3)' U1(6)]' - Kv*dXb1;
        ddX2 = Ku*[U2(1:3)' U2(6)]' - Kv*dXb2;
        
        dXb1 = dXb1 + ddX1*toc(t_sim);
        dXb2 = dXb2 + ddX2*toc(t_sim);
        
        X1_aux = F1*dXb1;
        X2_aux = F2*dXb2;
        
        X1(7:9) = X1_aux(1:3);
        X1(12) = X1_aux(4);
        X2(7:9) = X2_aux(1:3);
        X2(12) = X2_aux(4);
        
        X1(1:3) = X1(1:3) + X1(7:9)*toc(t_sim);
        X1(6) = X1(6) + X1(12)*toc(t_sim);
        X2(1:3) = X2(1:3) + X2(7:9)*toc(t_sim);
        X2(6) = X2(6) + X2(12)*toc(t_sim);
        
        t_sim = tic;
        
        X1d = qd(1:3);
        X2d = [qd(1) + qd(4)*cos(qd(5))*cos(qd(6)); qd(2) + qd(4)*sin(qd(5))*cos(qd(6)); qd(3) + qd(4)*sin(qd(6))];
        
        formacao_hist = [formacao_hist;qd' q' toc(t)];
        X1_hist = [X1_hist; X1'];
        X2_hist = [X2_hist; X2'];
        X1d_hist = [X1d_hist; X1d'];
        X2d_hist = [X2d_hist; X2d'];
                
        plot3(X1(1),X1(2),X1(3),'g*')
        axis([-1 10 -1 10 0 10])
        hold on
        grid on
        plot3(X2(1),X2(2),X2(3),'r*')
        plot3(X1d(1),X1d(2),X1d(3),'k*')
        plot3(X2d(1),X2d(2),X2d(3),'b*')
        plot3(X1_hist(:,1),X1_hist(:,2),X1_hist(:,3),'g')
        plot3(X2_hist(:,1),X2_hist(:,2),X2_hist(:,3),'r')
        plot3(X1d_hist(:,1),X1d_hist(:,2),X1d_hist(:,3),'k--')
        plot3(X2d_hist(:,1),X2d_hist(:,2),X2d_hist(:,3),'b--')
        hold off
        drawnow

    end
end


%%
qtil = formacao_hist(:,1:6) - formacao_hist(:,7:end-1);
% 
% figure();
% hold on;
% grid on;
% plot(formacao_hist(:,13),qtil(:,1));
% title('Erro de Posição');
% legend('Pos X');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(formacao_hist(:,13),qtil(:,2));
% title('Erro de Posição');
% legend('Pos Y');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(formacao_hist(:,13),qtil(:,3));
% title('Erro de Posição');
% legend('Pos Z');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(formacao_hist(:,13),qtil(:,4));
% title('Erro de Posição');
% legend('Rho');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(formacao_hist(:,13),qtil(:,5));
% title('Erro de Posição');
% legend('Alpha');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
% 
% figure();
% hold on;
% grid on;
% plot(formacao_hist(:,13),qtil(:,6));
% title('Erro de Posição');
% legend('Beta');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');
