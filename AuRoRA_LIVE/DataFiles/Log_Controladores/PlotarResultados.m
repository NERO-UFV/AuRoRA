% Plotar Resultados

clear 
close all
clc

% Dinâmico
D = load('20180415T223258.txt');

% Cinemático
C = load('20180415T223526.txt');

figure
% Posição XY
plot(D(:,1),D(:,2),'g--')
hold,grid on
plot(D(:,13),D(:,14),'k-')
plot(C(:,13),C(:,14),'r-')
axis([-3.5 3.5 -1.5 1.5])
legend(['Ref';'DC ';'KC '])
xlabel('x [m]','interpreter','latex')
ylabel('y [m]','interpreter','latex')

figure
% Sinal de controle
subplot(211),hold,grid on
plot(D(:,end),D(:,27),'k-')
plot(C(:,end),C(:,27),'r-')
axis([0 120 0 0.5])
legend(['DC';'KC'])
xlabel('Time [s]','interpreter','latex')
ylabel('u [m/s]','interpreter','latex')

subplot(212),hold,grid on
plot(D(:,end),D(:,28),'k-')
plot(C(:,end),C(:,28),'r-')
axis([0 120 -1 1])
legend(['DC';'KC'])
xlabel('Time [s]','interpreter','latex')
ylabel('$\omega$ [rad/s]','interpreter','latex')

figure
% Sinal de controle
subplot(211),hold,grid on
plot(D(:,end),D(:,1)-D(:,13),'k-')
plot(C(:,end),C(:,1)-C(:,13),'r-')
axis([0 120 -0.2 0.4])
legend(['DC';'KC'])
xlabel('Time [s]','interpreter','latex')
ylabel('$\tilde{x}$ [m]','interpreter','latex')

subplot(212),hold,grid on
plot(D(:,end),D(:,2)-D(:,14),'k-')
plot(C(:,end),C(:,2)-C(:,14),'r-')
axis([0 120 -0.1 0.15])
legend(['DC';'KC'])
xlabel('Time [s]','interpreter','latex')
ylabel('$\tilde{y}$ [m]','interpreter','latex')

figure
hold,grid on
for ii = 1:length(D)
    De(ii) = norm(D(ii,1:2)-D(ii,13:14));
end
for ii = 1:length(C)
    Ce(ii) = norm(C(ii,1:2)-C(ii,13:14));
end
axis([0 60 0 0.35])
plot(D(:,end),De,'k-')
plot(C(:,end),Ce,'r-')
% axis([0 120 -1 1])
legend(['DC';'KC'])
xlabel('Time [s]','interpreter','latex')
ylabel('Error [m]','interpreter','latex')

