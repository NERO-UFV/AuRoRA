% Testar modelo dinâmico em uma trajetória

close all
clear
clc

A = ArDrone;

tmax = 40; % Tempo Simulação em segundos
X = zeros(1,19); % Dados correntes da simulação

figure(1)
drone = plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'bo','LineWidth',2,'MarkerSize',5);
axis([-3 3 -3 3 0 3])
grid on
% A.mCADplot
% drawnow
grid on
pause(2)
view(3)
disp('Start............')

% =========================================================================
t = tic;
tc = tic;
tp = tic;

XX = [];
TT = [];
while toc(t) < tmax
    if toc(tc) > 1/30
        TT = [TT toc(tc)];
        
        tc = tic;
        
        w = 0.025;
        % Trajetória desejada
        tt = toc(t);
        A.pPos.Xd(1) = 1*sin(2*pi*w*tt);            % x
        A.pPos.Xd(7) = 1*2*pi*w*cos(2*pi*w*tt);     % dx
        A.pPos.Xd(2) = 1*sin(2*pi*2*w*tt);          % y
        A.pPos.Xd(8) = 1*2*pi*2*w*cos(2*pi*2*w*tt); % dy
        A.pPos.Xd(3) = 0.5*cos(2*pi*w*tt) +1.25;    % z       
                        
        % Controlador
        A.rGetSensorData
        A = cNearHoverController(A);        
        A.rSendControlSignals;
        
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];
        
    end
%     if toc(tp) > 0.3
%         tp = tic;
%    
%         hold on
%         desj = plot3(XX(1,:),XX(2,:),XX(3,:),'r--');
%         
%         drawnow
%      
%     end
%     delete(drone)
%     drone = plot3(A.pPos.X(1),A.pPos.X(2),A.pPos.X(3),'bo','LineWidth',2,'MarkerSize',5);
%     axis([-3 3 -3 3 0 3]); grid on
%     
%     if mod(toc(t),15)==0
%         delete(desej)
%     end
    
end

figure
subplot(211),plot(XX(end,:),XX([4 16],:)'*180/pi)
legend('\phi_{Des}','\phi_{Atu}')
grid
subplot(212),plot(XX(end,:),XX([5 17],:)'*180/pi)
legend('\theta_{Des}','\theta_{Atu}')
grid

figure
plot(XX([1,13],:)',XX([2,14],:)')
% axis([-1.5 1.5 -1.5 1.5])
axis equal

figure
subplot(211),plot(XX(end,:),XX([1 13],:)')
legend('x_{Des}','x_{Atu}')
grid
subplot(212),plot(XX(end,:),XX([2 14],:)')
legend('y_{Des}','y_{Atu}')
grid

%%
figure
% dt = diff(XX(end,:));
% Todas
% qponto = vecnorm(XX(13:15,:)-XX(1:3,:));
% q = vecnorm(XX(19:21,:)-XX(7:9,:));

% Somente X
% qponto = XX(13,:)'-XX(1,:)';
% q = XX(19,:)'-XX(7,:)';

%Somente Y
% qponto = (XX(14,:)'-XX(2,:)');
% q =  (XX(20,:)'-XX(8,:)');

% Somente Z
% qponto = XX(15,:)'-XX(3,:)';
% q = XX(21,:)'-XX(9,:)';

% Somente phi
% qponto = XX(16,:)'-XX(4,:)';
% q = XX(22,:)'-XX(10,:)';

% Somente theta
qponto = XX(17,:)'-XX(5,:)';
q = XX(23,:)'-XX(11,:)';

% Somente psi
% qponto = XX(18,:)'-XX(6,:)';
% q = XX(24,:)'-XX(12,:)';

grid on
xlim([min(q),max(q)])
ylim([min(qponto),max(qponto)])
xlabel('$\tilde{q}$','Interpreter', 'LaTeX','FontSize',15)
ylabel('$$\dot{\tilde{q}}$$','Interpreter', 'LaTeX','FontSize',15)
title('Retrato de fase')
hold on
plot(0,0,'ro','MarkerSize',10)
comet(q,qponto)


% figure
% subplot(211),plot(XX(end,:),XX(19,:)')
% legend('x_{Des}','x_{Atu}')
% grid
% subplot(212),plot(XX(end,:),[XX(13,:); [0 diff(XX(13,:))]*30])
% legend('y_{Des}','y_{Atu}')
% grid

