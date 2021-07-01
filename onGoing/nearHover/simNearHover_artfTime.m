% Guiar drone virtual usando joystick
% Testar modelo dinâmico

close all
clear
clc
% profile on


A = ArDrone;

% % =========================================================================
% figure(1)
% axis([-3 3 -3 3 0 3])
% grid on
% A.mCADplot
% drawnow
% pause(1)
% view(3)
% disp('Start..........')
% 
% % =========================================================================

tmax = 60; % Tempo Simulação em segundos (min=40)
X = zeros(1,19); % Dados correntes da simulação
total = 0:1/30:tmax;
XX = zeros(29,size(total,2)); % Dados correntes da simulação
c =0;
total = tic;

tg = zeros(size(total));
for t = 0:1/30:tmax
    c=c+1;
        if t > 3*tmax/4
            A.pPos.Xd(1) = 0;
            A.pPos.Xd(2) = 0;
            A.pPos.Xd(3) = 1.5;
            A.pPos.Xd(6) = pi/4;
        elseif (t) > 2*tmax/4
            A.pPos.Xd(1) = -1;
            A.pPos.Xd(2) = 0;
            A.pPos.Xd(3) = 2;
            A.pPos.Xd(6) = 0;
        elseif (t) > tmax/4
            A.pPos.Xd(1) = 1;
            A.pPos.Xd(2) = 1;
            A.pPos.Xd(3) = 1;
            A.pPos.Xd(6) = 0;
        else
            A.pPos.Xd(1) = 2;
            A.pPos.Xd(2) = -1;
            A.pPos.Xd(3) = 1;
            A.pPos.Xd(6) = pi/4;
        end
        
      
        tc = tic;
        A = cNearHoverController(A);
        tg(c) = toc(tc);
        
        XX(:,c) = [A.pPos.Xd; A.pPos.X; A.pSC.Ud; t];   
       
        A.pSC.U = A.pSC.Ud;
        
        A.rSendControlSignals;
end   
disp(toc(total))

%% Mostra simulação?
mostrar = 1;
if mostrar
    showSim(XX,A)
end


%% Plotar figuras?
plotar = 0;
if plotar
    f = figure;
    f.Name = 'phi';
    subplot(211),plot(XX(end,:),XX([4 16],:)'*180/pi)
    legend('\phi_{Des}','\phi_{Atu}')
    grid
    subplot(212),plot(XX(end,:),XX([5 17],:)'*180/pi)
    legend('\theta_{Des}','\theta_{Atu}')
    grid

    f = figure;
    f.Name = 'z';
    subplot(211),plot(XX(end,:),XX([3 15],:)')
    legend('z_{Des}','z_{Atu}')
    grid
    subplot(212),plot(XX(end,:),XX([6 18],:)'*180/pi)
    legend('\psi_{Des}','\psi_{Atu}')
    grid

    f = figure;
    f.Name = 'phi e theta';
    subplot(211),plot(XX(end,:),XX(25,:))
    legend('\phi_{Des}')
    grid
    subplot(212),plot(XX(end,:),XX(26,:))
    legend('\theta_{Des}')
    grid

    f = figure;
    f.Name = 'x';
    subplot(211),plot(XX(end,:),XX([1 13],:)')
    legend('x_{Des}','x_{Atu}')
    grid
    subplot(212),plot(XX(end,:),XX([2 14],:)')
    legend('y_{Des}','y_{Atu}')
    grid

    f = figure;
    f.Name = 'dx,dy,dz';
    subplot(311),plot(XX(end,:),XX([7 19],:)')
    legend('dx_{Des}','dx_{Atu}')
    grid
    subplot(312),plot(XX(end,:),XX([8 20],:)')
    legend('dy_{Des}','dy_{Atu}')
    grid
    subplot(313),plot(XX(end,:),XX([9 21],:)')
    axis([0 tmax -1 1])
    legend('dz_{Des}','dz_{Atu}')
    grid
end

%%
save MATsimNearHover_artfTime.mat
save('MATsimNearHover_artfTime','-append')

