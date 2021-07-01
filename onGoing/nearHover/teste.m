% Guiar drone virtual usando joystick
% Testar modelo dinâmico

close all
clear
clc
% profile on


A = ArDrone;
B = ArDrone;
mCADcolor(B,[0 1 0])


a = 2;
b = 2;
wrobo = 0.75; % Dados do robô

wmax = wrobo/sqrt(a^2+4*b^2);

perc = 4.35;
w = perc*wmax;

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

tmax = 3*60; % Tempo Simulação em segundos (min=40)
X = zeros(1,19); % Dados correntes da simulação
total = 0:1/30:tmax;
XX = zeros(29,size(total,2)); % Dados correntes da simulação
c =0;
total = tic;

tg = zeros(size(total));
for t = 0:1/30:tmax
    c=c+1;
            r = t*0.035+ 0.2;
            A.pPos.Xd(1) = r*cos(w*t);
            A.pPos.Xd(2) = r*sin(w*t);
            A.pPos.Xd(7) = -w*r*sin(w*t);
            A.pPos.Xd(8) = w*r*cos(w*t);
            A.pPos.Xd(3) = 1.2;
        
      
        tc = tic;
        A = cNearHoverController(A);
        tg(c) = toc(tc);
        
        XX(:,c) = [A.pPos.Xd; A.pPos.X; A.pSC.Ud; t];   
       
        A.pSC.U = A.pSC.Ud;
        
        A.rSendControlSignals;
end   
disp(toc(total))

tmax = 60; % Tempo Simulação em segundos (min=40)
X = zeros(1,19); % Dados correntes da simulação
total = 0:1/30:tmax;
XX2 = zeros(29,size(total,2)); % Dados correntes da simulação
c =0;
total = tic;
tg = zeros(size(total));
for t = 0:1/30:tmax
    c=c+1;
    
            r = t*0.035+ 0.2;
            B.pPos.Xd(1) = r*cos(w*t);
            B.pPos.Xd(2) = r*sin(w*t);
            B.pPos.Xd(7) = -w*r*sin(w*t);
            B.pPos.Xd(8) = w*r*cos(w*t);
            B.pPos.Xd(3) = 1.2;

        
        % Controlador
        B.rGetSensorData
        
        tc = tic;
        B = cUnderActuatedController(B);
        tg(c) = toc(tc);
        
        XX2(:,c) = [B.pPos.Xd; B.pPos.X; B.pSC.Ud; t];   
       
        B.rSendControlSignals;
end   
disp(toc(total))

%% Mostra simulação?
mostrar = 1;
if mostrar
    % =====================================================================
    figure(1)
    axis([-8 8 -8 8 0 3])
    grid on
    hold on
    view(3)
    disp('Start..........')
    % =====================================================================
    ta = 0; 
    hold on
    
rastroMax = 1000;
rastroAtu = diag(A.pPos.X(1:3))*ones(3,rastroMax);
rastroD = diag(A.pPos.Xd(1:3))*ones(3,rastroMax);
  
    for c = 1:8:length(XX)

        rastroAtu = [rastroAtu(:,3:end) XX(13:15,c)];
        rastroD = [rastroD(:,3:end) XX(1:3,c)];
        
        c1 = c;
        if XX(end,c)-ta > 1/10
            ta = XX(end,c);
            
            try
                delete(rastroArD)
            end
            
            rastroArD = plot3(rastroAtu(1,:),rastroAtu(2,:),rastroAtu(3,:),'k--');
            rastroDes = plot3(rastroD(1,:),rastroD(2,:),rastroD(3,:),'r--');
        end

   
        B.pPos.X = XX2(13:24,c);
        B.mCADplot;
        A.pPos.X = XX(13:24,c);
        A.mCADplot;
        drawnow
        
    end
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
% save MATsimNearHover_artfTime.mat
% save('MATsimNearHover_artfTime','-append')

