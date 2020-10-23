close all;% clc;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% PLOT DOS RESULTADOS DO EXPERIMENTO
% Pegando a matriz de DADOS
DADOS = evalin('base','DADOS');
F = 1;

%           1 -- 12         13 -- 24        25 -- 26        
%           P.pPos.Xd'      P.pPos.X'       P.pSC.Ud'    
%
%           27 -- 38        39 -- 50        51 -- 62        63 -- 66
%           A{1}.pPos.Xd'   A{1}.pPos.X'    A{1}.pPos.Xr'   A{1}.pSC.U'
%
%           67 -- 78        79 -- 90        91 -- 102       103 -- 106
%           A{2}.pPos.Xd'   A{2}.pPos.X'    A{2}.pPos.Xr'   A{2}.pSC.U'
% 
%           107 -- 115      116 -- 124      125 -- 133      134 -- 142
%           Qd'             Q'              dQd'            dQ'
% 
%           143 -- 151      152 -- 160      161 -- 169
%           Xd'             X'              dX'
% 
%           170             171
%           toc(T)          toc(T_ALFA)

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% SIMULAÇÃO
SIMULACAO = 0;

figure(F)
F = F + 1;
Point(1) = plot3(DADOS(1,12+1),DADOS(1,12+2),DADOS(1,12+3),'ok','MarkerSize',10,'LineWidth',2);
hold on
Point(2) = plot3(DADOS(1,38+1),DADOS(1,38+2),DADOS(1,38+3),'^k','MarkerSize',10,'LineWidth',2);
Point(3) = plot3(DADOS(1,78+1),DADOS(1,78+2),DADOS(1,78+3),'^k','MarkerSize',10,'LineWidth',2);
H(1) = plot3([DADOS(1,12+1) DADOS(1,38+1)],[DADOS(1,12+2) DADOS(1,38+2)],[DADOS(1,12+3) DADOS(1,38+3)],'b','LineWidth',2);
H(2) = plot3([DADOS(1,12+1) DADOS(1,78+1)],[DADOS(1,12+2) DADOS(1,78+2)],[DADOS(1,12+3) DADOS(1,78+3)],'b','LineWidth',2);
H(3) = plot3([DADOS(1,38+1) DADOS(1,78+1)],[DADOS(1,38+2) DADOS(1,78+2)],[DADOS(1,38+3) DADOS(1,78+3)],'r','LineWidth',2);
TPlot(1) = text(DADOS(1,12+1)+.1,DADOS(1,12+2)-.1,DADOS(1,12+3)+.2,'P','FontWeight','bold');
TPlot(2) = text(DADOS(1,38+1)+.1,DADOS(1,38+2)-.1,DADOS(1,38+3)+.2,'A1','FontWeight','bold');
TPlot(3) = text(DADOS(1,78+1)+.1,DADOS(1,78+2)-.1,DADOS(1,78+3)+.2,'A2','FontWeight','bold');
grid on
axis([-3 3 -3 3 DADOS(1,12+3) 3])
view([-50 30])

title('SIMULAÇÃO','FontWeight','bold')
xlabel('X','FontWeight','bold')
ylabel('Y','FontWeight','bold')
zlabel('Z','FontWeight','bold')

if SIMULACAO == 1
pause(3)
end

T_TEMP = DADOS(1,end);
ii = 2;
ii_temp = ii;

while ii <= size(DADOS,1)   
    if SIMULACAO == 1
    clc
    disp(DADOS(ii,end))
    try
        delete(Point)
        delete(H)
        delete(TPlot)
    catch
    end
    Point(1) = plot3(DADOS(ii,12+1),DADOS(ii,12+2),DADOS(ii,12+3),'ok','MarkerSize',10,'LineWidth',2);
    Point(2) = plot3(DADOS(ii,38+1),DADOS(ii,38+2),DADOS(ii,38+3),'^k','MarkerSize',10,'LineWidth',2);
    Point(3) = plot3(DADOS(ii,78+1),DADOS(ii,78+2),DADOS(ii,78+3),'^k','MarkerSize',10,'LineWidth',2);
    H(1) = plot3([DADOS(ii,12+1) DADOS(ii,38+1)],[DADOS(ii,12+2) DADOS(ii,38+2)],[DADOS(ii,12+3) DADOS(ii,38+3)],'b','LineWidth',2);
    H(2) = plot3([DADOS(ii,12+1) DADOS(ii,78+1)],[DADOS(ii,12+2) DADOS(ii,78+2)],[DADOS(ii,12+3) DADOS(ii,78+3)],'b','LineWidth',2);
    H(3) = plot3([DADOS(ii,38+1) DADOS(ii,78+1)],[DADOS(ii,38+2) DADOS(ii,78+2)],[DADOS(ii,38+3) DADOS(ii,78+3)],'r','LineWidth',2);
    end
    plot3([DADOS(ii_temp,12+1) DADOS(ii,12+1)],[DADOS(ii_temp,12+2) DADOS(ii,12+2)],[DADOS(ii_temp,12+3) DADOS(ii,12+3)],'-r','LineWidth',1.3);
    plot3([DADOS(ii_temp,38+1) DADOS(ii,38+1)],[DADOS(ii_temp,38+2) DADOS(ii,38+2)],[DADOS(ii_temp,38+3) DADOS(ii,38+3)],'-g','LineWidth',1.3);
    plot3([DADOS(ii_temp,78+1) DADOS(ii,78+1)],[DADOS(ii_temp,78+2) DADOS(ii,78+2)],[DADOS(ii_temp,78+3) DADOS(ii,78+3)],'-b','LineWidth',1.3);
    
    if SIMULACAO == 1
    drawnow
    end
    
    ii_temp = ii;
    ii = ii + 4;
end

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% ERROS

% Pioneer
figure(F)
F = F + 1;
subplot(2,1,1)
plot(DADOS(:,end),(DADOS(:,1)-DADOS(:,12+1)),'--b','LineWidth',1.5)
hold on
grid on
ylabel('Erro(m)','FontWeight','bold')
title('ERRO DE POSIÇÃO PIONEER','FontWeight','bold')
plot(DADOS(:,end),(DADOS(:,2)-DADOS(:,12+2)),'-r','LineWidth',1.5)
plot(DADOS(:,end),(DADOS(:,3)-DADOS(:,12+3)),'-.g','LineWidth',1.5)
subplot(2,1,2)
plot(DADOS(:,end),(DADOS(:,6)-DADOS(:,12+6))*180/pi,'--b','LineWidth',1.5)
grid on
ylabel('Psitil(graus)','FontWeight','bold')
xlabel('Tempo(s)','FontWeight','bold')

% ArDrone 1
figure(F)
F = F + 1;
subplot(2,1,1)
plot(DADOS(:,end),(DADOS(:,38+1)-DADOS(:,142+3+1)),'--b','LineWidth',1.5)
hold on
grid on
ylabel('Erro(m)','FontWeight','bold')
title('ERRO DE POSIÇÃO ARDRONE 1','FontWeight','bold')
plot(DADOS(:,end),(DADOS(:,38+2)-DADOS(:,142+3+2)),'-r','LineWidth',1.5)
plot(DADOS(:,end),(DADOS(:,38+3)-DADOS(:,142+3+3)),'-.g','LineWidth',1.5)
subplot(2,1,2)
plot(DADOS(:,end),(DADOS(:,38+6)-DADOS(:,26+12+6))*180/pi,'--b','LineWidth',1.5)
grid on
ylabel('Psitil(graus)','FontWeight','bold')
xlabel('Tempo(s)','FontWeight','bold')

% ArDrone 2
figure(F)
F = F + 1;
subplot(2,1,1)
plot(DADOS(:,end),(DADOS(:,78+1)-DADOS(:,142+6+1)),'--b','LineWidth',1.5)
hold on
grid on
ylabel('Erro(m)','FontWeight','bold')
title('ERRO DE POSIÇÃO ARDRONE 2','FontWeight','bold')
plot(DADOS(:,end),(DADOS(:,78+2)-DADOS(:,142+6+2)),'-r','LineWidth',1.5)
plot(DADOS(:,end),(DADOS(:,78+3)-DADOS(:,142+6+3)),'-.g','LineWidth',1.5)
subplot(2,1,2)
plot(DADOS(:,end),(DADOS(:,78+6)-DADOS(:,66+12+6))*180/pi,'--b','LineWidth',1.5)
grid on
ylabel('Psitil(graus)','FontWeight','bold')
xlabel('Tempo(s)','FontWeight','bold')

% Formação
figure(F)
F = F + 1;
subplot(3,3,1:3)
plot(DADOS(:,end),(DADOS(:,106+1)-DADOS(:,106+9+1)),'--b','LineWidth',1.5)
hold on
grid on
ylabel('Erro(m)','FontWeight','bold')
title('ERRO DE POSIÇÃO DA FORMAÇÃO','FontWeight','bold')
plot(DADOS(:,end),(DADOS(:,106+2)-DADOS(:,106+9+2)),'-r','LineWidth',1.5)
plot(DADOS(:,end),(DADOS(:,106+3)-DADOS(:,106+9+3)),'-.g','LineWidth',1.5)
subplot(3,3,4:6)
plot(DADOS(:,end),(DADOS(:,106+4)-DADOS(:,106+9+4))*180/pi,'--b','LineWidth',1.5)
hold on
grid on
ylabel('Erro(graus)','FontWeight','bold')
title('ERRO DE ORIENTAÇÃO DA FORMAÇÃO','FontWeight','bold')
plot(DADOS(:,end),(DADOS(:,106+5)-DADOS(:,106+9+5))*180/pi,'-r','LineWidth',1.5)
plot(DADOS(:,end),(DADOS(:,106+6)-DADOS(:,106+9+6))*180/pi,'-.g','LineWidth',1.5)
subplot(3,3,8)
xlabel('Tempo(s)','FontWeight','bold')
title('ERRO DE POSTURA DA FORMAÇÃO','FontWeight','bold')
subplot(3,3,7:8)
plot(DADOS(:,end),(DADOS(:,106+7)-DADOS(:,106+9+7)),'--b','LineWidth',1.5)
hold on
grid on
ylabel('Erro(m)','FontWeight','bold')
plot(DADOS(:,end),(DADOS(:,106+8)-DADOS(:,106+9+8)),'-r','LineWidth',1.5)
subplot(3,3,9)
plot(DADOS(:,end),(DADOS(:,106+9)-DADOS(:,106+9+9))*180/pi,'--b','LineWidth',1.5)
grid on
ylabel('Betatil(graus)','FontWeight','bold')

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% DESEJADO X REAL

% Pioneer
figure(F)
F = F + 1;
subplot(4,1,1)
plot(DADOS(:,end),DADOS(:,142+1),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,12+1),'-r','LineWidth',1.5)
grid on
ylabel('X(m)','FontWeight','bold')
title('POSIÇÃO DO PIONEER','FontWeight','bold')
subplot(4,1,2)
plot(DADOS(:,end),DADOS(:,142+2),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,12+2),'-r','LineWidth',1.5)
grid on
ylabel('Y(m)','FontWeight','bold')
subplot(4,1,3)
plot(DADOS(:,end),DADOS(:,142+3),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,12+3),'-r','LineWidth',1.5)
grid on
ylabel('Z(m)','FontWeight','bold')
subplot(4,1,4)
plot(DADOS(:,end),DADOS(:,6)*180/pi,'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,12+6),'-r','LineWidth',1.5)
grid on
ylabel('Psi(°)','FontWeight','bold')
xlabel('Tempo(s)','FontWeight','bold')

% ArDrone 1
figure(F)
F = F + 1;
subplot(4,1,1)
plot(DADOS(:,end),DADOS(:,142+3+1),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,38+1),'-r','LineWidth',1.5)
grid on
ylabel('X(m)','FontWeight','bold')
title('POSIÇÃO ARDRONE 1','FontWeight','bold')
subplot(4,1,2)
plot(DADOS(:,end),DADOS(:,142+3+2),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,38+2),'-r','LineWidth',1.5)
grid on
ylabel('Y(m)','FontWeight','bold')
subplot(4,1,3)
plot(DADOS(:,end),DADOS(:,142+3+3),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,38+3),'-r','LineWidth',1.5)
grid on
ylabel('Z(m)','FontWeight','bold')
subplot(4,1,4)
plot(DADOS(:,end),DADOS(:,26+6)*180/pi,'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,38+6),'-r','LineWidth',1.5)
grid on
ylabel('Psi(°)','FontWeight','bold')
xlabel('Tempo(s)','FontWeight','bold')

% ArDrone 2
figure(F)
F = F + 1;
subplot(4,1,1)
plot(DADOS(:,end),DADOS(:,142+6+1),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,78+1),'-r','LineWidth',1.5)
grid on
ylabel('X(m)','FontWeight','bold')
title('POSIÇÃO ARDRONE 2','FontWeight','bold')
subplot(4,1,2)
plot(DADOS(:,end),DADOS(:,142+6+2),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,78+2),'-r','LineWidth',1.5)
grid on
ylabel('Y(m)','FontWeight','bold')
subplot(4,1,3)
plot(DADOS(:,end),DADOS(:,142+6+3),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,78+3),'-r','LineWidth',1.5)
grid on
ylabel('Z(m)','FontWeight','bold')
subplot(4,1,4)
plot(DADOS(:,end),DADOS(:,66+6)*180/pi,'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,78+6),'-r','LineWidth',1.5)
grid on
ylabel('Psi(°)','FontWeight','bold')
xlabel('Tempo(s)','FontWeight','bold')

% Formação
figure(F)
F = F + 1;
subplot(3,3,1)
plot(DADOS(:,end),DADOS(:,106+1),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,106+9+1),'-r','LineWidth',1.5)
grid on
ylabel('Xtil(m)','FontWeight','bold')
subplot(3,3,2)
plot(DADOS(:,end),DADOS(:,106+2),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,106+9+2),'-r','LineWidth',1.5)
grid on
ylabel('Ytil(m)','FontWeight','bold')
title('POSIÇÃO DA FORMAÇÃO','FontWeight','bold')
subplot(3,3,3)
plot(DADOS(:,end),DADOS(:,106+3),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,106+9+3),'-r','LineWidth',1.5)
grid on
ylabel('Ztil(m)','FontWeight','bold')
subplot(3,3,4)
plot(DADOS(:,end),DADOS(:,106+4)*180/pi,'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,106+9+4)*180/pi,'-r','LineWidth',1.5)
grid on
ylabel('Thetatil(°)','FontWeight','bold')
subplot(3,3,5)
plot(DADOS(:,end),DADOS(:,106+5)*180/pi,'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,106+9+5)*180/pi,'-r','LineWidth',1.5)
grid on
ylabel('Phitil(°)','FontWeight','bold')
title('ORIENTAÇÃO DA FORMAÇÃO','FontWeight','bold')
subplot(3,3,6)
plot(DADOS(:,end),DADOS(:,106+6)*180/pi,'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,106+9+6)*180/pi,'-r','LineWidth',1.5)
grid on
ylabel('Psitil(°)','FontWeight','bold')
subplot(3,3,7)
plot(DADOS(:,end),DADOS(:,106+7),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,106+9+7),'-r','LineWidth',1.5)
grid on
ylabel('Ptil(m)','FontWeight','bold')
subplot(3,3,8)
plot(DADOS(:,end),DADOS(:,106+8),'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,106+9+8),'-r','LineWidth',1.5)
grid on
ylabel('Qtil(m)','FontWeight','bold')
xlabel('Tempo(s)','FontWeight','bold')
title('POSTURA DA FORMAÇÃO','FontWeight','bold')
subplot(3,3,9)
plot(DADOS(:,end),DADOS(:,106+9)*180/pi,'--b','LineWidth',1.5)
hold on
plot(DADOS(:,end),DADOS(:,106+9+9)*180/pi,'-r','LineWidth',1.5)
grid on
ylabel('Betatil(°)','FontWeight','bold')














