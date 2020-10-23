%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% PLOT DOS RESULTADOS DO EXPERIMENTO
% Pegando a matriz de DADOS
DADOS = evalin('base','DADOS');


%           1 -- 12         13 -- 24        25 -- 28        29 -- 32
%           A{1}.pPos.Xd'   A{1}.pPos.X'    A{1}.pSC.Ud'    A{1}.pSC.U'
%
%           33 -- 44        45 -- 56        57 -- 60        61 -- 64
%           A{2}.pPos.Xd'   A{2}.pPos.X'    A{2}.pSC.Ud'    A{2}.pSC.U'
%
%           65 -- 76        77 -- 88        89 -- 90        91 -- 92
%           P.pPos.Xd'      P.pPos.X'       P.pSC.Ud'       P.pSC.U'
% 
%           93 -- 101       102 -- 110      111 -- 119      120 -- 128
%           Qd'             Q'              Xd'             X'
% 
%           129 -- 140      141 -- 152      153
%           A{1}.pPar.Xr'   A{2}.pPar.Xr'   toc(T_Alfa)

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
figure
subplot(2,3,1)
plot(DADOS(:,end),DADOS(:,12+1))
hold on
plot(DADOS(:,end),DADOS(:,128+7))
grid on
title('X')
subplot(2,3,2)
plot(DADOS(:,end),DADOS(:,12+2))
hold on
plot(DADOS(:,end),DADOS(:,128+8))
grid on
title('Y')
subplot(2,3,3)
plot(DADOS(:,end),DADOS(:,12+3))
hold on
plot(DADOS(:,end),DADOS(:,128+9))
grid on
title('Z')
subplot(2,3,4)
plot(DADOS(:,end),DADOS(:,44+1))
hold on
plot(DADOS(:,end),DADOS(:,140+7))
grid on
title('X')
subplot(2,3,5)
plot(DADOS(:,end),DADOS(:,44+2))
hold on
plot(DADOS(:,end),DADOS(:,140+8))
grid on
title('Y')
subplot(2,3,6)
plot(DADOS(:,end),DADOS(:,44+3))
hold on
plot(DADOS(:,end),DADOS(:,140+9))
grid on
title('Z')

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
figure
subplot(3,3,1)
plot(DADOS(:,end),(DADOS(:,0+1)-DADOS(:,12+1)))
grid on
title('X')
subplot(3,3,2)
plot(DADOS(:,end),(DADOS(:,0+2)-DADOS(:,12+2)))
grid on
title('Y')
subplot(3,3,3)
plot(DADOS(:,end),(DADOS(:,0+3)-DADOS(:,12+3)))
grid on
title('Z')
subplot(3,3,4)
plot(DADOS(:,end),(DADOS(:,32+1)-DADOS(:,44+1)))
grid on
title('X')
subplot(3,3,5)
plot(DADOS(:,end),(DADOS(:,32+2)-DADOS(:,44+2)))
grid on
title('Y')
subplot(3,3,6)
plot(DADOS(:,end),(DADOS(:,32+3)-DADOS(:,44+3)))
grid on
title('Z')
subplot(3,3,7)
plot(DADOS(:,end),(DADOS(:,64+1)-DADOS(:,76+1)))
grid on
title('X')
subplot(3,3,8)
plot(DADOS(:,end),(DADOS(:,64+2)-DADOS(:,76+2)))
grid on
title('Y')
subplot(3,3,9)
plot(DADOS(:,end),(DADOS(:,64+3)-DADOS(:,76+3)))
grid on
title('Z')

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
figure
subplot(3,3,1)
plot(DADOS(:,end),DADOS(:,0+1))
hold on
plot(DADOS(:,end),DADOS(:,12+1))
grid on
title('X')
subplot(3,3,2)
plot(DADOS(:,end),DADOS(:,0+2))
hold on
plot(DADOS(:,end),DADOS(:,12+2))
grid on
title('Y')
subplot(3,3,3)
plot(DADOS(:,end),DADOS(:,0+3))
hold on
plot(DADOS(:,end),DADOS(:,12+3))
grid on
title('Z')
subplot(3,3,4)
plot(DADOS(:,end),DADOS(:,32+1))
hold on
plot(DADOS(:,end),DADOS(:,44+1))
grid on
title('X')
subplot(3,3,5)
plot(DADOS(:,end),DADOS(:,32+2))
hold on
plot(DADOS(:,end),DADOS(:,44+2))
grid on
title('Y')
subplot(3,3,6)
plot(DADOS(:,end),DADOS(:,32+3))
hold on
plot(DADOS(:,end),DADOS(:,44+3))
grid on
title('Z')
subplot(3,3,7)
plot(DADOS(:,end),DADOS(:,64+1))
hold on
plot(DADOS(:,end),DADOS(:,76+1))
grid on
title('X')
subplot(3,3,8)
plot(DADOS(:,end),DADOS(:,64+2))
hold on
plot(DADOS(:,end),DADOS(:,76+2))
grid on
title('Y')
subplot(3,3,9)
plot(DADOS(:,end),DADOS(:,64+3))
hold on
plot(DADOS(:,end),DADOS(:,76+3))
grid on
title('Z')

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
figure
subplot(3,3,1)
plot(DADOS(:,end),DADOS(:,110+1))
hold on
plot(DADOS(:,end),DADOS(:,119+1))
grid on
title('X')
subplot(3,3,2)
plot(DADOS(:,end),DADOS(:,110+2))
hold on
plot(DADOS(:,end),DADOS(:,119+2))
grid on
title('Y')
subplot(3,3,3)
plot(DADOS(:,end),DADOS(:,110+3))
hold on
plot(DADOS(:,end),DADOS(:,119+3))
grid on
title('Z')
subplot(3,3,4)
plot(DADOS(:,end),DADOS(:,110+4))
hold on
plot(DADOS(:,end),DADOS(:,119+4))
grid on
title('X')
subplot(3,3,5)
plot(DADOS(:,end),DADOS(:,110+5))
hold on
plot(DADOS(:,end),DADOS(:,119+5))
grid on
title('Y')
subplot(3,3,6)
plot(DADOS(:,end),DADOS(:,110+6))
hold on
plot(DADOS(:,end),DADOS(:,119+6))
grid on
title('Z')
subplot(3,3,7)
plot(DADOS(:,end),DADOS(:,110+7))
hold on
plot(DADOS(:,end),DADOS(:,119+7))
grid on
title('X')
subplot(3,3,8)
plot(DADOS(:,end),DADOS(:,110+8))
hold on
plot(DADOS(:,end),DADOS(:,119+8))
grid on
title('Y')
subplot(3,3,9)
plot(DADOS(:,end),DADOS(:,110+9))
hold on
plot(DADOS(:,end),DADOS(:,119+9))
grid on
title('Z')

figure
subplot(4,1,1)
plot(DADOS(:,end),DADOS(:,56+1))
grid on
title('X')
subplot(4,1,2)
plot(DADOS(:,end),DADOS(:,56+2))
grid on
title('Y')
subplot(4,1,3)
plot(DADOS(:,end),DADOS(:,56+3))
grid on
title('Z')
subplot(4,1,4)
plot(DADOS(:,end),DADOS(:,56+4))
grid on
title('PSI')

%%
figure
plot3(DADOS(:,44+1),DADOS(:,44+2),DADOS(:,44+3))
axis equal
% Circunferencia
AnguloZ = linspace(0,2*pi,30);
X = cos(AnguloZ);
Y = sin(AnguloZ);
Z = 1.5*ones(1,size(AnguloZ,2));

% Linear
% Ponto{1} = [0 0 0];
% Ponto{2} = [3 3 3];
% Parametro = linspace(0,norm([Ponto{1};Ponto{2}]),200);
% X = Ponto{2} + (Ponto{1} - Ponto{2})

hold on
plot3(X,Y,Z,'b.')
grid on
axis equal