clc, clear, close all


idAtraso = 10;
% idAtraso = 1;
% filename = sprintf('C:%cUsers%cleo_j%cDropbox%cDropbox%cAuRoRA 2018%cTime-delayed Communication%cDados Simu%cPosition%csimu_atraso_%d.mat', '\', '\', '\', '\', '\', '\', '\', '\', '\', idAtraso);

% filename = sprintf('C:%cUsers%cleo_j%cDropbox%cDropbox%cAuRoRA 2018%cTime-delayed Communication%cDados Simu%cPosition%csimu_atraso_variavel1.mat', '\', '\', '\', '\', '\', '\', '\', '\', '\');
% filename = sprintf('C:%cUsers%cleo_j%cDropbox%cDropbox%cAuRoRA 2018%cTime-delayed Communication%cDados Simu%cPosition%csimu_atraso_variavel2.mat', '\', '\', '\', '\', '\', '\', '\', '\', '\');

% Lemniscata
% filename = sprintf('C:%cUsers%cleo_j%cDropbox%cDropbox%cAuRoRA 2018%cTime-delayed Communication%cDados Simu%cTrajectory Tracking%csimu_atraso_%d(Lemniscata).mat', '\', '\', '\', '\', '\', '\', '\', '\', '\', idAtraso);
% Circunferência
filename = sprintf('C:%cUsers%cleo_j%cDropbox%cDropbox%cAuRoRA 2018%cTime-delayed Communication%cDados Simu%cTrajectory Tracking%csimu_atraso_%d(Circunferência).mat', '\', '\', '\', '\', '\', '\', '\', '\', '\', idAtraso);

Arq = load(filename,'Hist');

A = ArDrone;    % Atrasado


f1 = figure('Name','Simulação Posicionamento ArDrone','NumberTitle','off');
% f1.Position = [435 2 930 682];
f1.Position = [1367 11 831 634]; % Segunda tela em Sete Lagoas!
figure(f1);

title('Task: Position Control')
xlabel({'Eixo $$x$$', '[m]'},'FontSize',12,'FontWeight','bold','interpreter','latex');
ylabel({'Eixo $$y$$', '[m]'},'FontSize',12,'FontWeight','bold','interpreter','latex');
zlabel({'Eixo $$z$$', '[m]'},'FontSize',12,'FontWeight','bold','interpreter','latex');
axis equal
axis([-4 2 -3 3 0 3])
view(3)
view(50,10)
grid on
hold on

A.mCADplot;

%%
A = ArDrone;    
A.pPos.X = Arq.Hist(200*1/2, 13:24)';
A.mCADplot;
hold on

%%
A = ArDrone;    
A.pPos.X = Arq.Hist(200*2/2, 13:24)';
A.mCADplot;
hold on

%%
A = ArDrone;    
A.pPos.X = Arq.Hist(200*4/2, 13:24)';
A.mCADplot;
hold on

%%
A = ArDrone;    % Atrasado
A.pPos.X = Arq.Hist(200*6/2, 13:24)';
A.mCADplot;
hold on

%%
A = ArDrone;    % Atrasado
A.pPos.X = Arq.Hist(200*5/2, 13:24)';
A.mCADplot;
hold on

%%
A = ArDrone;    % Atrasado
A.pPos.X = Arq.Hist(200*8/2, 13:24)';
A.mCADplot;
hold on

%%
% A = ArDrone;    % Atrasado
% A.pPos.X = Arq.Hist(end, 13:24)';
% A.mCADplot;
% hold on

%% - Plota rota Desejada e Realizada:
plot3(Arq.Hist(:,1), Arq.Hist(:,2),Arq.Hist(:,3),'-b') % Desejado
plot3(Arq.Hist(:,13), Arq.Hist(:,14),Arq.Hist(:,15),'-r') % Realizado

axis([-2 2 -2 2.25 0 2.25])

