%% Análise do intervalo de aquisição de imagens (05/02/2020)
% Obs: tanto para Bebop 2 quanto ArDrone

% Boas práticas
clearvars
close all
clc

% Conectar com ROS
RI = RosInterface; % 
setenv('ROS_IP','192.168.0.158') % IP do computador que está rodando o código principal
setenv('ROS_MASTER_URI','http://192.168.0.146:11311') % IP do computador que está rodando o mestre
RI.rConnect('192.168.0.146'); % Conectar ao mestre

% Inscrever no topico:
camA  = rossubscriber('/ardrone/image_raw/compressed');


% client = rossvcclient("/ardrone/togglecam"); % ativa camera inferior
% msg = rosmessage(client);
% calling = call(client,msg);

% camB = rossubscriber('/ardrone/bottom/image_raw/compressed'); % 
% databadB = receive(camB);
% imB =readImage(databadB);
% IBH{end+1,1} = imB;
% disp('Leu B')
% 
% subplot(212)
% imshow(imB)
% drawnow


t = tic;
ta = 0;
h = [];
IAH = {}; IBH = {};
disp('Preloop')

while toc(t) < 45
databadA = receive(camA);
imA =readImage(databadA);
IAH{end+1,1} = imA;
disp('Leu A')

subplot(211)
imshow(imA)

dt = abs(toc(t) - ta);
h = [h,dt];
fprintf('dt: %0.2g \n',dt)
ta = toc(t)
end

fprintf('dt_maximo: %0.2g \n',max(h))
fprintf('dt_medio: %0.2g \n',mean(h))
fprintf('dt_moda: %0.2g \n',mode(h))
fprintf('dt_variancia: %0.2g \n',var(h))
fprintf('dt_desvio_padrao: %0.2g \n',std(h))



%% Plots
[M,I] = maxk(h,5);
close all
figure(1)
stem(h)
hold on
plot(1:size(h,2),mean(h).*ones(1,size(h,2)),'b--','LineWidth',1.5)
hold on
plot(1:size(h,2),(mean(h)+std(h)).*ones(1,size(h,2)),'g--','LineWidth',1.5)
plot(1:size(h,2),(mean(h)-std(h)).*ones(1,size(h,2)),'g--','LineWidth',1.5)
stem(I,M,'ro','LineWidth',1.5)
plot(1:size(h,2),(1/30).*ones(1,size(h,2)),'r--','LineWidth',1.5)
plot(1:size(h,2),(mode(h)).*ones(1,size(h,2)),'k--','LineWidth',1.5)

p = polyfit(1:size(h,2),h,20);
y = polyval(p,1:size(h,2));
hold on
plot(y,'b','LineWidth',.5)

legend('\Delta_T','Mean','Mean+STD','Mean-STD','5 MAX','T_{min}','Mode','FontSize',14)
title('Image aquisistion interval','FontSize',16)
xlabel('Aquisition number (N)','FontSize',14)
ylabel('Time interval [s]','FontSize',14)
xlim([0,size(h,2)-1])
grid on
