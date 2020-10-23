%% readVid - Alexandre Caldeira (10/02/2020)
% Lê img/vid e plota melhores retas vistas

% Boas Práticas
clearvars
close all
clc

% Plotar retas vistas em foto:
% path = 'C:\Users\alexa\Dropbox\AuRoRA 2018\Vision\Screenshot_2.png';
% plot_lines(path)

% Plotar retas vistas em foto:
FileName = ['C:\Users\alexa\Documents\MEGA\MEGAsync\NERO_online\Txt_sketches'...
                      '\ICUAS 2020\Bebop_2_2020-02-08T182746-0300_6262D5.mp4'];
v = VideoReader(FileName);
i = 700;
figure(1)
TH = [];
L = 0; th = 0;
lines = [];
n = linspace(0,1,500);
while i < v.NumFrames
frame = read(v,i);
[L,th] = plotLinesOnVid2(frame);
TH = [TH,th];
% lines = [lines,L];
% figure(2)
% hold on
% plot(n.*mean(TH),'b-','LineWidth',1.3)
% plot(n.*(mean(TH)+std(TH)),'r--')
% plot(n.*(mean(TH)-std(TH)),'r--')
% 
% plot(n.*mean(-TH),'k-','LineWidth',1.3)
% plot(n.*(mean(-TH)+std(-TH)),'r--')
% plot(n.*(mean(-TH)-std(-TH)),'r--')
% drawnow

i = i+10;
end
