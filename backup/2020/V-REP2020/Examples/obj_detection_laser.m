%% Clean screen
clear all
close all
clc

%% Create the constructor Class
V = VREP;

%% Connect Matlab to V-Rep
V.vConnect;

%% Get handle tags from Pioneer
V.vHandle('Pioneer_p3dx');
%% Set Velocity
Ud = [0;0];
V.vSendControlSignals(Ud,1);
%% Get laser data
pause(.5)
Map = V.vGetLaserData(1);
x = Map(:,1);
y = Map(:,2);
d = Map(:,3);
d = inverte(d);

%% Plot r
f1 = figure(1);
r = stem(1:size(d),d);

%% Plot s
f2 = figure(2);
for ii=2:size(d)
    s(ii) = stem(ii,d(ii)-d(ii-1));
    set(s(ii),'Color',[0,0,1])
%     set(s(ii),'LineStyle','none') %no line
    set(s(ii),'LineWidth',1)
    set(s(ii),'MarkerFaceColor',[0 0.447058826684952 0.74117648601532])
    hold on
end

%% Plot f
% plot()

V.vDisconnect