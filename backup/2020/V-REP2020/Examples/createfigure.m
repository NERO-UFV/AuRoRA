function  s = createfigure(X1, Y1,tit,xLab,yLab,Leg)
%CREATEFIGURE(X1, Y1)
%  X1:  stem x
%  Y1:  stem y

%  Auto-generated by MATLAB on 07-Aug-2019 17:23:57

% Create figure
figure1 = figure('InvertHardcopy','off','Color',[1 1 1]);

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create stem
s = stem(X1,Y1,'DisplayName',Leg,...
    'MarkerEdgeColor',[0 0.447058826684952 0.74117648601532],...
    'MarkerSize',13,...
    'Marker','.',...
    'Color',[0 0 0]);

% Create ylabel
ylabel(yLab);

% Create xlabel
xlabel(xLab);

% Create title
title(tit);

% Uncomment the following line to preserve the X-limits of the axes
xlim(axes1,[0 184]);
% Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes1,[0 9]);
% Uncomment the following line to preserve the Z-limits of the axes
% zlim(axes1,[-1 1]);
box(axes1,'on');
% Set the remaining axes properties
set(axes1,'FontSize',14,'XGrid','on','YGrid','on');
% Create legend
legend1 = legend(axes1,'show');
% set(legend1,...
%     'Position',[0.778341135214501 0.843169400226223 0.103744147859758 0.0590163918792225],...
%     'FontSize',14);

