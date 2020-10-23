close all; clear all; clc;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% PLOT DOS RESULTADOS DO EXPERIMENTO

load '202001302257_AgoraFoi.mat'
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%                          REFINAR OS DADOS                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

TEMPOS_REAIS = [30 37.5 45 60 75 90 97.5 128];
DRONES = [1 1 0 0; 1 1 0 0; 0 1 0 0; 0 1 0 0; 0 1 0 0; 1 1 0 0; 1 1 0 0; 1 1 0 0];
F_ATIVAS = [1 1 0; 0 1 0; 0 1 0; 0 1 0; 0 1 0; 0 1 1; 0 1 1; 0 1 1];
F_ROBO{1} = [1 1 3; 1 0 0; 1 0 0; 1 0 0; 1 0 0; 1 0 0; 1 0 0; 1 0 0];
F_ROBO{2} = [2 2 4; 2 2 1; 2 2 1; 2 2 1; 2 2 1; 2 1 4; 2 1 4; 2 1 4];
F_ROBO{3} = [3 0 0; 3 0 0; 3 0 0; 3 0 0; 3 0 0; 3 2 3; 3 2 3; 3 2 3];
FORMACAO = 1;

SIMULACAO = 0;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% PLOT DOS RESULTADOS DO EXPERIMENTO

%           ETAPAS
%
%           1               2--5            6--8            9--11
%           PARTE           DRONES          FORMACAO        PIONEER
%           
%           12--14          15              16
%           TF{1}.pPar.R    toc(T)          toc(T_ALFA)

F = 1;

%           DADOS
%
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
%           TF.pPos.Qd'     TF.pPos.Q'      TF.pPos.dQd'    TF.pPos.dQr'
% 
%           143 -- 151      152 -- 160      161 -- 169
%           TF.pPos.Xd'     TF.pPos.X'      TF.pPos.dXr'
% 
%           170             171'
%           toc(T)          toc(T_ALFA)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%                              SIMULAÇÃO                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if SIMULACAO == 1
P{1} = Pioneer3DX;
P{2} = Pioneer3DX;
P{3} = Pioneer3DX;

figure(F)
F = F + 1;
% vert = [-1.5 -2 0;
%         1.5  -2 0;
%         1.5  2  0;
%         -1.5 2  0;
%         1.5  -2 1;
%         1.5  2  1;
%         -1.5 2  1];

vert = [-2.5 -2 0;
        2.5  -2 0;
        2.5  2  0;
        -2.5 2  0;
        2.5  -2 2;
        2.5  2  2;
        -2.5 2  2];

fac = [1 2 3 4;2 3 6 5;3 4 7 6];
% patch('Vertices',vert,'Faces',fac,...
%     'FaceColor',0.8*[1 1 1],'FaceAlpha',0.3,'LineWidth',1.6)
% hold on
% grid on
% axis equal
% xlabel('$\textbf{X [m]}$','interpreter','latex','FontWeight','bold');
% ylabel('$\textbf{Y [m]}$','interpreter','latex','FontWeight','bold');
% zlabel('$\textbf{Z [m]}$','interpreter','latex','FontWeight','bold');
% view(3)

PT_TEMP = 0;
LGD = [];
TTL = {};
% PPT = [3 4 5 6 9 10 11 12];
% PPT = [4 5 6 10 12 16 17 18];
PPT = [1 2 3 4 6 7 8 9];

for jj = 1:size(TEMPOS_REAIS,2)
A{1} = ArDrone;
A{2} = ArDrone;
subplot(3,3,PPT(jj))
patch('Vertices',vert,'Faces',fac,...
    'FaceColor',0.8*[1 1 1],'FaceAlpha',0.3,'LineWidth',1.6)
hold on
grid on
% axis equal
xlabel('$\textbf{X [m]}$','interpreter','latex','FontWeight','bold');
ylabel('$\textbf{Y [m]}$','interpreter','latex','FontWeight','bold');
zlabel('$\textbf{Z [m]}$','interpreter','latex','FontWeight','bold');
view(3)
title(['$\textbf{Stage ' num2str(jj) '}$'],'interpreter','latex','FontWeight','bold')
T = TEMPOS_REAIS(jj);
if jj == 1
    T_A = 0;
else
    T_A = TEMPOS_REAIS(jj-1);
end
PT = sum(TEMPOS_REAIS <= T);
PT_DADOS = find(and((DADOS{2}(:,end) < T),(DADOS{2}(:,end) > T_A)));
COLOR = {[0 1 0] [1 0 0] [0 0 1]};
% ID = [2 1 3];
for ii = find(F_ATIVAS(PT,:)==1)
    if ii == 3 && PT_TEMP == 0
        PT_TEMP = PT_DADOS(1);
    end
    if ii == 3
        PT_DADOS = PT_DADOS + 1 - PT_TEMP;
    end
    LINE_P{ii,1} = plot3(DADOS{ii}(PT_DADOS,12+1),...
                       DADOS{ii}(PT_DADOS,12+2),...
                       DADOS{ii}(PT_DADOS,12+3),...
                       'Color',COLOR{ii},'LineWidth',1.6);
    LINE_P{ii,2} = ['Pioneer3DX ' num2str(ii) ' Route'];
    
    P{ii}.rSetPose([DADOS{ii}(PT_DADOS(end),12+1);
                    DADOS{ii}(PT_DADOS(end),12+2);
                    DADOS{ii}(PT_DADOS(end),12+3);
                    DADOS{ii}(PT_DADOS(end),12+6)]')
    P{ii}.mCADplot(1,COLOR{ii})
    
    LINE_A1{ii,1} = plot3(DADOS{ii}(PT_DADOS,38+1),...
                        DADOS{ii}(PT_DADOS,38+2),...
                        DADOS{ii}(PT_DADOS,38+3),...
                        'LineStyle','--','Color',COLOR{ii},'LineWidth',1.6,'AlignVertexCenters','on');
    LINE_A1{ii,2} = ['Bebop ' num2str(F_ROBO{ii}(PT,2)) ' Route'];
    
    A{F_ROBO{ii}(PT,2)}.pPos.X([1:3 6]) = [DADOS{ii}(PT_DADOS(end),38+1);
                                           DADOS{ii}(PT_DADOS(end),38+2);
                                           DADOS{ii}(PT_DADOS(end),38+3);
                                           DADOS{ii}(PT_DADOS(end),38+6)];
    A{F_ROBO{ii}(PT,2)}.mCADcolor(COLOR{ii});
    A{F_ROBO{ii}(PT,2)}.mCADplot(1);
    
    LGD = [LGD LINE_P{ii,1} LINE_A1{ii,1}];
    TTL{1,end+1} = LINE_P{ii,2};
    TTL{1,end+1} = LINE_A1{ii,2};
    if DRONES(PT,F_ROBO{ii}(PT,3)) == 1
        LINE_A2{ii,1} = plot3(DADOS{ii}(PT_DADOS,78+1),...
                      DADOS{ii}(PT_DADOS,78+2),...
                      DADOS{ii}(PT_DADOS,78+3),...
                      'LineStyle','--','Color',COLOR{ii},'LineWidth',1.6,'AlignVertexCenters','on');
        LINE_A2{ii,2} = ['Bebop ' num2str(F_ROBO{ii}(PT,3)) ' Route'];
        
        A{F_ROBO{ii}(PT,3)}.pPos.X([1:3 6]) = [DADOS{ii}(PT_DADOS(end),78+1);
                                               DADOS{ii}(PT_DADOS(end),78+2);
                                               DADOS{ii}(PT_DADOS(end),78+3);
                                               DADOS{ii}(PT_DADOS(end),78+6)];
        A{F_ROBO{ii}(PT,3)}.mCADcolor(COLOR{ii});
        A{F_ROBO{ii}(PT,3)}.mCADplot(1);
        LGD = [LGD LINE_A2{ii,1}];
        TTL{1,end+1} = LINE_A2{ii,2};
    elseif jj == 3 || jj == 4 || jj == 5
        A{1}.pPos.X(1:3) = [DADOS{ii}(PT_DADOS(end),12+1);
                            DADOS{ii}(PT_DADOS(end),12+2);
                            0.3225];
        A{1}.mCADcolor(COLOR{2});
        A{1}.mCADplot(1);
    end
end
end

% legend(LGD,TTL,...
%        'interpreter','latex',...
%        'orientation','horizontal',...
%        'location','northoutside');

return

vert = [-3 -2 0;
        3  -2 0;
        3  2  0;
        -3 2  0;
        3  -2 2;
        3  2  2;
        -3 2  2];

fac = [1 2 3 4;2 3 6 5;3 4 7 6];

P{1} = Pioneer3DX;
P{2} = Pioneer3DX;
P{3} = Pioneer3DX;

PT_TEMP = 0;
LGD = [];
TTL = {};
% PPT = [1 2 7 8];
PPT = [1 2 3 7 8 9 13 14 15];

for jj = 1:size(TEMPOS_REAIS,2)
A{1} = ArDrone;
A{2} = ArDrone;
% subplot(3,6,PPT)
patch('Vertices',vert,'Faces',fac,...
    'FaceColor',0.8*[1 1 1],'FaceAlpha',0.3,'LineWidth',1.6)
hold on
grid on
% axis equal
xlabel('$\textbf{X [m]}$','interpreter','latex','FontWeight','bold');
ylabel('$\textbf{Y [m]}$','interpreter','latex','FontWeight','bold');
zlabel('$\textbf{Z [m]}$','interpreter','latex','FontWeight','bold');
view(3)
title(['$\textbf{Overview}$'],'interpreter','latex','FontWeight','bold')
T = TEMPOS_REAIS(jj);
if jj == 1
    T_A = 0;
else
    T_A = TEMPOS_REAIS(jj-1);
end
PT = sum(TEMPOS_REAIS <= T);
PT_DADOS = find(and((DADOS{2}(:,end) < T),(DADOS{2}(:,end) > T_A)));
COLOR = {[0 1 0] [1 0 0] [0 0 1]};
% ID = [2 1 3];
for ii = find(F_ATIVAS(PT,:)==1)
    if ii == 3 && PT_TEMP == 0
        PT_TEMP = PT_DADOS(1);
    end
    if ii == 3
        PT_DADOS = PT_DADOS + 1 - PT_TEMP;
    end
    LINE_P{ii,1} = plot3(DADOS{ii}(PT_DADOS,12+1),...
                       DADOS{ii}(PT_DADOS,12+2),...
                       DADOS{ii}(PT_DADOS,12+3),...
                       'Color',COLOR{ii},'LineWidth',1.6);
    LINE_P{ii,2} = ['Pioneer3DX ' num2str(ii) ' Route'];
    
    P{ii}.rSetPose([DADOS{ii}(PT_DADOS(end),12+1);
                    DADOS{ii}(PT_DADOS(end),12+2);
                    DADOS{ii}(PT_DADOS(end),12+3);
                    DADOS{ii}(PT_DADOS(end),12+6)]')
    P{ii}.mCADplot(1,COLOR{ii})
    
    LINE_A1{ii,1} = plot3(DADOS{ii}(PT_DADOS,38+1),...
                        DADOS{ii}(PT_DADOS,38+2),...
                        DADOS{ii}(PT_DADOS,38+3),...
                        'LineStyle',':','Color',COLOR{ii},'LineWidth',1.6,'AlignVertexCenters','on');
    LINE_A1{ii,2} = ['Bebop ' num2str(F_ROBO{ii}(PT,2)) ' Route'];
    
    A{F_ROBO{ii}(PT,2)}.pPos.X([1:3 6]) = [DADOS{ii}(PT_DADOS(end),38+1);
                                           DADOS{ii}(PT_DADOS(end),38+2);
                                           DADOS{ii}(PT_DADOS(end),38+3);
                                           DADOS{ii}(PT_DADOS(end),38+6)];
    A{F_ROBO{ii}(PT,2)}.mCADcolor(COLOR{ii});
    A{F_ROBO{ii}(PT,2)}.mCADplot(1);
    
    LGD = [LGD LINE_P{ii,1} LINE_A1{ii,1}];
    TTL{1,end+1} = LINE_P{ii,2};
    TTL{1,end+1} = LINE_A1{ii,2};
    if DRONES(PT,F_ROBO{ii}(PT,3)) == 1
        LINE_A2{ii,1} = plot3(DADOS{ii}(PT_DADOS,78+1),...
                      DADOS{ii}(PT_DADOS,78+2),...
                      DADOS{ii}(PT_DADOS,78+3),...
                      'LineStyle',':','Color',COLOR{ii},'LineWidth',1.6,'AlignVertexCenters','on');
        LINE_A2{ii,2} = ['Bebop ' num2str(F_ROBO{ii}(PT,3)) ' Route'];
        
        A{F_ROBO{ii}(PT,3)}.pPos.X([1:3 6]) = [DADOS{ii}(PT_DADOS(end),78+1);
                                               DADOS{ii}(PT_DADOS(end),78+2);
                                               DADOS{ii}(PT_DADOS(end),78+3);
                                               DADOS{ii}(PT_DADOS(end),78+6)];
        A{F_ROBO{ii}(PT,3)}.mCADcolor(COLOR{ii});
        A{F_ROBO{ii}(PT,3)}.mCADplot(1);
        LGD = [LGD LINE_A2{ii,1}];
        TTL{1,end+1} = LINE_A2{ii,2};
    end
end
end   
   
% subplot(3,6,11)
TTL1 = {TTL{1}, TTL{2}, TTL{3}, TTL{4}, TTL{16}, TTL{17}};   
lgd = legend(LGD([3 4 1 2 16 17]),TTL1,...
       'interpreter','latex',...
       'orientation','horizontal',...
       'FontSize',10,...
       'location','northoutside');
lgd.NumColumns = 1;
title(lgd,'$\textbf{Legend}$','interpreter','latex','FontWeight','bold')
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%                              FORMAÇÕES                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% figure(F)
% F = F + 1;
% left_color = [0 0 0];
% right_color = [0 0 0];
% set(fig,'defaultAxesColorOrder',[left_color; right_color]);
% hold on
% grid on
% yyaxis left
% %%%%%%%%%%%%
% yyaxis right

F_A{1} = find(F_ATIVAS(:,1)==1);
F_A{2} = find(F_ATIVAS(:,2)==1);
F_A{3} = find(F_ATIVAS(:,3)==1);

CNew = {'-g','-r','-b'};

for ii = [2 1 3]
T = TEMPOS_REAIS(F_A{ii}(end));
if F_A{ii}(1) == 1
    T_A = 0;
else
    T_A = TEMPOS_REAIS(F_A{ii}(1)-1);
end
F_A{ii} = find(and((DADOS{2}(:,end) < T),(DADOS{2}(:,end) > T_A)));
if F_A{ii}(1) ~= 1
    F_A{ii} = F_A{ii} + 1 - F_A{ii}(1);
end
% Posição
fig = figure(F);
% F = F + 1;
left_color = [0 0 0];
right_color = [0 0 0];
set(fig,'defaultAxesColorOrder',[left_color; right_color]);
subplot(3,3,1)
hold on
grid on
plot(DADOS{ii}(F_A{ii},end),DADOS{ii}(F_A{ii},106+1)-DADOS{ii}(F_A{ii},115+1),...
     CNew{ii},'LineWidth',1.6);
subplot(3,3,4)
hold on
grid on
plot(DADOS{ii}(F_A{ii},end),DADOS{ii}(F_A{ii},106+2)-DADOS{ii}(F_A{ii},115+2),...
     CNew{ii},'LineWidth',1.6);
subplot(3,3,7)
hold on
grid on
plot(DADOS{ii}(F_A{ii},end),DADOS{ii}(F_A{ii},106+3)-DADOS{ii}(F_A{ii},115+3),...
     CNew{ii},'LineWidth',1.6);

% Orientação
set(fig,'defaultAxesColorOrder',[left_color; right_color]);
subplot(3,3,2)
hold on
grid on
plot(DADOS{ii}(F_A{ii},end),DADOS{ii}(F_A{ii},106+4)-DADOS{ii}(F_A{ii},115+4),...
     CNew{ii},'LineWidth',1.6);
subplot(3,3,5)
hold on
grid on
plot(DADOS{ii}(F_A{ii},end),DADOS{ii}(F_A{ii},106+5)-DADOS{ii}(F_A{ii},115+5),...
     CNew{ii},'LineWidth',1.6);
subplot(3,3,8)
hold on
grid on
plot(DADOS{ii}(F_A{ii},end),DADOS{ii}(F_A{ii},106+6)-DADOS{ii}(F_A{ii},115+6),...
     CNew{ii},'LineWidth',1.6);

 
% Postura
set(fig,'defaultAxesColorOrder',[left_color; right_color]);
subplot(3,3,3)
hold on
grid on
plot(DADOS{ii}(F_A{ii},end),DADOS{ii}(F_A{ii},106+7)-DADOS{ii}(F_A{ii},115+7),...
     CNew{ii},'LineWidth',1.6);
subplot(3,3,6)
hold on
grid on
plot(DADOS{ii}(F_A{ii},end),DADOS{ii}(F_A{ii},106+8)-DADOS{ii}(F_A{ii},115+8),...
     CNew{ii},'LineWidth',1.6);
subplot(3,3,9)
hold on
grid on
plot(DADOS{ii}(F_A{ii},end),DADOS{ii}(F_A{ii},106+9)-DADOS{ii}(F_A{ii},115+9),...
     CNew{ii},'LineWidth',1.6);
end

 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%                                BEBOPS                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

B_A{1} = find(DRONES(:,1)==1);
B_A{2} = find(DRONES(:,2)==1);

BEBOP{1} = [];
BEBOP{2} = [];

for jj = 1:3
    LINHAS_TEMP = find(F_ROBO{jj}(sum(TEMPOS_REAIS<=DADOS{jj}(:,end),2)+1,2) == 1);
    if ~isempty(LINHAS_TEMP)
        BEBOP{1}(LINHAS_TEMP,:) = DADOS{jj}(LINHAS_TEMP,[27:66 142+(4:6) 151+(4:6) 160+(4:6) 6 171]);
    end
    
    LINHAS_TEMP = find(and(F_ROBO{jj}(sum(TEMPOS_REAIS<=DADOS{jj}(:,end),2)+1,3) == 1,DRONES(sum(TEMPOS_REAIS<=DADOS{jj}(:,end),2)+1,1) == 1));
    if ~isempty(LINHAS_TEMP)
        BEBOP{1}(LINHAS_TEMP,:) = DADOS{jj}(LINHAS_TEMP,[67:106 142+(7:9) 151+(7:9) 160+(7:9) 6 171]);
    end
    if jj < 3
        LINHAS_TEMP = find(F_ROBO{jj}(sum(TEMPOS_REAIS<=DADOS{jj}(:,end),2)+1,2) == 2);
        if ~isempty(LINHAS_TEMP)
            BEBOP{2}(LINHAS_TEMP,:) = DADOS{jj}(LINHAS_TEMP,[27:66 142+(4:6) 151+(4:6) 160+(4:6) 6 171]);
        end
    else
        LINHAS_TEMP = find(F_ROBO{jj}(sum(TEMPOS_REAIS<=DADOS{jj}(:,end),2)+1,2) == 2);
        if ~isempty(LINHAS_TEMP)
            BEBOP{2}(LINHAS_TEMP+size(BEBOP{2},1),:) = DADOS{jj}(LINHAS_TEMP,[27:66 142+(4:6) 151+(4:6) 160+(4:6) 6 171]);
        end
    end
end


for ii = 2
% Posição
fig = figure(F);
F = F + 1;
left_color = [0 0 0];
right_color = [0 0 0];
set(fig,'defaultAxesColorOrder',[left_color; right_color]);
subplot(3,2,1)
hold on
grid on
plot(BEBOP{ii}(:,end),BEBOP{ii}(:,40+1)-BEBOP{ii}(:,12+1),...
     '-r','LineWidth',1.6);
subplot(3,2,3)
hold on
grid on
% plot([BEBOP{ii}(1,end) BEBOP{ii}(end,end)],[0 0],...
%      '-g','LineWidth',1.0);
plot(BEBOP{ii}(:,end),BEBOP{ii}(:,40+2)-BEBOP{ii}(:,12+2),...
     '-r','LineWidth',1.6);
subplot(3,2,5)
hold on
grid on
% plot([BEBOP{ii}(1,end) BEBOP{ii}(end,end)],[0 0],...
%      '-g','LineWidth',1.0);
plot(BEBOP{ii}(:,end),BEBOP{ii}(:,40+3)-BEBOP{ii}(:,12+3),...
     '-r','LineWidth',1.6);
% subplot(4,2,7)
% hold on
% grid on
% % plot([BEBOP{ii}(1,end) BEBOP{ii}(end,end)],[0 0],...
% %      '-g','LineWidth',1.0);
% plot(BEBOP{ii}(:,end),BEBOP{ii}(:,end-1)-BEBOP{ii}(:,12+6),...
%      '-r','LineWidth',1.6);
end

B_TEMP = BEBOP{1};
B_T = find(sum(BEBOP{1},2)==0);
BEBOP1{1} = BEBOP{1}(1:(B_T(1)-1),:);
BEBOP1{2} = BEBOP{1}((B_T(end)+1):end,:);
HATCH = [BEBOP{2}(B_T(1),end) BEBOP{2}(B_T(1),end) BEBOP{2}(B_T(end),end) BEBOP{2}(B_T(end),end);
         -1 1 1 -1];
     
% fig = figure(F);
% F = F + 1;
% left_color = [0 0 0];
% right_color = [0 0 0];
% set(fig,'defaultAxesColorOrder',[left_color; right_color]);


for ii = 1:2
% Posição
subplot(3,2,2)
hold on
grid on
% plot([BEBOP1{ii}(1,end) BEBOP1{ii}(end,end)],[0 0],...
%      '-g','LineWidth',1.0);
if ii == 1
PPP{3} = plot(BEBOP1{ii}(:,end),BEBOP1{ii}(:,40+1)-BEBOP1{ii}(:,12+1),...
     '-g','LineWidth',1.6);
else
PPP{3} = plot(BEBOP1{ii}(:,end),BEBOP1{ii}(:,40+1)-BEBOP1{ii}(:,12+1),...
     '-b','LineWidth',1.6);
end
subplot(3,2,4)
hold on
grid on
% plot([BEBOP1{ii}(1,end) BEBOP1{ii}(end,end)],[0 0],...
%      '-g','LineWidth',1.0);
if ii == 1
plot(BEBOP1{ii}(:,end),BEBOP1{ii}(:,40+2)-BEBOP1{ii}(:,12+2),...
     '-g','LineWidth',1.6);
else
plot(BEBOP1{ii}(:,end),BEBOP1{ii}(:,40+2)-BEBOP1{ii}(:,12+2),...
     '-b','LineWidth',1.6);
end
subplot(3,2,6)
hold on
grid on
% plot([BEBOP1{ii}(1,end) BEBOP1{ii}(end,end)],[0 0],...
%      '-g','LineWidth',1.0);
if ii == 1
plot(BEBOP1{ii}(:,end),BEBOP1{ii}(:,40+3)-BEBOP1{ii}(:,12+3),...
     '-g','LineWidth',1.6);
else
plot(BEBOP1{ii}(:,end),BEBOP1{ii}(:,40+3)-BEBOP1{ii}(:,12+3),...
     '-b','LineWidth',1.6);
end
% subplot(4,2,8)
% hold on
% grid on
% % plot([BEBOP1{ii}(1,end) BEBOP1{ii}(end,end)],[0 0],...
% %      '-g','LineWidth',1.0);
% if ii == 1
% plot(BEBOP1{ii}(:,end),BEBOP1{ii}(:,end-1)-BEBOP1{ii}(:,12+6),...
%      '-g','LineWidth',1.6);
% else
% plot(BEBOP1{ii}(:,end),BEBOP1{ii}(:,end-1)-BEBOP1{ii}(:,12+6),...
%      '-b','LineWidth',1.6);
% end
end

subplot(3,2,2)
axis([0 128 -.2 .2])
hold on
grid on
% yyaxis left
ANGLE = 30;
SPACING = 5;
A = patch(HATCH(1,:),HATCH(2,:)*.2,'k','LineStyle','none');
hatchfill(A,'single',ANGLE,SPACING,'none');
subplot(3,2,4)
axis([0 128 -.2 .2])
hold on
grid on
% yyaxis left
A = patch(HATCH(1,:),HATCH(2,:)*.2,'k','LineStyle','none');
hatchfill(A,'single',ANGLE,SPACING,'none');
subplot(3,2,6)
axis([0 128 -.2 .2])
hold on
grid on
% yyaxis left
A = patch(HATCH(1,:),HATCH(2,:)*.2,'k','LineStyle','none');
hatchfill(A,'single',ANGLE,SPACING,'none');
% subplot(3,2,8)
% axis([0 128 -.2 .2])
% hold on
% grid on
% % yyaxis left
% A = patch(HATCH(1,:),HATCH(2,:)*.2,'k','LineStyle','none');
% hatchfill(A,'single',ANGLE,SPACING,'none');
%%
%690
%510
% figure(4)
% subplot(3,3,1)
% ylabel('$\tilde{\mathbf{x}}_{F}^{\#n} \textbf{ [m]}$','interpreter','latex','FontWeight','bold')
% % axis([0 128 -.2 .2])
% % xticks([0 40 80 120])
% % axis([0 40 -.1 .1])
% axis([0 128 -.2 .2])
% subplot(3,3,4)
% % legend(['Formation $\#1$ error'],'interpreter','latex','orientation','horizontal','location','northoutside');
% ylabel('$\tilde{\mathbf{y}}_{F}^{\#n} \textbf{ [m]}$','interpreter','latex','FontWeight','bold')
% axis([0 128 -.2 .2])
% subplot(3,3,7)
% ylabel('$\tilde{\mathbf{z}}_{F}^{\#n} \textbf{ [m]}$','interpreter','latex','FontWeight','bold')
% axis([0 128 -.2 .2])
% xlabel('$\textbf{Time} \textbf{ [s]}$','interpreter','latex','FontWeight','bold')
% subplot(3,3,2)
% ylabel('$\tilde{\mathbf{\theta}}_{F}^{\#n} \textbf{ [rad]}$','interpreter','latex','FontWeight','bold')
% axis([0 128 -.2 .2])
% subplot(3,3,5)
% ylabel('$\tilde{\mathbf{\phi}}_{F}^{\#n} \textbf{ [rad]}$','interpreter','latex','FontWeight','bold')
% axis([0 128 -.2 .2])
% subplot(3,3,8)
% ylabel('$\tilde{\mathbf{\psi}}_{F}^{\#n} \textbf{ [rad]}$','interpreter','latex','FontWeight','bold')
% axis([0 128 -.2 .2])
% xlabel('$\textbf{Time} \textbf{ [s]}$','interpreter','latex','FontWeight','bold')
% subplot(3,3,3)
% ylabel('$\tilde{\mathbf{p}}_{F}^{\#n} \textbf{ [m]}$','interpreter','latex','FontWeight','bold')
% axis([0 128 -.2 .2])
% subplot(3,3,6)
% ylabel('$\tilde{\mathbf{q}}_{F}^{\#n} \textbf{ [m]}$','interpreter','latex','FontWeight','bold')
% axis([0 128 -.2 .2])
% subplot(3,3,9)
% ylabel('$\tilde{\mathbf{\beta}}_{F}^{\#n} \textbf{ [rad]}$','interpreter','latex','FontWeight','bold')
% axis([0 128 -.2 .2])
% xlabel('$\textbf{Time} \textbf{ [s]}$','interpreter','latex','FontWeight','bold')
%%%
figure(5)
subplot(3,2,1)
ylabel('$\tilde{\mathbf{x}}_{UAV}^{\#1} \textbf{ [m]}$','interpreter','latex','FontWeight','bold')
axis([0 128 -.2 .2])
xticks([0 20 40 60 80 100 120])
subplot(3,2,3)
ylabel('$\tilde{\mathbf{y}}_{UAV}^{\#1} \textbf{ [m]}$','interpreter','latex','FontWeight','bold')
axis([0 128 -.2 .2])
xticks([0 20 40 60 80 100 120])
subplot(3,2,5)
ylabel('$\tilde{\mathbf{z}}_{UAV}^{\#1} \textbf{ [m]}$','interpreter','latex','FontWeight','bold')
axis([0 128 -.2 .2])
xticks([0 20 40 60 80 100 120])
% subplot(4,2,7)
% ylabel('$\tilde{\mathbf{\psi}}_{UAV}^{\#1} \textbf{ [rad]}$','interpreter','latex','FontWeight','bold')
% axis([0 128 -.2 .2])
% xticks([0 20 40 60 80 100 120])
xlabel('$\textbf{Time} \textbf{ [s]}$','interpreter','latex','FontWeight','bold')
subplot(3,2,2)
ylabel('$\tilde{\mathbf{x}}_{UAV}^{\#2} \textbf{ [m]}$','interpreter','latex','FontWeight','bold')
axis([0 128 -.2 .2])
xticks([0 20 40 60 80 100 120])
subplot(3,2,4)
ylabel('$\tilde{\mathbf{y}}_{UAV}^{\#2} \textbf{ [m]}$','interpreter','latex','FontWeight','bold')
axis([0 128 -.2 .2])
xticks([0 20 40 60 80 100 120])
subplot(3,2,6)
ylabel('$\tilde{\mathbf{z}}_{UAV}^{\#2} \textbf{ [m]}$','interpreter','latex','FontWeight','bold')
axis([0 128 -.2 .2])
xticks([0 20 40 60 80 100 120])
% subplot(4,2,8)
% ylabel('$\tilde{\mathbf{\psi}}_{UAV}^{\#2} \textbf{ [rad]}$','interpreter','latex','FontWeight','bold')
% axis([0 128 -.2 .2])
% xticks([0 20 40 60 80 100 120])
xlabel('$\textbf{Time} \textbf{ [s]}$','interpreter','latex','FontWeight','bold')

%%
h = figure(1);
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
txt1 = 'uavsarrumado';
print(h,txt1,'-dpdf','-r0')
