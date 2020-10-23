%% UAV with inverted pendulum (dynamic model simulation)
%Clear workspace
clearvars
close all
clc

%Setting up system
%obs.: alpha, beta, phi, theta, psi -> [rad]
S = InvPendulum;
%                a     b      ph  th  ps  x   y   z
S.pPos.Q(1:8) = [.2    .2      0   0   0   0   0   1.5];

%% Plot simulation space
% ObjF.Fig(1) = figure(1);
% set(ObjF.Fig(1),'units','pix','pos',[300 150 1200/1.3 800/1.3],'PaperPositionMode','auto')
% axis image
% ObjF.xlim = [-.5 .5];
% ObjF.ylim = [-.5 .5];
% ObjF.zlim = [ 0 1];
% xlabel('x'); ylabel('y'); zlabel('z');
% axis([ObjF.xlim ObjF.ylim ObjF.zlim])
% view(45,30)
% view(110,20)
view(3)
grid on

%//TODO: Plot pendulum

%% Time and datalog files
tmax = 30;
t = tic;
tp= tic;
LOG = [];  

%% Simulation loop
while toc(t) < tmax
%     disp(S.pPos.Q)
%     S.pPar.g = 0.005;
    S.sInvPendDynamicModel;
    S.pPos.Q = real(S.pPos.Q);
    S.pPos.Q(8) = 1.5;
    S.pPos.Q(7) = 0;
    S.pPos.Q(6) = 0;
%     disp(S.pPos.Q)
    hold on
    S.mCADplot
    drawnow
%     sa = sin(pendulum.pPos.Q(1));
%     sb = sin(pendulum.pPos.Q(2));
%     D  = ((1-(sa^2)-(sb^2))^(1/2));
%     pendulum.pPos.Xp = [pendulum.pPos.Q(6) + pendulum.pPar.r*sa ;...
%                         pendulum.pPos.Q(7) + pendulum.pPar.r*sb ;...
%                         pendulum.pPos.Q(8) + pendulum.pPar.r*D     ];
    sa = sin(S.pPos.Q(1));
    sb = sin(S.pPos.Q(2));
    D  = ((1-(sa^2)-(sb^2))^(1/2));
    S.pPos.Xp = [S.pPos.Q(6) + S.pPar.r*sa ;...
                 S.pPos.Q(7) + S.pPar.r*sb ;...
                 S.pPos.Q(8) + S.pPar.r*D     ];
   S.pPos.Xp = real(S.pPos.Xp);
   disp(S.pPos.Xp)
             
    hold on
%     if toc(tp)>1
        plot3(S.pPos.Xp(1),S.pPos.Xp(2),S.pPos.Xp(3),'o')
%         tp = tic;
%     end
%     set(ObjF.Fig(1),'units','pix','pos',[300 150 1200/1.3 800/1.3],'PaperPositionMode','auto')
    LOG = [LOG [S.pPos.Q; S.pPos.dQ; S.pPos.ddQ; S.pPos.Xp; toc(t)]];
    
end                      