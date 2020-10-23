%% UAV with inverted pendulum (inverted dynamics control simulation)
%Clear workspace
clearvars
close all
clc

%Setting up system
%obs.: alpha, beta, phi, theta, psi -> [rad]
S = InvPendulum;
a = 0.1;
b = 0;
%                a     b      ph  th  ps  x   y   z
S.pPos.Q(1:8) = [a     b      0   0   0   0   0   1.5];
S.pPos.Qd(1:8)= [0     0      0   0   0   0   0   1.5];
% disp(S.pPos.Q(6:8))

% Gains 
% kdph = 1;
% kdth = 1;
% kdz  = 1;
% cgains = [-kdph -kdth kdph kdth 1 1 1 kdz; 1 1 1 1 1 1 1 1]*;  %'21/01/19 17:10'
% cgains = [.1 .1 .2 .2 .2 .1 .1 .1; .1 .1 .2 .2 .2 .1 .1 .1]; %'22/01/19 12:02'

% cgains = [-2 -5 2 5 2 .5 2 .5;-15 -1 15 1 2.5  1 20 1]; % Marcos
%          a    b   ph  th  ps  x   y   z  a    b   ph  th  ps  x   y   z
% phg  = 6;
% thg  = 22;
% dphg = 15;
% dthg = 20;
% cgains = [-phg  -thg  phg  thg  22  .5   2  2;-dphg -dthg dphg dthg 20 1  20  1]; % Marcos

% cgains = [-2 -5 2 5 2 .5 2 .5;-15 -1 15 1 2.5  1 20 1]*0.2; % Marcos
% 
cgains = [-.001 -.001 .02  0.1 .02  .0025 .01  .01; ...
          -.1   -.005  .15 0.01 .025 .005  1   .005];  % Marcos editado
% cgains(1,3:6) = cgains(1,3:6);
% cgains(1,8) = cgains(1,8);

% Fz-EqM
%    -1.5842
%    -1.5959
%    -0.0007
%    -0.0010
%    -0.0015
%    -0.2220
%    -0.2192
%    14.5722
%% Plot simulation space
ObjF.Fig(1) = figure(1);
set(ObjF.Fig(1),'units','pix','pos',[300 150 1200/1.3 800/1.3],'PaperPositionMode','auto')
axis image
ObjF.xlim = [-1 1];
ObjF.ylim = [-1 1];
ObjF.zlim = [ 0 3];
xlabel('x'); ylabel('y'); zlabel('z');
axis([ObjF.xlim ObjF.ylim ObjF.zlim])
% view(110,20)
view(3) %default
% view(50,40)
hold on
plot3(S.pPos.Q(6),S.pPos.Q(7),S.pPos.Q(8),'x')
grid on

%//TODO: Pretty plot pendulum

% for k=1:0.1:2
    % Trying gains:
%     cgains = cgains*k*1e-9;

    %% Time and datalog files
    tmax = 30;
    t = tic;
    tp= tic;
    LOG = [];  
    
    %% Simulation loop:
    while toc(t) < tmax
        disp(S.pPos.Q(3:5))
        % Draw drone
        S.mCADplot
        drawnow

        % Draw inverted pendulum
        sa = sin(S.pPos.Q(1));
        sb = sin(S.pPos.Q(2));
        D  = ((1-(sa^2)-(sb^2))^(1/2));
        S.pPos.Xp = [S.pPos.Q(6) + S.pPar.r*sa ;...
                     S.pPos.Q(7) + S.pPar.r*sb ;...
                     S.pPos.Q(8) + S.pPar.r*D     ];
        S.pPos.Xp = real(S.pPos.Xp);
        hold on
        if toc(tp)>0.5
            plot3(S.pPos.Xp(1),S.pPos.Xp(2),S.pPos.Xp(3),'o')
            tp = tic;
        end

%         S.pPar.g = 5;
%         S.pSC.Qr  = S.pPos.Q;
%         S.pSC.dQr = S.pPos.dQ;
    %     disp(sum(S.pSC.Qr-S.k=1:0.2:2pPos.Q))
        % Simulate system dynamics
        S.sInvPendDynamicModel;

        S.pPos.Qd(1:8)= [0 0 0 0 0 0 0 1.5];
%         S.pPos.Q = S.pPos.Qd;
%         S.pPos.Q   = real(S.pPos.Q); % Fix possible problems
%         clc
%         disp('[a b Z]^T')
%         disp(S.pPos.Q([1 2 8]))
%         disp('ddQ:')
%         disp(S.pPos.ddQ)
%         disp('Qtil:')
%         disp(S.pPos.Qtil(8))
%         disp('ddQd:')
%         disp(S.pPos.ddQd)
%         disp(S.pPos.Q(8))
%         disp('__________________')
        % Evaluate errors
        S.pPos.Qtil = S.pPos.Qd - S.pPos.Q;
        S.pPos.dQtil = S.pPos.dQd - S.pPos.dQ;

        % Evaluate control signals
        cInvDynCntrl(S,cgains); % Inverted dynamics control

        % Save data
%         LOG = [LOG [S.pPos.Q; S.pPos.dQ; S.pPos.ddQ; S.pPos.Xp; S.pPos.Qtil; k; toc(t)]];
        LOG = [LOG [S.pPos.Q; S.pPos.dQ; S.pPos.ddQ; S.pPos.Xp; S.pPos.Qtil; toc(t)]];

%         if (abs(S.pPos.Q(8)) > 5)||(S.pPos.Q(1) > 0.35)||(S.pPos.Q(2) > 0.35)||isnan(S.pPos.ddQ(8))||(abs(S.pPos.Q(3)) > 0.5)||(abs(S.pPos.Q(4)) > 0.5)
%             break
%         end

    end         
% end