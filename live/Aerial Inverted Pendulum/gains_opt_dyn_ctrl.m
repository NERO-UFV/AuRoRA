%% UAV with inverted pendulum (inverted dynamics control simulation)
%% Gains selection and optimization
%Clear workspace
clearvars
clc

%Setting up system
%obs.: alpha, beta, phi, theta, psi -> [rad]
S = InvPendulum;
a = 0;
b = 0;
%                a     b      ph  th  ps  x   y   z
S.pPos.Q(1:8)  = [a     b      0   0   0   0   0   1.5];
S.pPos.Qd(1:8) = [0     0      0   0   0   0   0   1.5];
S.pPar.Gains   = [-.001 -.001 .02  0.05 .02  .0025 .01  .01; ...
                  -.1   -.005  .15 0.01 .025 .005  1   .005];  % Marcos editado
S.pPar.Gains(1,3:6) = S.pPar.Gains(1,3:6)*sqrt(2);
S.pPar.Gains(1,8) = S.pPar.Gains(1,8)*0.001;
erro = S.pPos.Qtil(8)+ 999;
count = 0;
%% Plot simulation space
ObjF.Fig(1) = figure(1);
set(ObjF.Fig(1),'units','pix','pos',[300 150 1200/1.3 800/1.3],'PaperPositionMode','auto')
axis image
ObjF.xlim = [-1 1];
ObjF.ylim = [-1 1];
ObjF.zlim = [ 0 3];
xlabel('x'); ylabel('y'); zlabel('z');
axis([ObjF.xlim ObjF.ylim])
view(3)
plot3(S.pPos.Q(6),S.pPos.Q(7),S.pPos.Q(8),'x')
grid on

%//TODO: Pretty plot pendulum!
cgains1 =S.pPar.Gains;
while erro > .5
    cgains = S.pPar.Gains;
    
    %% Time and datalog files
    tmax = 30;
    t    = tic;
    tp   = tic;
    LOG  = [];  
    GLOG = [];
%     try
        %% Simulation loop:
        while toc(t) < tmax
            S.pSC.Ud = zeros(4,1);

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
            plot3(S.pPos.Xp(1),S.pPos.Xp(2),S.pPos.Xp(3),'o')

            % Simulate system dynamics
            S.sInvPendDynamicModel;

            S.pPos.Qd(1:8)= [0 0 0 0 0 0 0 1.5];
            S.pPos.Q   = real(S.pPos.Q); % Fix possible problems

            disp('[a b Z]^T')
            disp(S.pPos.Q([1 2 8]))
            disp('ddQ:')
            disp(S.pPos.ddQ)
            disp('Qtil:')
            disp(S.pPos.Qtil(8))
            disp('ddQd:')
            disp(S.pPos.ddQd)

            % Evaluate errors
            S.pPos.Qtil = S.pPos.Qd - S.pPos.Q;
            S.pPos.dQtil = S.pPos.dQd - S.pPos.dQ;

            % Evaluate control signals
            cInvDynCntrl(S,cgains); % Inverted dynamics control

            % Save data
            LOG  = [LOG [S.pPos.Q; S.pPos.dQ; S.pPos.ddQ; S.pPos.Xp; S.pPos.Qtil; toc(t)]];
            
            % 'Gain quality filter'
            if ((abs(S.pPos.Q(8)) > 3)||(S.pPos.Q(1) > 0.35)||(S.pPos.Q(2) > 0.35))||isnan(S.pPos.Q(8))||isnan(S.pPos.ddQ(8))
                count = count+1;
                break
            end
        end         
%     catch
        %clear variables
        S.pPos.ddQ = zeros(8,1);
        S.pPos.dQ = zeros(8,1);
        S.pPos.Q = zeros(8,1);
        S.pPos.Q(8) = 1.5;
        S.pPar.Gainsa = S.pPar.Gains; %Update previous gainsr.Gains;
        GLOG = [GLOG; [S.pPar.Gains]];
        %New gains:
        for i=1:8
            if isnan(S.pPos.Qtil(i))||isnan(S.pPar.Gains(1,i))
                if S.pPar.Gains(i) > 0
                    S.pPar.Gains(1,i) = cgains1(1,i) - 0.001*count;
                    S.pPar.Gains(2,i) = cgains1(2,i) - 0.001*count;
                else
                    S.pPar.Gains(1,i) = cgains1(1,i) + 0.001*count;
                    S.pPar.Gains(2,i) = cgains1(2,i) + 0.001*count;
                end
            else
                S.pPar.Gains(1,i)  = S.pPar.Gains(1,i) + (normalize(S.pPos.Qtil(i))*.1) + (normalize(S.pPos.dQtil(i))*.05); 
                S.pPar.Gains(2,i)  = S.pPar.Gains(2,i) + (normalize(S.pPos.dQtil(i))*.05); 
            end
        end
        if isnan(S.pPos.Qtil(8))
            erro = 999;
        else
            erro = abs(S.pPos.Qtil(8));
        end
%     end
end