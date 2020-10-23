  
% Criando uma variável para representar o Robô
P = Pioneer3DX;

%% Get Destination point and Robot Position
Goal = [1 1.5];
P.pPar.ti=tic;
rb = OPT.RigidBody;
P = getOptData(rb,P);

Curve = H.BezierCurve([P.pPos.X(1) 2 Goal(1);P.pPos.X(2) -2 Goal(2)]);
path.n = length(Curve(1,:));
[path.min,path.k] = min(sqrt((P.pPos.X(1) - Curve(1,:)).^2 + (P.pPos.X(2) - Curve(2,:)).^2));

if path.k == 1
    path.k = path.k + 1;
    path.beta = atan2((Curve(2,path.k) - Curve(2,path.k-1)),(Curve(1,path.k) - Curve(1,path.k-1)));
end


P.pPos.Xd([1 2]) = [Curve(1,path.k); Curve(2,path.k)];

vmax = .35;





%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
P.pFlag.EmergencyStop = 0;
figure(2)
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);



                     
                     
%% Figure
figure(1)
hold on
axis([-3.5,3.5,-3.5,3.5])

t_amostragem = 0.1;
tmax = 40;
t_loop = tic;
t= tic;
Dados = [];
                     
k =2;
while toc(t)<tmax

        if toc(t_loop) > t_amostragem
            t_loop = tic;
            
            
            %% Trajetória desejada
            %% Caminho
            
            [path.min,path.k]= min(sqrt((P.pPos.X(1) - Curve(1,:)).^2 + (P.pPos.X(2) - Curve(2,:)).^2));
            
            
            %%
            if path.min < 0.1 && path.k < path.n
                path.k = path.k + 1;
                disp(path.k)
                path.betaA = path.beta;
                path.beta = atan2((Curve(2,path.k) - Curve(2,path.k-1)),(Curve(1,path.k) - Curve(1,path.k-1)));
                path.dbeta = path.beta - path.betaA;
                
                P.pPos.Xd([7 8]) = ...
                    ([vmax/(1 + k*abs(path.dbeta))*cos(path.beta);
                    vmax/(1 + k*abs(path.dbeta))*sin(path.beta)]);
                
                
        
            end
            
            if path.k == path.n
                P.pPos.Xd([7 8]) = [0;0];
            end
            
            
            
            
            %% Set Desired Position
            P.pPos.Xd([1 2]) = [Curve(1,path.k); Curve(2,path.k)];
            
            % Get Position and Velocity from Optitrack
            rb = OPT.RigidBody;
            P = getOptData(rb,P);
            

            % Control System
            K=[cos(P.pPos.X(6)) -0.15*sin(P.pPos.X(6)); sin(P.pPos.X(6)) 0.15*cos(P.pPos.X(6))];   

            P.pPos.Xtil = P.pPos.Xd([1 2])-P.pPos.X([1 2]);
            
%             P.pPos.Xd([7 8])= sign(P.pPos.Xtil([1 2])).*P.pPos.Xd([7 8]);
            
            P.pSC.Ud = K\(P.pPos.Xd([7 8])+0.8*P.pPos.Xtil([1 2]));
            
          
            
            % Send Control Signal to Pioneer
            cmd_vel.Linear.X = P.pSC.Ud(1);
            cmd_vel.Angular.Z = P.pSC.Ud(2);
            send(pub,cmd_vel)

            % Trace
            
            Dados=[Dados; [path.beta P.pPos.Xd([7 8])' P.pPos.X([1 2])' P.pPos.Xtil([1 2])' P.pSC.Ud' toc(t)]];
            
            
            try
                delete(h)
            end
            figure(1)
            h = plot(Dados(:,1),Dados(:,2));
            drawnow
            
            
            %% EMERGÊNCIA
            figure(2)
            drawnow
            if btnEmergencia == 1
                P.pFlag.EmergencyStop = 1;
            end
            
            if btnEmergencia ~= 0 || P.pFlag.EmergencyStop ~= 0
                disp('Pioneer stopping by  ');
                
                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg
                    P.pSC.Ud = [0 0]';
                    cmd_vel.Linear.X = P.pSC.Ud(1);
                    cmd_vel.Angular.Z = P.pSC.Ud(2);
                    send(pub,cmd_vel)
                    
                end
                break;
            end
            
            
            
        end
        
end
% 
% Ud = [0; 0];
% cmd_vel.Linear.X = Ud(1);
% cmd_vel.Angular.Z =Ud(2);
% send(pub,cmd_vel)