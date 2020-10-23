classdef TangentialEscape < handle
    properties
        % Properties or Parameters
        pXv
        pDobs
        pFF
        pGamma
        pFlag  % Flags
        pCurve % Perform curve manuever
        pList  % List of obstacle detection
    end
    methods
        function sTE = TangentialEscape()
            sInit(sTE);
            sTE.pList  = [];
            sTE.pFlag  = 0;
            sTE.pCurve = 0;
            sTE.pXv    = [0;0]; % [m]
        end
        
        function sInit(sTE)
            sTE.pDobs  = 1;   % [m]
            sTE.pFF    = 1;
            sTE.pGamma = 0;
        end
        function robot = sFindVirtualGoal(sTE,robot,RangeData)
            [dmin,pos] = min(RangeData(2,:));
            beta = RangeData(1,pos);
            display(beta*180/pi)
            
            if sTE.pCurve
                % Verificar se atingiu o ponto
                if norm(sTE.pXv(1:2) - robot.pPos.X(1:2)) < 0.1
                    sTE.pCurve = 0;
                end
            else
                sTE.pXv(1:2) = robot.pPos.Xd(1:2);                
                if dmin < sTE.pDobs
                    sTE.pDobs = 1; % Increase safety zone
                    
                    % --------------------------------------
                    % Indicativo de novo obstáculo
                    if sTE.pFlag == 0
                        if beta < 0
                            sTE.pFlag = -1;
                        else
                            sTE.pFlag =  1;
                        end
                        % Adicionar ponto de detecção de obstáculo
                        if ~isempty(sTE.pList)
                            OK = 1;
                            for ii = 1:size(sTE.pList,2)
                                if norm(sTE.pList(:,ii)-robot.pPos.X(1:2)) < sTE.pDobs
                                    OK = 0;
                                    break
                                end
                            end
                            if OK
                                sTE.pList(:,end+1) = robot.pPos.X(1:2);
                            end
                        else
                            sTE.pList(:,end+1) = robot.pPos.X(1:2);
                        end
                    end
                    % --------------------------------------
                    
                    theta = atan2(robot.pPos.Xd(2) - robot.pPos.X(2),robot.pPos.Xd(1) - robot.pPos.X(1));
                    rad2deg(theta)
                    alpha = theta - robot.pPos.X(6);
                    if beta < 0   
                        % Obstacle on rigth
                        gamma =  pi/2 - alpha + beta;
                    else
                        % Obstacle on left
                        gamma = -pi/2 - alpha + beta;
                    end
                    % Forgetfulness factor (FF)
                    sTE.pGamma = sTE.pGamma*(1-sTE.pFF) + sTE.pFF*gamma;
                    rad2deg(sTE.pGamma)
                    
                    % Robot-Goal Distance
                    % Conventional movement
                    % sTE.pXv(1:2) = robot.pPos.X(1:2) + [cos(sTE.pGamma) -sin(sTE.pGamma); sin(sTE.pGamma) cos(sTE.pGamma)]*(robot.pPos.Xd(1:2) - robot.pPos.X(1:2));
                    % Cautious movement
                    ang = sTE.pGamma; %-robot.pPos.X(6);
%                     sTE.pXv(1:2) = robot.pPos.X(1:2) + [cos(ang) -sin(ang); sin(ang) cos(ang)]*[cos(theta); sin(theta)]*sTE.pDobs*0.5;
                    sTE.pXv(1:2) = robot.pPos.X(1:2) + [cos(ang) -sin(ang); sin(ang) cos(ang)]*(robot.pPos.Xd(1:2) - robot.pPos.X(1:2));
                    display(sTE.pXv(1:2)')
                    disp('obs....')
                else                    
                    % Executa a curva após obstáculo
                    if sTE.pFlag ~= 0
                        sTE.pCurve = 1;
                        % Determinar ponto de giro
                        if sTE.pFlag > 0
                            ang = robot.pPos.X(6) + pi/4;
                        else
                            ang = robot.pPos.X(6) - pi/4;
                        end
                        sTE.pXv(1:2) = robot.pPos.X(1:2) + sTE.pDobs*[cos(ang);sin(ang)];
                        sTE.pFlag = 0;                                           
                    end
                    sInit(sTE); % Restore values                    
                end
            end
            
            robot.pPos.Xd(1:2) = sTE.pXv(1:2);
        end
    end
end



