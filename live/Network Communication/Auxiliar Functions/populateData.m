function Robot = populateData(Rede,Robot)
% This function assign netdatashare data to robot.
% Inputs: NetDataShare class and robot class
% output: robot (with populated variables)

type = class(Robot);
ID   = Robot.pID;

if ~isempty(Rede.pMSG.getFrom)
    
    n    = length(Rede.pMSG.getFrom);
    
    if n >= ID
        % In case the robot is Pioneer
        if type(1) == 'P'
            Robot.pPos.Xd = Rede.pMSG.getFrom{ID}(2+(1:12));
            Robot.pPos.X  = Rede.pMSG.getFrom{ID}(14+(1:12));
            Robot.pSC.Ud  = Rede.pMSG.getFrom{ID}(26+(1:2));
            Robot.pSC.U   = Rede.pMSG.getFrom{ID}(28+(1:2));
            
            % Calculate control point position
            Robot.pPos.Xc([1 2 6]) = Robot.pPos.X([1 2 6]) - ...
                [Robot.pPar.a*cos(Robot.pPos.Xc(6)); Robot.pPar.a*sin(Robot.pPos.Xc(6)); 0];
            
        % For Ardrone
        elseif type(1) == 'A'
            
            Robot.pPos.Xd = Rede.pMSG.getFrom{ID}(2+(1:12));
            Robot.pPos.X  = Rede.pMSG.getFrom{ID}(14+(1:12));
            Robot.pSC.Ud  = Rede.pMSG.getFrom{ID}(26+(1:4));
            Robot.pSC.U   = Rede.pMSG.getFrom{ID}(30+(1:4));        % Calculate control point position
                       
        else
            disp('Robot class not known! Must be ''A'' or ''P''');
        end
    else
        
        str = sprintf('%d',ID);
        disp(['There are no data for robot with ID = ',str]);
        
    end
end
