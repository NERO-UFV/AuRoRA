%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:


function rSendControlSignals(obj)

if obj.pFlag.Connected == 1
    if obj.pCom.cStatus(32)==1    % flying state flag (1=flying/0=landed)
        % Experiment Mode: Ardrone 2.0
        %drone.rCommand;
    end
else
    % Simulation Mode
    obj.pSC.U = obj.pSC.Ud;
end

% Stand-by mode
obj.pSC.Ud = [0; 0; 0; 0];

end

