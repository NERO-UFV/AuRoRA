%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:


function rCmdStop(obj)
       
    % Linear Variable
    obj.pVel.Linear.X = 0;
    obj.pVel.Linear.Y = 0;
    obj.pVel.Linear.Z = 0;
    
    % Angular Variable
    obj.pVel.Angular.X = 0;
    obj.pVel.Angular.Y = 0;
    obj.pVel.Angular.Z = 0;      
    
    % Send Commando do Stop Drone Velocities
    send(obj.pubCmdVel,obj.pVel);
    
end

