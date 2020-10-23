%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:

function mSetGeometryStopMsg(obj)

   % Explicit Set 0 Vel to Message, it can be pass like a vector also        
    obj.msStopVel.Linear.X = 0;
    obj.msStopVel.Linear.Y = 0;
    obj.msStopVel.Linear.Z = 0;
        
    obj.msStopVel.Angular.X = 0;
    obj.msStopVel.Angular.Y = 0;
    obj.msStopVel.Angular.Z = 0;

end

