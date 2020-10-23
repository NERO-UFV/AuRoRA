%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:


function rCmdVel(obj,iVel)

    % Linear Variable
    obj.pVel.Linear.X = iVel(1);
    obj.pVel.Linear.Y = iVel(2);
    obj.pVel.Linear.Z = iVel(3);
    
    % Angular Variable
    obj.pVel.Angular.X = iVel(4);
    obj.pVel.Angular.Y = iVel(5);
    obj.pVel.Angular.Z = iVel(6);   

    send(obj.pubCmdVel,obj.pVel);

end

