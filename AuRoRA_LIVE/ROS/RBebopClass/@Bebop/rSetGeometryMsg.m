%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:

function output = mSetGeometryMsg(obj,twistObj)

    % Set Currernt Drone Velocity Linear[x,y,z] Angular[x,y,z]
    obj.msgVel = twistObj;

end

