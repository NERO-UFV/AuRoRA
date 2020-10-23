%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:

function output = rDisconnect(obj)

    % Try to Estabilish Connection
    
    disp(" ");
    try
        rosshutdown;
        disp(" ROS Master Node Shutdown Success!! ");         
        obj.pCom = 0;        
    catch    
        disp(" ROS Master Node Shutdonwn Failure! ");
        obj.pCom = 1;        
    end        
    disp(" ");
    
    % Return Function
    output = obj.pCom;
    
end

