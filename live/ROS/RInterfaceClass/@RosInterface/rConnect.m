%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:



function output = rConnect(obj,iHostParam)

    if nargin > 1    
        % If string conteins '.' treat as IP number otherwise HostName
        if contains(iHostParam,'.')
            obj.pHostName = '';
            obj.pHostIp = iHostParam;
        else           
            obj.pHostIp = '';
            obj.pHostName = iHostParam;
        end        
    end
    

    % Try to use Host IP or Host Name to connect
    if (obj.pHostIp ~= char(0))        
        rosHost = obj.pHostIp;
    else if (obj.pHostName ~= char(0))
            rosHost = obj.pHostName;
        else 
            obj.pCom = false;    
            return;
        end        
    end
    
   
    % Assegura servidor desconectado do cliente 
    obj.rDisconnect;

    % Try to Estabilish Connection
    try        
        rosinit(rosHost); %        
        obj.pCom = true;        
    catch ME
        obj.pCom = false;
        disp(" ROS Master Node Connection Failure");
        disp(' ');
        
        throw(ME); % Throw Exception to upper level 
    end

    % Return Function
    output = obj.pCom;
    
end

