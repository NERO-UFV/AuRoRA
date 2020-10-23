%    ***************************************************************
%    *    Univeridade Federal do Espírito Santo - UFES             *                          
%    *    Course:  Master of Science                               *
%    *    Student: Mauro Sergio Mafra Moreira                      *
%    *    Email:   mauromafra@gmail.com                            *
%    *    Revision: 01                           Data 00/00/2019   *
%    ***************************************************************

% Description:


classdef RosInterface < handle
  
    
    properties        
        % Properties or Parameters                                
                
        pHostIp                           % ROS Master Host IP Address
        pHostName                         % ROS Master Host Name  
        
        % Comunication Status
        pCom                               % True/False
        pComQly                            % Good Midle Bad
        pComVec                            % n Last Status        
        pLST                               % ROS Topic List
       
        % Standards Messages
        
        % Standards Publishers 
        pubCmdVel
        
        % Standards Listeners        
        lisRobPos
        
        % Standards Publishers Listeners        
        pubWatchDog
                
    end
    
    methods
        function obj = RosInterface
            mInit(obj); % Initialize variables
        end
        
        rConnect(obj,iHostParam); %
        rDisconnect(obj); %
        
        rSendMgs(obj,iPub,iMsg);
        rGetMsg(obj,iPub,iMsg);
                
        rUpdate(obj);       % Update All ROS interface Properties
        rUpdateCom(obj);    % Update ROS interface Comunication Properties
      
    end
    
end