classdef JoyControl < handle
    % In a methods block, set the method attributes
    % and add the function signature
    properties
        % Properties or Parameters
        pJoy       % Xbox Joystick Connection
        pID        % Identification
        pDigital   % Button
        pAnalog    % Axis
        pIsConnected
        
    end
    
    methods
        function obj = JoyControl(ID)
            if nargin < 1
                ID = 1;
            end
            obj.pID = ID;
            obj.pIsConnected = 0;
            mConnect(obj);
        end
        
        function mConnect(obj)
            try
                obj.pJoy = vrjoystick(obj.pID);
                disp ('Joystick Conected!');
                obj.pIsConnected = 1;
                % Perform Joystick reading
                obj.pDigital = button(obj.pJoy);
                obj.pAnalog  = axis(obj.pJoy);
                
            catch error
                obj.pIsConnected = 0;
                display(error.message);
                disp('Possible error: Joystick could not be connected.')
                disp('Possible error: Simulink Virtual Reality 3D is missing.')
            end
        end
        
        function mRead(obj)
            % Perform Joystick reading
            obj.pDigital = button(obj.pJoy);
            obj.pAnalog   = axis(obj.pJoy);
            
            % Corrects axis bias
            for ii = 1:5
                if abs(obj.pAnalog(ii)) < 0.2
                    obj.pAnalog(ii) = 0;
                end
            end
        end
        
        function robot = mControl(obj,robot)
            % Joystick control
            % XBOX gamepad mapping:
            %       [LB/LT]                                   [RB/RT]
            %     . - . - . -                               - . - . - .
            %   .             \ - - - - - - - - - -  - -  /            .
            %  .       ^                                                .
            % .       (-)                                      [4]      |
            % |  <(-) [9] (+)>     <[7] (XBOX) [8]>         [3]   [2]   .
            % .       (+)                             ^        [1]      |
            % |        v                             (-)                .
            % .       (LA)                      <(-)[10](+)>            |
            % |                            (RA)      (+)                .
            % .                                       v                 |
            % |             - - - - - - - - - - - - - - - -             .
            % .          /                                  \           |
            %  \      /                                        \       /
            %   \  /                                              \  /
            %
            %
            %      ---                           ---
            %    /  --  \                      / --  \
            %   |  |LT|  \ _________________ /  |RT|  |
            %   |  |  |                         |  |  |
            %   |   --                           --   |
            %   |                                     |
            %    \ [5-LB] - - - - - - - - - -  [6-RB] /
            %      _____/                     \______
            %
            %
            % LA,RA = Left Axis,Right Axis
            % LT,RT = Left Trigger, Right Trigger (ANALOG)
            % LB,RB = Left Button, Right Button
            %
            % AXIS VECTOR: [XL YL (LT+/RT-) XR YR]
            
            if obj.pIsConnected == 1
                mRead(obj)
                type = class(robot);
                % Select robot type
                if type(1) == 'P' || type(1) == 'p'
                    if sum(abs(obj.pAnalog)) > 0
                        % Joystick Control signals
                        temp  = obj.pAnalog'.*[1 -1 -1 -1 1]';
                        robot.pSC.Ud = [0.75*temp(2); 0.5*temp(4)];
                        % robot.pSC.Ud = [0.25*temp(2); 0.5*temp(4)];
                    else
                        robot.pSC.Ud = [0; 0];
                    end
                    
                    % ====================================================
                elseif type(1) == 'A' || type(1) == 'a' 
                    if robot.pFlag.Connected == 1 % (WiFi Connected)                                                
                        if (robot.pCom.cStatus(32)==0)
                            % Arrobot is landed
                            % Conditions to takeoff (Y button pressed)
                            if obj.pDigital(4)==1
                                robot.rTakeOff;
                            end
                        else
                            % Arrobot is flying
                            if obj.pDigital(1)==1
                                % Condition to land (A button pressed)
                                robot.rLand;
                            end
                        end
                    end
                    
                    if sum(abs(obj.pAnalog)) > 0
                        % Joystick Control signals
                        temp  = obj.pAnalog'.*[1 -1 -1 1 1]';
                        robot.pSC.Ud = [temp(4); temp(5); temp(2); temp(3)];
                  
                    end
                    
                elseif type(1) == 'B' || type(1) == 'b'                    
                      if robot.pFlag.Connected == 1 % (WiFi Connected)                                                
                          if obj.pDigital(4)==1
                              robot.rTakeOff;
                          end
                          
                          % Arrobot is flying
                          if obj.pDigital(1)==1
                              % Condition to land (A button pressed)
                              robot.rLand;
                          end
                       end
                                        
                       if sum(abs(obj.pAnalog)) > 0
                           % Joystick Control signals
                           temp  = obj.pAnalog'.*[1 -1 -1 -1 -1]';
                           robot.pSC.Ud = [temp(5); temp(4); temp(2); 0 ;0; temp(3)];                           
                       end
                    
                else                    
                    disp('Robot type not supported');
                end
            end
        end
        
    end
end