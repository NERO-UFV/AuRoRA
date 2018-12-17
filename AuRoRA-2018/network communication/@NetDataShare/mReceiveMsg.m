function mReceiveMsg(obj)

% obj.pMSG.getFrom = [];
% obj.pMSG.getFormation = [];

timeOut = tic;
timeMax = 0.02;

% display(obj.pUDP.BytesAvailable)
% while toc(timeOut) < timeMax && 
while    obj.pUDP.BytesAvailable > 0
    msg = fgetl(obj.pUDP);
    
    if length(msg) > 100
        if isequal(msg([1 end]),'[]') % Compare message header
            % Message OK
            hashtag = strfind(msg,'#');
            
            switch char(str2double(msg(2:hashtag(1)-1)))
                % Pioneer P3DX 
                case 'P' 
                    % padrão 2 sinais de controle
                    msgCod = sscanf(msg,['[%f#%f#' ...
                        'Xd,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,' ...
                        'X,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,' ...
                        'Ud,%f,%f,'...
                        'U,%f,%f,'...
                        't,%f,]']);
%                       msgCod = sscanf(msg,['[%f#%f#' ...
%                         'X,%f,%f,%f]']);
                    obj.pMSG.getFrom{msgCod(2)} = msgCod;
                    % padrão 4 sinais de controle
%                       msgCod = sscanf(msg,['[%f#%f#' ...
%                         'Xd,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,' ...
%                         'X,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,' ...
%                         'Ud,%f,%f,%f,%f,'...
%                         'U,%f,%f,%f,%f,]']);
%                     obj.pMSG.getFrom{msgCod(2)} = msgCod;
                % ArDrone    
                case 'A' 
                    msgCod = sscanf(msg,['[%f#%f#' ...
                        'Xd,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,' ...
                        'X,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,' ...
                        'Ud,%f,%f,%f,%f,'...
                        'U,%f,%f,%f,%f,]']);
                    obj.pMSG.getFrom{msgCod(2)} = msgCod;
                    
            end
        end
    end
end

flushinput(obj.pUDP);
flushinput(obj.pUDP);
flushinput(obj.pUDP);