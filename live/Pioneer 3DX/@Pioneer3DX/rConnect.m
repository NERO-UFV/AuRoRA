function rConnect(p3dx)

try
    % Real robot
    aria_init;
    arrobot_connect;
    
    % MobileSim
    eval(['aria_init -rh 127.0.0.1:' num2str(8101) ])
    
    p3dx.pFlag.Connected = 1;
    
catch
    
    % Real robot
    aria_init;
    arrobot_connect;
    
    p3dx.pFlag.Connected = 1;
    
    
end

end
