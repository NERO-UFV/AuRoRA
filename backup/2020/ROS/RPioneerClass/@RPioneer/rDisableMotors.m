function rDisableMotors(obj)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    Client = rossvcclient(['/P' num2str(obj.pID) '/disable_motors']);
    Msg = rosmessage(Client);
    call(Client,Msg);
end

