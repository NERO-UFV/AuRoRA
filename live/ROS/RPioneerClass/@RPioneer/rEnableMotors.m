function rEnableMotors(obj)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    Client = rossvcclient(['/P' num2str(obj.pID) '/enable_motors']);
    Msg = rosmessage(Client);
    call(Client,Msg);
end

