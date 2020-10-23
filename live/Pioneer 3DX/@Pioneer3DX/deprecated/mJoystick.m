function mJoystick(obj,ID,Modelo)

if nargin < 2
    ID = 1;    
end
if nargin < 3
    Modelo = 'Xbox';
end

obj.pSC.Joystick.ID = ID;
obj.pSC.Joystick.Modelo = Modelo;

try
    obj.pSC.Joystick.J = vrjoystick(obj.pSC.Joystick.ID);
    obj.pSC.Joystick.OK = 1;
    
catch err;
    obj.pSC.Joystick.OK = 0;
    display(err.message);
end
    
end