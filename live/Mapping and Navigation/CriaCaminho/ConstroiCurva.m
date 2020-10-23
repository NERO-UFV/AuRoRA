function [ camx, camy ] = ConstroiCurva( curva )
%PP_CURVA Summary of this function goes here
%   Detailed explanation goes here
t = size(curva,1);
if t>1
    cx = curva(:,1);
    cy = curva(:,2);
    tx = size(cx,1);
    xq = linspace(cx(1),cx(tx),10*(tx));
    yq = pchip(cx,cy,xq);
    xy = [xq' yq'];     
    camx = xy(:,1);
    camy = xy(:,2);
end
end

