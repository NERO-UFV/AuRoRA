function [dCurve] = DerivBezierCurve(ControlPoints,time)
    % Number of control points on bezier curve
    N = size(ControlPoints,2);
    if nargin==1
        t=0:0.01:1;  % time
    else
        t = time;
    end 
    
    Sum = 0;
    for k=0:N-2
    %        - Derivada da curva de Bèzier:
        Sum = Sum + nchoosek(N-1,k).*ControlPoints(:,k+1).*((1-t).^(N-2-k)).*(t.^(k-1)).*(-t.*(N-1)+k);
    end
    dCurve = Sum;
end