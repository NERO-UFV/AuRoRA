function [Curve] = BezierCurve(hpo,ControlPoints,time)
    % Number of control points on bezier curve
    N = size(ControlPoints,2);
    if nargin==2
        t=0:0.02:1;  % time
    else
        t = time;
    end
    
    Sum = 0;
    for i=0:N-1
        Sum = Sum + nchoosek(N-1,i)*(1-t).^(N-1-i).*(t.^i).*ControlPoints(:,i+1);
    end
    Curve = Sum;
end