% This function applies the quaternion multiplication
function [g] = quart_mul(q, p)
    gr = q(1) * p(1) - (q(2) * p(2) + q(3) * p(3) + q(4) * p(4));
    gi = q(1) * p(2) + q(2) * p(1) + q(3) * p(4) - q(4) * p(3);
    gj = q(1) * p(3) + q(3) * p(1) - q(2) * p(4) + q(4) * p(2);
    gk = q(1) * p(4) + q(4) * p(1) + q(2) * p(3) - q(3) * p(2);
    g = [gr, gi, gj, gk];
end