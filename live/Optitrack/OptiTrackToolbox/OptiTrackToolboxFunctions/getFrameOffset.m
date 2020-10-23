function HgOffset = getFrameOffset(MarkerPosition,MarkerDesignPosition)
% GETFRAMEOFFSET define the offset transformation between the OptiTrack 
% assigned reference and the design reference frame
%   HgOffset = GETFRAMEOFFSET(MarkerPosition,MarkerDesignPosition) defines
%   the transformation relating the marker positions in the body-fixed
%   frame as specified in Motive to the marker design position.
%
%   Note: This function requires the order of marker coordinates specified
%   in "MarkerPosition" to match the order of marker coordinates specified
%   in "MarkerDesignPosition." Failure to establish the correct
%   correspondence will result in an incorrect offset position/orientation.
%
%   See also: OptiTrack
%
%   M. Kutzer 23Feb2016, USNA

% Updates
%   10Mar2016 - Updated documentation

%% Calculate offset
% TODO - allow option to sort points to find the best correspondence
q = MarkerPosition;
p = MarkerDesignPosition;
HgOffset = pointsToSE3(q,p);

end

function H = pointsToSE3(q,p)
% POINTSTOSE3 finds the best fit rigid body between two sets of point
% correspondences.
%
%   POINTSTOSE3(q,p) This function finds a rigid body motion that best 
%   moves p to q assuming a correspondence between each vector contained 
%   in p and q (i.e. q(:,i) <--> p(:,i)).
%
%   p = H(1:3,:)*[q;ones(1,N)]
%
%   p = [p_1,p_2,p_3...p_N], p_i - 3x1 
%   q = [q_1,q_2,q_3...q_N], q_i - 3x1
%
%   References
%       [1] D.W. Eggert1, A. Lorusso2, R.B. Fisher3, "Estimating 3-D rigid 
%       body transformations: a comparison of four major algorithms," 
%       Machine Vision and Applications (1997) 9: 272–290
%
%   (c) M. Kutzer 10July2015, USNA

%TODO - implement special cases 3.2.4 from [1]

% Define total number of points
N = size(p,2);

% Calculate relative rigid body motion (METHOD 2.1)
p_cm = (1/N)*sum(p,2);
q_cm = (1/N)*sum(q,2);

p_rel = bsxfun(@minus,p,p_cm);
q_rel = bsxfun(@minus,q,q_cm);

C = p_rel*transpose(q_rel);

[U,D,V] = svd(C);

R = V*transpose(U);

if det(R) < 0 % account for reflections
    %TODO - confirm that this step of finding the location of the singular
    %value of C is needed, or if V_prime = [v1,v2,-v3] always
    ZERO = 1e-7; % close enough to zero
    d = undiag(D);
    bin = (abs(d) < ZERO);
    sgn = ones(1,3);
    sgn(bin) = -1;
    V_prime = [sgn(1)*V(:,1),sgn(2)*V(:,2),sgn(3)*V(:,3)];
    R = V_prime*transpose(U);
end

T = q_cm - R*p_cm;

H = eye(4);
H(1:3,1:3) = R;
H(1:3,4) = T;
end % end function