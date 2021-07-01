function tf = Collinear(hpo,P, tol)
% True for collinear points.
%   TF = 1 if the points P(i,:),  i=1,...,size(P,1) are collinear up to a given
%   tolerance TOL. Works for any dimension (number of columns in P) for any 
%   number of points (rows of P). If not given, TOL = 0.
%
%   Example:
%       % Two points are always collinear
%       collinear([1 0; 1 5]); % returns true
%       % Three points in 3D which are supposed to be collinear
%       collinear([0 0 0; 1 1 1; 5 5 5]); % returns false due to numerical error
%       % The previous example with looser tolerance
%       collinear([0 0 0; 1 1 1; 5 5 5], 1e-14); % returns true
%
%   See also:  RANK
%   The algorithm for three points is from Tim Davis:
%       http://blogs.mathworks.com/loren/2008/06/06/collinearity/#comment-29479
%
%   Zoltan Csati
%   24/04/2017
if nargin < 2
    tol = 0;
end
tf = rank(bsxfun(@minus, P, P(1,:)), tol) < 2;