function n = quatnormalize(q)
%QUATNORMALIZE Normalize a quaternion.
%   N = QUATNORMALIZE(Q) computes N, the result of normalizing the
%   quaternion Q.

% https://github.com/dhr/matlab-tools/blob/master/quaternions/quatnormalize.m

n = bsxfun(@rdivide, q, sqrt(sum(q.^2, 2)));