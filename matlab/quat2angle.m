function [z y x] = quat2angle(q)
%QUAT2ANGLE Convert quaternion to Euler angles.
%   [Z Y X] = QUAT2ANGLE(Q) calculates the Euler angles, Z, Y, X, for a
%   quaternion, Q.

% https://github.com/dhr/matlab-tools/blob/master/quaternions/quat2angle.m

qn = quatnormalize(q);
z = atan2(2.*(qn(:,2).*qn(:,3) + qn(:,1).*qn(:,4)), ...
          qn(:,1).^2 + qn(:,2).^2 - qn(:,3).^2 - qn(:,4).^2);
y = asin(-2.*(qn(:,2).*qn(:,4) - qn(:,1).*qn(:,3)));
x = atan2(2.*(qn(:,3).*qn(:,4) + qn(:,1).*qn(:,2)), ...
          qn(:,1).^2 - qn(:,2).^2 - qn(:,3).^2 + qn(:,4).^2);