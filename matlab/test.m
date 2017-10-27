clear all, clc;

pos  = [0.00146465 0.000124675 0.0208581];
quat = [0.654 -0.652 -0.271 0.271]; % (w x y z)

quat = [0.707 -0.707 0 0]; % (w x y z)

quat = [-0.038 -0.69 0.710 0.128]; % (w x y z)

[yaw, pitch, roll] = quat2angle(quat);

rad2deg(yaw)
rad2deg(pitch)
rad2deg(roll)