clear;clc;close all;
%% Demo (edit values as you like)
% Angles in radians:
theta1 = deg2rad(45);
theta2 = deg2rad(45);
theta3 = deg2rad(45);

% Link lengths:
L1 = 9.5;
L2 = 16.5;

[x, y, z] = fk_from_image(theta1, theta2, theta3, L1, L2);
fprintf('x = %.6f\ny = %.6f\nz = %.6f\n', x, y, z);

function [x, y, z] = fk_from_image(theta1, theta2, theta3, L1, L2)
%FK_FROM_IMAGE Forward kinematics from the matrix in the provided image.
%   Inputs:
%     theta1, theta2, theta3 : joint angles (radians)
%     L1, L2                 : link lengths as in the image equation
%   Outputs:
%     x, y, z                : position components

    s1 = sin(theta1);  c1 = cos(theta1);
    s2 = sin(theta2);  c2 = cos(theta2);
    s3 = sin(theta3);  c3 = cos(theta3);

    x = L2*c1*c2*c3 + L2*c1*s2*s3 + L1*c1*c2;
    y = -(L2*s1*c2*c3 + L2*s1*s2*s3 + L1*s1*c2);
    z = -(-L2*s2*c3 + L2*c2*s3 - L1*s2);
end