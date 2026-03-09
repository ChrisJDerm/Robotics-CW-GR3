clear;clc;close all;

L1 = 9.5;
L2 = 16.5;

ik(16.4, 16.4, -6.9, L1, L2);

function [sol1, sol2, sol3, sol4] = ik(x, y, z, L1, L2)

    % Convert from flipped frame -> FK frame
    y0 = -y;
    z0 = -z;

    theta1 = atan2(y0, x);

    r = sqrt(x^2 + y0^2);

    d = sqrt(r^2 + z0^2);

    % Reachability check (optional)
    % if d > (L1+L2) || d < abs(L1-L2)
    %     error("Target out of reach");
    % end

    a_len = (L1^2 - L2^2 + d^2) / (2*d);
    h_sq  = L1^2 - a_len^2;
    h     = sqrt(max(h_sq, 0));

    % Point along the line from origin to target
    px = a_len * r / d;
    pz = a_len * z0 / d;

    % Perpendicular offset (two intersections)
    ox = -h * z0 / d;
    oz =  h * r  / d;

    % Two possible elbow points
    P1a = [px + ox, pz + oz];
    P1b = [px - ox, pz - oz];

    % Solve for each branch
    [t2a, t3a] = branch_angles(r, z0, P1a(1), P1a(2));
    [t2b, t3b] = branch_angles(r, z0, P1b(1), P1b(2));

    sol1 = rad2deg([wrapToPi(theta1), wrapToPi(t2a), wrapToPi(t3a)]);
    sol2 = rad2deg([wrapToPi(theta1), wrapToPi(t2b), wrapToPi(t3b)]);

    % Same physical solutions, wrapped by 2*pi on theta1 if you want 4 outputs
    sol3 = rad2deg([wrapToPi(theta1 + 2*pi), wrapToPi(t2a), wrapToPi(t3a)]);
    sol4 = rad2deg([wrapToPi(theta1 + 2*pi), wrapToPi(t2b), wrapToPi(t3b)]);
end

function [theta2, theta3] = branch_angles(r, z, p1r, p1z)
    a    = atan2(p1z, p1r);          % P1 = (L1 cos a, L1 sin a)
    theta2 = -a;

    phi  = atan2(z - p1z, r - p1r);  % absolute angle of link 2 in your FK form
    theta3 = phi - a;
end