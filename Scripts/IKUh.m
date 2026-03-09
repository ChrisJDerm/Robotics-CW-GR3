clear; clc;

% ---- Demo values (edit) ----
L1 = 9.5;
L2 = 16.5;

x = 10;
y = 10;
z = 3;

[sol1, sol2, ok, msg] = ik_position_only(x,y,z,L1,L2);

disp(msg);
if ok
    fprintf('\nSolution 1 [t1 t2 t3] (deg):  %.6f  %.6f  %.6f\n', rad2deg(sol1));
    fprintf('Solution 2 [t1 t2 t3] (deg):  %.6f  %.6f  %.6f\n', rad2deg(sol2));

    % Optional check: forward-kinematics position from the image
    p1 = fk_from_image(sol1(1), sol1(2), sol1(3), L1, L2);
    p2 = fk_from_image(sol2(1), sol2(2), sol2(3), L1, L2);

    fprintf('\nFK check sol1 [x y z]:  %.6f  %.6f  %.6f\n', p1);
    fprintf('FK check sol2 [x y z]:  %.6f  %.6f  %.6f\n', p2);
end

%% -------- Functions --------

function [solPlus, solMinus, ok, msg] = ik_position_only(x,y,z,L1,L2)
%IK_POSITION_ONLY Solve for [theta1 theta2 theta3] given x,y,z (radians).
% Returns two solutions: solPlus and solMinus as 1x3 [t1 t2 t3].
%
% Based on reducing to:
%   r = sqrt(x^2+y^2)
%   r = L2*cos(phi) + L1*cos(t2)
%   z = L2*sin(phi) - L1*sin(t2)
% where phi = t3 - t2.

    tol = 1e-9;

    % theta1 from ratio y/x (quadrant-safe)
    t1 = atan2(y, x);

    r = hypot(x, y);
    d = hypot(r, z);

    if abs(L2) < tol
        ok = false; msg = 'Unsolvable: L2 is ~0 (degenerate).';
        solPlus = [NaN NaN NaN]; solMinus = solPlus;
        return;
    end
    if d < tol
        ok = false; msg = 'Unsolvable: target at origin (d ~ 0) degeneracy.';
        solPlus = [NaN NaN NaN]; solMinus = solPlus;
        return;
    end

    % Solve d*cos(phi - beta) = K
    K = (r^2 + z^2 + L2^2 - L1^2) / (2*L2);
    c = K / d;

    if abs(c) > 1 + 1e-6
        ok = false; msg = 'Unreachable: no real IK solution for this xyz.';
        solPlus = [NaN NaN NaN]; solMinus = solPlus;
        return;
    end

    c = max(-1, min(1, c));         % clamp for numerical safety
    beta = atan2(z, r);
    ang  = acos(c);

    phi1 = beta + ang;              % branch 1
    phi2 = beta - ang;              % branch 2

    solPlus  = solve_branch(t1, r, z, L1, L2, phi1);
    solMinus = solve_branch(t1, r, z, L1, L2, phi2);

    ok = true; msg = 'OK: two IK branches returned.';
end

function sol = solve_branch(t1, r, z, L1, L2, phi)
% Given phi = t3 - t2, solve t2 then t3.

    alpha = atan2(z - L2*sin(phi), r - L2*cos(phi));
    t2 = -alpha;
    t3 = t2 + phi;

    sol = [wrapToPi_local(t1), wrapToPi_local(t2), wrapToPi_local(t3)];
end

function p = fk_from_image(t1,t2,t3,L1,L2)
% Position-only forward kinematics from the image.
    c1 = cos(t1); s1 = sin(t1);
    c2 = cos(t2); s2 = sin(t2);
    c3 = cos(t3); s3 = sin(t3);

    common = L2*(c2*c3 + s2*s3) + L1*c2;

    x = c1*common;
    y = s1*common;
    z = L2*(c2*s3 - s2*c3) - L1*s2;

    p = [x y z];
end

function a = wrapToPi_local(a)
% Wrap angle to (-pi, pi]
    a = mod(a + pi, 2*pi) - pi;
end