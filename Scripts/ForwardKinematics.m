clear;clc;close all;

syms theta1 theta2 theta3 L1 L2 L3

DHTable =   [0, 0, 0, theta1;
            pi/2, 0, L1, theta2;
            0, L2, 0, theta3];

T01 = transform(1, DHTable);
T12 = transform(2, DHTable);
T23 = transform(3, DHTable);
T03 = T01 * T12 * T23;

P3 = [0, -L3, 0, 1]';

P = T03 * P3;
% U = T03 * [0; -1; 0; 0];

t1 = 0;
t2 = 0;
t3 = 0;

% cm
l1 = 7.3;
l2 = 9.8;
l3 = 18.5;

P_out = subs(P, [L1, L2, L3, theta1, theta2, theta3], [l1, l2, l3, t1, t2, t3])
% U_out = subs(U, [L1, L2, L3, theta1, theta2, theta3], [l1, l2, l3, t1, t2, t3])

function out = transform(i, DH)
    alp = DH(i, 1);
    a = DH(i, 2);
    d = DH(i, 3);
    th = DH(i, 4);

    out = [cos(th), -sin(th), 0, a;
        cos(alp)*sin(th), cos(alp)*cos(th), -sin(alp), -sin(alp)*d;
        sin(alp)*sin(th), sin(alp)*cos(th), cos(alp), cos(alp)*d;
        0, 0, 0, 1];
end