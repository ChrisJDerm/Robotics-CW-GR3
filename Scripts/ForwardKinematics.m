clear;clc;close all;

syms J1 J2 J3 L1 L2 L3

DHTable =   [0, 0, 0, -J1;
            pi/2, 0, 0, J2;
            0, L2, 0, -(J3+pi/2)];

T01 = transform(1, DHTable);
T12 = transform(2, DHTable);
T23 = transform(3, DHTable);
T03 = T01 * T12 * T23;

P3 = [0, L3, 0, 1]';

P = T03 * P3;

% cm
l1 = 0;
l2 = 9.5;
l3 = 16.5;

j1 = deg2rad(30);
j2 = deg2rad(45);
j3 = deg2rad(60);

P_out = double(subs(P, [L1, L2, L3, J1, J2, J3], [l1, l2, l3, j1, j2, j3]))

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