%%%%%%%%%%%%%%%%%%%%%%
% Boundary Conditions %
%%%%%%%%%%%%%%%%%%%%%%

x0 = 10;  y0 = 20;  z0 = 5;    % start position
xf = 80;  yf = 50;  zf = 30;   % end position
tf = 3;                        % final time

%%%%%%%%%%%%%%%%%%%%%%
% Create Time Array %
%%%%%%%%%%%%%%%%%%%%%%

t = 0:0.001:tf;
t = t';

%%%%%%%%%%%%%%%%%%%%%%
% Cubic Polynomial Coefficients %
%%%%%%%%%%%%%%%%%%%%%%

ax0 = x0;
ax1 = 0;
ax2 = 3/tf^2*(xf-x0);
ax3 = -2/tf^3*(xf-x0);

ay0 = y0;
ay1 = 0;
ay2 = 3/tf^2*(yf-y0);
ay3 = -2/tf^3*(yf-y0);

az0 = z0;
az1 = 0;
az2 = 3/tf^2*(zf-z0);
az3 = -2/tf^3*(zf-z0);

%%%%%%%%%%%%%%%%%%%%%%
% Position Trajectories %
%%%%%%%%%%%%%%%%%%%%%%

x = ax0 + ax1*t + ax2*t.^2 + ax3*t.^3;
y = ay0 + ay1*t + ay2*t.^2 + ay3*t.^3;
z = az0 + az1*t + az2*t.^2 + az3*t.^3;

%%%%%%%%%%%%%%%%%%%%%%
% Velocity Trajectories %
%%%%%%%%%%%%%%%%%%%%%%

xdot = ax1 + 2*ax2*t + 3*ax3*t.^2;
ydot = ay1 + 2*ay2*t + 3*ay3*t.^2;
zdot = az1 + 2*az2*t + 3*az3*t.^2;

%%%%%%%%%%%%%%%%%%%%%%
% Acceleration Trajectories %
%%%%%%%%%%%%%%%%%%%%%%

xddot = 2*ax2 + 6*ax3*t;
yddot = 2*ay2 + 6*ay3*t;
zddot = 2*az2 + 6*az3*t;

%%%%%%%%%%%%%%%%%%%%%%
% Plot XYZ vs Time %
%%%%%%%%%%%%%%%%%%%%%%

figure
sgtitle('3D Cubic Trajectory Components')

subplot(3,3,1), plot(t,x), title('x Position')
subplot(3,3,2), plot(t,xdot), title('x Velocity')
subplot(3,3,3), plot(t,xddot), title('x Acceleration')

subplot(3,3,4), plot(t,y), title('y Position')
subplot(3,3,5), plot(t,ydot), title('y Velocity')
subplot(3,3,6), plot(t,yddot), title('y Acceleration')

subplot(3,3,7), plot(t,z), title('z Position')
subplot(3,3,8), plot(t,zdot), title('z Velocity')
subplot(3,3,9), plot(t,zddot), title('z Acceleration')

%%%%%%%%%%%%%%%%%%%%%%
% Plot 3D Path %
%%%%%%%%%%%%%%%%%%%%%%

figure
plot3(x,y,z,'LineWidth',2)
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
title('3D Cubic Position Curve')