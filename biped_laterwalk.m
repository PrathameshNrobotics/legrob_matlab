clc
clear all;
close all;

L_hip = 1; % hip length
L_shin = 5; % shin length (length: hip to foot). change this according to the urdf
lim = L_hip/2; % stride length (radius of swing foot). keep this 1/10th of the shin length
x_rbase_init = L_hip/2; y_rbase_init = 0;
x_lbase_init = -L_hip/2; y_lbase_init = 0;
x_base_ = 0;
xl1 = x_lbase_init;
yl1 = y_lbase_init;
for iter = 0:2*lim:4*lim
% First Step (Right Leg)
for xr1 = iter+x_rbase_init:0.1:iter+x_rbase_init+2*lim
    x_base = x_base_ + (xr1 - x_rbase_init-iter)/2;
    x_rbase = x_base + x_rbase_init;
    x_lbase = x_base + x_lbase_init;
    yr1 = abs(sqrt((lim)^2 - (xr1-iter-x_rbase_init-lim)^2));
    % a,b,c are intermediate variables in inverse kinematics
    a = abs(sqrt((xr1-x_rbase)^2 + (yr1-y_rbase_init)^2));
    b = abs(sqrt((L_shin-y_rbase_init)^2));
    c = abs(sqrt((xr1-x_rbase)^2 + (yr1 - L_shin)^2));
    theta1 = acos((b^2 + c^2 - a^2)/(2*b*c)); % theta1: revolute hip joint (pitch)
    p = L_shin - c; % prismatic joint movement
    theta2 = pi/2 + theta1;

    % plotting function (not needed in pybullet)
    xr2 = xr1 + L_shin*cos(theta2);
    yr2 = yr1 + L_shin*sin(theta2);
    xl2 = xl1 + L_shin*cos(pi-theta2);
    yl2 = yl1 + L_shin*sin(pi-theta2);
    l = abs((xr2-xr1)^2 + (yr2-yr1)^2);
    plot([xl1,xl2],[yl1,yl2], 'k', [xr1,xr2],[yr1,yr2], 'r', [x_base,x_base],[y_rbase_init+3*L_shin/4,L_shin], 'b', [x_rbase,x_lbase],[L_shin,L_shin], 'b', 'linewidth', 4)
    axis([-10 10 -10 20 -0.1 10])
    xlabel('x'); ylabel('y'); zlabel('z');
    pause(0.1)
end

for xl1 = iter+x_lbase_init:0.1:iter+x_lbase_init+2*lim
    x_base_ = x_base + (xl1 - iter - x_lbase_init)/2;
    x_rbase = x_base_ + x_rbase_init;
    x_lbase = x_base_ + x_lbase_init;
    yl1 = abs(sqrt((lim)^2 - (xl1- iter- x_lbase_init-lim)^2));
    % a,b,c are intermediate variables in inverse kinematics
    a = abs(sqrt((xl1-x_lbase)^2 + (yl1-y_lbase_init)^2));
    b = abs(sqrt((L_shin-y_lbase_init)^2));
    c = abs(sqrt((xl1-x_lbase)^2 + (yl1 - L_shin)^2));
    theta1 = acos((b^2 + c^2 - a^2)/(2*b*c)); % theta1: revolute hip joint (pitch)
    p = L_shin - c; % prismatic joint movement
    theta2 = pi/2 - theta1;
    % plotting function (not needed in pybullet)
    xl2 = xl1 + L_shin*cos(theta2);
    yl2 = yl1 + L_shin*sin(theta2);
    xr2 = xr1 + L_shin*cos(pi-theta2);
    yr2 = yr1 + L_shin*sin(pi-theta2);
    l = abs((xl2-xl1)^2 + (yl2-yl1)^2);
    plot([xl1,xl2],[yl1,yl2], 'k', [xr1,xr2],[yr1,yr2], 'r', [x_base_,x_base_],[y_rbase_init+3*L_shin/4,L_shin], 'b', [x_rbase,x_lbase],[L_shin,L_shin], 'b', 'linewidth', 4)
    axis([-10 10 -10 20 -0.1 10])
    xlabel('x'); ylabel('y'); zlabel('z');
    pause(0.1)
end
end
