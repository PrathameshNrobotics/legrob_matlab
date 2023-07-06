clc
clear all;
close all;

zyi = pi/2; 
n = 4;

L_hip = 1; % hip length
L_shin = 5; % shin length (length: hip to foot). change this according to the urdf
lim = L_hip/(2*n); % stride length (radius of swing foot). keep this 1/10th of the shin length
x_rbase_init = L_hip/2; y_rbase_init = 0; z_rbase_init = 0;
x_lbase_init = -L_hip/2; y_lbase_init = 0; z_lbase_init = 0;
x_base_ = 0; y_base = 0; z_base = 0;
xl1 = x_lbase_init;
yl1 = y_lbase_init;


% First Step (Right Leg)
% for step = 1:1:n
difference = 2*lim;
xr1_initial = x_rbase_init;
yr1_initial = y_rbase_init;
xr1_final = x_rbase_init - difference;
yr1_final = y_rbase_init;
yr1 = yr1_initial;
zl1 = 0;
flag = 0;
yaw_angle = acos((L_hip - xr1_final + xr1_initial)/L_hip);
for xr1 = xr1_initial:-0.05:xr1_final
    %     yr1 = yr1 + 0.05;
    x_base = x_base_;
    x_rbase = x_base + x_rbase_init;
    x_lbase = x_base + x_lbase_init;
    xr_centre = (xr1_final+xr1_initial)/2;
    yr_centre = (yr1_final+yr1_initial)/2;
    zr1 = abs(sqrt((lim)^2 - (xr1 - xr_centre)^2 - (yr_centre)^2));
    % a,b,c are intermediate variables in inverse kinematics
    a = abs(sqrt((xr1-x_rbase)^2 + (zr1-z_rbase_init)^2));
    b = abs(sqrt((L_shin-z_rbase_init)^2));
    c = abs(sqrt((xr1-x_rbase)^2 + (zr1 - L_shin)^2));
    theta_pitch1 = acos((b^2 + c^2 - a^2)/(2*b*c)); % theta1: revolute hip joint (pitch)
    p = L_shin - c; % prismatic joint movement
    theta_pitch2 = pi/2 - theta_pitch1;

    % plotting function (not needed in pybullet)
    xr2 = xr1 + L_shin*cos(theta_pitch2);
    yr2 = yr1;
    zr2 = zr1 + L_shin*sin(theta_pitch2);
    xl2 = xl1;
    yl2 = yl1;
    zl2 = zl1 + L_shin;
    l = abs((xr2-xr1)^2 + (yr2-yr1)^2 + (zr2-zr1)^2);
    if abs(xr1_final - xr1) < 0.1 && flag == 0
        yaw = yaw_angle;
        disp('right yaw');
        flag = 1;
    end
     
    plot3([xl1,xl2],[yl1,yl2],[zl1,zl2], 'k', [xr1,xr2],[yr1,yr2],[zr1,zr2], 'r', 'linewidth', 4)
    
    axis([-10 10 -10 10 -0.1 10])
    xlabel('x'); ylabel('y'); zlabel('z');
    pause(0.2)
end
flag = 0;
for yl1 = 0:0.001:abs(sqrt(L_hip^2 - (L_hip-0.25)^2))
    y_base = yl1/2;
    zl1 = abs(sqrt((L_hip^2 - (L_hip-0.25)^2) - yl1^2));
    % a,b,c are intermediate variables in inverse kinematics
    a = abs(sqrt((yl1-y_base)^2 + (zl1-z_base)^2));
    b = abs(sqrt((L_shin-z_base)^2 ));
    c = abs(sqrt((yl1-y_base)^2 + (zl1 - L_shin)^2));
    theta_roll1 = acos((b^2 + c^2 - a^2)/(2*b*c)); % theta1: revolute hip joint (roll)
    p = L_shin - c; % prismatic joint movement
    if yl1<0
        theta_roll2 = pi/2 - theta_roll1;
    else
        theta_roll2 = pi/2 + theta_roll1;
    end
    
    if abs(abs(sqrt(L_hip^2 - (L_hip-0.25)^2)) - yl1) < 0.1 && flag == 0
        yaw = yaw_angle;
        flag = 1;
        disp('left yaw');
    end
    
    % plotting function (not needed in pybullet)
    xl2 = xl1;
    yl2 = yl1 + L_shin*cos(theta_roll2);
    zl2 = zl1 + L_shin*sin(theta_roll2);
    xr2 = xr1;
    yr2 = yr2;
    zr2 = zr2;
    l = abs((xr2-xr1)^2 + (yr2-yr1)^2);
    plot3([xl1,xl2],[yl1,yl2],[zl1,zl2], 'k', [xr1,xr2],[yr1,yr2],[zr1,zr2], 'r', 'linewidth', 4)
    axis([-10 10 -10 10 -0.1 10])
    xlabel('x'); ylabel('y'); zlabel('z');
    pause(0.01)
end
