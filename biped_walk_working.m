clc
clear all;
close all;

L_shin = 5; % shin length (length: hip to foot). change this according to the urdf
lim = 0.5; % stride length (radius of swing foot). keep this 1/10th of the shin length
x_base = 0; y_base = 0;

% First Step (Right Leg)
for xr1 = 0:0.1:lim
    x_base = xr1/2;
    yr1 = abs(sqrt((lim)^2 - xr1^2));
    % a,b,c are intermediate variables in inverse kinematics
    a = abs(sqrt((xr1-x_base)^2 + (yr1-y_base)^2));
    b = abs(sqrt((L_shin-y_base)^2 ));
    c = abs(sqrt((xr1-x_base)^2 + (yr1 - L_shin)^2));
    theta1 = acos((b^2 + c^2 - a^2)/(2*b*c)); % theta1: revolute hip joint (roll)
    p = L_shin - c; % prismatic joint movement
    if xr1<0
        theta2 = pi/2 - theta1;
    else
        theta2 = pi/2 + theta1;
    end

    % plotting function (not needed in pybullet)
    xr2 = xr1 + L_shin*cos(theta2);
    yr2 = yr1 + L_shin*sin(theta2);
    xl2 = L_shin*cos(pi-theta2);
    yl2 = L_shin*sin(pi-theta2);
    l = abs((xr2-xr1)^2 + (yr2-yr1)^2);
    plot([0,xl2],[0,yl2], 'k', [xr1,xr2],[yr1,yr2], 'r', [x_base,x_base],[y_base+3*L_shin/4,L_shin], 'b', 'linewidth', 4)
    axis([-10 10 -10 20 -0.1 10])
    xlabel('x'); ylabel('y'); zlabel('z');
    pause(0.2)
end

x_base_right = x_base; 


% Normal Walk
for iter = 0:2*lim:4*lim
  
%     x_base_left = x_base_right + lim;

%     Left Leg movement
    for xl1 = iter:0.1:iter+2*lim
        x_base_left = x_base_right + (xl1 - iter)/2;
        yl1 = abs(sqrt((lim)^2 - (xl1-iter-lim)^2));
        a = abs(sqrt((xl1-x_base_left)^2 + (yl1-y_base)^2));
        b = abs(sqrt((L_shin-y_base)^2 ));
        c = abs(sqrt((xl1-x_base_left)^2 + (yl1 - L_shin)^2));
        thetal1 = acos((b^2 + c^2 - a^2)/(2*b*c));
        p = L_shin - c;
        if xl1< x_base_right + lim/2
            thetal2 = pi/2 - thetal1;
        else
            thetal2 = pi/2 + thetal1;
        end
        xl2 = xl1 + L_shin*cos(thetal2);
        yl2 = yl1 + L_shin*sin(thetal2);
        xr2 = xr1 + L_shin*cos(pi-thetal2);
        yr2 = yr1 + L_shin*sin(pi-thetal2);
        l = abs((xl2-xl1)^2 + (yl2-yl1)^2);
        plot([xl1,xl2],[yl1,yl2], 'k', [xr1,xr2],[yr1,yr2], 'r', [x_base_left,x_base_left],[y_base+3*L_shin/4,L_shin], 'b', 'linewidth', 4)
        axis([-10 10 -10 20 -0.1 10])
        xlabel('x'); ylabel('y'); zlabel('z');
        pause(0.2)
    end

    %     x_base_right = x_base_left + lim;

%     Right Leg movement
    for xr1 = iter+lim:0.1:iter+3*lim
        x_base_right = x_base_left + (xr1 - lim-iter)/2;
        yr1 = abs(sqrt((lim)^2 - (xr1-2*lim-iter)^2));
        a = abs(sqrt((xr1-x_base_right)^2 + (yr1-y_base)^2));
        b = abs(sqrt((L_shin-y_base)^2 ));
        c = abs(sqrt((xr1-x_base_right)^2 + (yr1 - L_shin)^2));
        thetar1 = acos((b^2 + c^2 - a^2)/(2*b*c));
        p = L_shin - c;
        if xr1< x_base_left + lim/2
            thetar2 = pi/2 - thetar1;
        else
            thetar2 = pi/2 + thetar1;
        end
        xr2 = xr1 + L_shin*cos(thetar2);
        yr2 = yr1 + L_shin*sin(thetar2);
        xl2 = xl1 + L_shin*cos(pi-thetar2);
        yl2 = yl1 + L_shin*sin(pi-thetar2);
        l = abs((xr2-xr1)^2 + (yr2-yr1)^2);
        plot([xl1,xl2],[yl1,yl2], 'k', [xr1,xr2],[yr1,yr2], 'r', [x_base_right,x_base_right],[y_base+3*L_shin/4,L_shin], 'b', 'linewidth', 4)
        axis([-10 10 -10 20 -0.1 10])
        xlabel('x'); ylabel('y'); zlabel('z');
        pause(0.2)
    end

end