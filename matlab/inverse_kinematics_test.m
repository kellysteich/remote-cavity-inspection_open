L1 = 0.24; L2 = L1; L3 = 0.4; L4 = L3;
L5 = 0.1; L6 = 0.1;

% L1 = 1; L2 = L1; L3 = 2; L4 = L3;
% L5 = 2; L6 = 0.5;

x1 = 0; y1 = 0; x2 = L5; y2 = 0;

alpha = pi/2 + pi/4;

fprintf('\n*****Input lengths and const angle*****\nL1: %4.4f , L2: %4.4f\nL3: %4.4f , L4: %4.4f\nL5: %4.4f , L6: %4.4f\nalpha: %4.4f\n',...
    L1,L2,L3,L4,L5,L6,alpha)

%% inverse kinematics to find q1 and q2
xp_ref = 0.0; yp_ref = 0.5;

fprintf('\n*****Input*****\nxp_ref: %4.4f , yp_ref: %4.4f\n', xp_ref,yp_ref)

a1 = L1^2 + yp_ref^2 + (xp_ref)^2 - L3^2 + 2*(xp_ref)*L1;
a2 = L1^2 + yp_ref^2 + (xp_ref-L5)^2 - L3^2 + 2*(xp_ref-L5)*L1;

b1 = -4*yp_ref*L1;
b2 = -4*yp_ref*L1;

c1 = L1^2 + yp_ref^2 + (xp_ref)^2 - L3^2 - 2*(xp_ref)*L1;
c2 = L1^2 + yp_ref^2 + (xp_ref-L5)^2 - L3^2 - 2*(xp_ref-L5)*L1;

z1 = ( -b1 + sqrt(b1^2 - 4*a1*c1) ) / (2*a1);
z2 = ( -b2 - sqrt(b2^2 - 4*a2*c2) ) / (2*a2);

q1 = 2*atan(z1);
q2 = 2*atan(z2);
fprintf('\n*****Output*****\nq1: %4.4f , q2: %4.4f\n', q1,q2)

%%
%use forward kinematics with given q1 and q2 to plot result
% q1 = pi/2-pi/16;
% q2 = pi/2+pi/16;
% q1 = 100*pi/180;
% q2 = 80*pi/180;
q1 = 5*pi/6;
q2 = pi/6;

fprintf('\n*****Input angles*****\nq1: %4.4f , q2: %4.4f\n', q1,q2)

%% My equations
a = 2*L3*(L1*cos(q1) - L2*cos(q2) - L5);
b = 2*L3*(L1*sin(q1) - L2*sin(q2));
c = L4^2 - L3^2 - L1^2 - L2^2 - L5^2 +2*L1*L2*sin(q1)*sin(q2) + 2*L1*cos(q1)*(L2*cos(q2) + L5) - 2*L2*L5*cos(q2);
q3 = 2*(atan((b - sqrt(a^2 + b^2 - c^2))/(a+c)));
q4 = pi - asin( (L1*sin(q1) + L3*sin(q3) - L2*sin(q2)) / (L4) );
fprintf('\n*****Output angles*****\nq3: %4.4f , q4: %4.4f\n', q3,q4)

%% Plot
x3 = L1*cos(q1);         y3 = L1*sin(q1);
x4 = L5 + L2*cos(q2);    y4 = L2*sin(q2);
xp = x3 + L3*cos(q3);    yp = y3 + L3*sin(q3);

xp2 = x4 + L4*cos(q4);   yp2 = y4 + L4*sin(q4);

angle = alpha - pi + q4;
x = xp + L6*cos(angle);  y = yp + L6*sin(angle);

x_plot = [x1 x3 xp x xp x4 x2];
y_plot = [y1 y3 yp y yp y4 y2];

f7 = figure(7)
plot(x_plot, y_plot,'k-o')
axis equal
fprintf('\n*****Output coords*****\nx3: %4.4f , y3: %4.4f\nx4: %4.4f , y4: %4.4f\nxp: %4.4f , yp: %4.4f (calculated with q4: xp : %4.4f , yp: %4.4f)\nx: %4.4f , y: %4.4f\n',...
    x3,y3,x4,y4,xp,yp,xp2,yp2,x,y);

% fprintf('\n*****Output Lengths*****\nL1: %4.4f , L2: %4.4f , L3: %4.4f , L4: %4.4f , L5: %4.4f , L6: %4.4f\n',...
%     norm([x1-x3,y1-y3]),norm([x2-x4,y2-y4]),norm([x3-xp,y3-yp]),norm([x4-xp,y4-yp]),norm([x1-x2,y1-y2]),norm([xp-x,yp-y]));

if abs(norm([x1-x3,y1-y3]) - norm([x2-x4,y2-y4])) > 0.0000000000001
    disp('Error! L1 != L2')
end
if abs(norm([x3-xp,y3-yp]) - norm([x4-xp,y4-yp])) > 0.0000000000001
    disp('Error! L3 != L4')
end
if abs(norm([x1-x3,y1-y3]) - L1) > 0.0000000000001
    disp('Error! L1 is not correct!')
end
if abs(norm([x2-x4,y2-y4]) - L2) > 0.0000000000001
    disp('Error! L2 is not correct!')
end
if abs(norm([x3-xp,y3-yp]) - L3) > 0.0000000000001
    disp('Error! L3 is not correct!')
end
if abs(norm([x4-xp,y4-yp]) - L4) > 0.0000000000001
    disp('Error! L4 is not correct!')
end
if abs(norm([xp-x,yp-y]) - L6) > 0.0000000000001
    disp('Error! L6 is not correct!')
end
u = [x3-x1, y3-y1, 0];
v = [abs(x3-x1), abs(y1-y1), 0];
if abs(atan2(norm(cross(u,v)),dot(u,v)) - q1) > 0.0000000000001
    disp('Error! q1 is not correct!')
    atan2(norm(cross(u,v)),dot(u,v))
end
u = [x4-x2, y4-y2, 0];
v = [abs(x4-x2), abs(y2-y2), 0];
if (atan2(norm(cross(u,v)),dot(u,v)) - q2) > 0.0000000000001
    disp('Error! q2 is not correct!')
    atan2(norm(cross(u,v)),dot(u,v))
end
u = [x-xp, y-yp, 0];
v = [x4-xp, y4-yp, 0];
if (atan2(norm(cross(u,v)),dot(u,v)) - alpha) > 0.0000000000001
    disp('Error! alpha is not correct!')
    atan2(norm(cross(u,v)),dot(u,v))
end
