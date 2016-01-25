L1 = 0.24; L2 = L1; L3 = 0.4; L4 = L3;
L5 = 0.1; L6 = 0.1;

% L1 = 1; L2 = L1; L3 = 2; L4 = L3;
% L5 = 2; L6 = 0.5;

x1 = 0; y1 = 0; x2 = L5; y2 = 0;

static_alpha = pi/2 + pi/4;

fprintf('\n*****Input lengths and const angle*****\nL1: %4.4f , L2: %4.4f\nL3: %4.4f , L4: %4.4f\nL5: %4.4f , L6: %4.4f\nalpha: %4.4f\n',...
    L1,L2,L3,L4,L5,L6,static_alpha)

semicircle_angles = linspace(pi,2*pi,50);

%% check manipulator workspace
x_possible = [];
y_possible = [];
min_q1 = acos(L5/(2*L1));
max_q2 = pi-min_q1;

for q1 = linspace(0,pi,50)
    if q1 <= min_q1
        max_q2 = pi-q1 - 0.0001;
    else
        max_q2 = pi;
    end
    for q2 = linspace(0,max_q2,50)       
        %fprintf('\n*****Input angles*****\nq1: %4.4f , q2: %4.4f\n', q1,q2)
        
        a = 2*L3*(L1*cos(q1) - L2*cos(q2) - L5);
        b = 2*L3*(L1*sin(q1) - L2*sin(q2));
        c = L4^2 - L3^2 - L1^2 - L2^2 - L5^2 +2*L1*L2*sin(q1)*sin(q2) + 2*L1*cos(q1)*(L2*cos(q2) + L5) - 2*L2*L5*cos(q2);
        q3 = 2*(atan((b - sqrt(a^2 + b^2 - c^2))/(a+c)));
        q4 = pi - asin( (L1*sin(q1) + L3*sin(q3) - L2*sin(q2)) / (L4) );
        
        x3 = L1*cos(q1);         y3 = L1*sin(q1);
        x4 = L5 + L2*cos(q2);    y4 = L2*sin(q2);
        xp = x3 + L3*cos(q3);    yp = y3 + L3*sin(q3);
        angle = static_alpha - pi + q4;
        x = xp + L6*cos(angle);  
        y = yp + L6*sin(angle);
        
        x_plot = [x1 x3 xp x xp x4 x2];
        y_plot = [y1 y3 yp y yp y4 y2];
        
%         delete(h);
%         h = plot(x_plot, y_plot,'k-o');
        
        if abs(norm([x1-x3,y1-y3]) - norm([x2-x4,y2-y4])) > 0.0000000000001
            %disp('Error! L1 != L2')
        elseif abs(norm([x3-xp,y3-yp]) - norm([x4-xp,y4-yp])) > 0.0000000000001
            %disp('Error! L3 != L4')
        elseif abs(norm([x1-x3,y1-y3]) - L1) > 0.0000000000001
            %disp('Error! L1 is not correct!')
        elseif abs(norm([x2-x4,y2-y4]) - L2) > 0.0000000000001
            %disp('Error! L2 is not correct!')
        elseif abs(norm([x3-xp,y3-yp]) - L3) > 0.0000000000001
            %disp('Error! L3 is not correct!')
        elseif abs(norm([x4-xp,y4-yp]) - L4) > 0.0000000000001
            %disp('Error! L4 is not correct!')
        elseif abs(norm([xp-x,yp-y]) - L6) > 0.0000000000001
            %disp('Error! L6 is not correct!')
        elseif abs(atan2(norm(cross([x3-x1, y3-y1, 0],[abs(x3-x1), abs(y1-y1), 0])),dot([x3-x1, y3-y1, 0],[abs(x3-x1), abs(y1-y1), 0])) - q1) > 0.0000000000001
            %disp('Error! q1 is not correct!')
        elseif (atan2(norm(cross([x4-x2, y4-y2, 0],[abs(x4-x2), abs(y2-y2), 0])),dot([x4-x2, y4-y2, 0],[abs(x4-x2), abs(y2-y2), 0])) - q2) > 0.0000000000001
            %disp('Error! q2 is not correct!')
        elseif (atan2(norm(cross([x-xp, y-yp, 0],[x4-xp, y4-yp, 0])),dot([x-xp, y-yp, 0],[x4-xp, y4-yp, 0])) - static_alpha) > 0.0000000000001
            %disp('Error! alpha is not correct!')
        elseif y3 < 0 || yp < 0
            %disp('Error! y is negative!')
        else
            x_possible = [x_possible, x];
            y_possible = [y_possible, y];
            %plot(x,y,'go','MarkerFaceColor', 'auto')
        end
    end
end

%% inverse kinematics to find q1 and q2
x_ref = 0.14; y_ref = 0.65;
%x_ref = 0.1; y_ref = 0.5;

fprintf('\n*****Input*****\nx_ref: %4.4f , y_ref: %4.4f\n', x_ref, y_ref)

%find all possible (xp,yp)
x_semicircle = (cos(semicircle_angles) * L6) + x_ref;
y_semicircle = (sin(semicircle_angles) * L6) + y_ref;

results_q1 = zeros(1,length(semicircle_angles));
results_q2 = zeros(1,length(semicircle_angles));
results_error = ones(1,length(semicircle_angles))*1000;

for i = 1:length(semicircle_angles)
    xp_ref = x_semicircle(i);
    yp_ref = y_semicircle(i);
    
    %fprintf('\n*****Test Input %i*****\nxp_ref: %4.4f , yp_ref: %4.4f\n', i, xp_ref, yp_ref)
    
    %try inverse kinematics with this xp & yp
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
    
    %fprintf('\n*****Output angles %i*****\nq1: %4.4f , q2: %4.4f\n', i, q1, q2)
    
    %try forward kinematics with this q1 & q2
    a = 2*L3*(L1*cos(q1) - L2*cos(q2) - L5);
    b = 2*L3*(L1*sin(q1) - L2*sin(q2));
    c = L4^2 - L3^2 - L1^2 - L2^2 - L5^2 +2*L1*L2*sin(q1)*sin(q2) + 2*L1*cos(q1)*(L2*cos(q2) + L5) - 2*L2*L5*cos(q2);
    q3 = 2*(atan((b - sqrt(a^2 + b^2 - c^2))/(a+c)));
    q4 = pi - asin( (L1*sin(q1) + L3*sin(q3) - L2*sin(q2)) / (L4) );
    
    x3 = L1*cos(q1);         y3 = L1*sin(q1);
    x4 = L5 + L2*cos(q2);    y4 = L2*sin(q2);
    xp = x3 + L3*cos(q3);    yp = y3 + L3*sin(q3);
    angle = static_alpha - pi + q4;
    x = xp + L6*cos(angle); 
    y = yp + L6*sin(angle);
    
    %fprintf('\n*****Output coords %i*****\nx3: %4.4f , y3: %4.4f\nx4: %4.4f , y4: %4.4f\nxp: %4.4f , yp: %4.4f\nx: %4.4f , y: %4.4f\n',...
    %i,x3,y3,x4,y4,xp,yp,x,y);
    
    if abs(norm([x1-x3,y1-y3]) - norm([x2-x4,y2-y4])) > 0.0000000000001
        disp('Error! L1 != L2')
    elseif abs(norm([x3-xp,y3-yp]) - norm([x4-xp,y4-yp])) > 0.0000000000001
        disp('Error! L3 != L4')
    elseif abs(norm([x1-x3,y1-y3]) - L1) > 0.0000000000001
        disp('Error! L1 is not correct!')
    elseif abs(norm([x2-x4,y2-y4]) - L2) > 0.0000000000001
        disp('Error! L2 is not correct!')
    elseif abs(norm([x3-xp,y3-yp]) - L3) > 0.0000000000001
        disp('Error! L3 is not correct!')
    elseif abs(norm([x4-xp,y4-yp]) - L4) > 0.0000000000001
        disp('Error! L4 is not correct!')
    elseif abs(norm([xp-x,yp-y]) - L6) > 0.0000000000001
        disp('Error! L6 is not correct!')
    elseif abs(atan2(norm(cross([x3-x1, y3-y1, 0],[abs(x3-x1), abs(y1-y1), 0])),dot([x3-x1, y3-y1, 0],[abs(x3-x1), abs(y1-y1), 0])) - q1) > 0.0000000000001
        disp('Error! q1 is not correct!')
    elseif abs(atan2(norm(cross([x4-x2, y4-y2, 0],[abs(x4-x2), abs(y2-y2), 0])),dot([x4-x2, y4-y2, 0],[abs(x4-x2), abs(y2-y2), 0])) - q2) > 0.0000000000001
        disp('Error! q2 is not correct!')
    elseif abs(atan2(norm(cross([x-xp, y-yp, 0],[x4-xp, y4-yp, 0])),dot([x-xp, y-yp, 0],[x4-xp, y4-yp, 0])) - static_alpha) > 0.0000000000001
        disp('Error! alpha is not correct!')
    else
        results_q1(i) = q1;
        results_q2(i) = q2;
        results_error(i) = sqrt((x-x_ref)^2+(y-y_ref)^2);
    end
end

% Find min error result
min_val = 0;
min_idx = 0;
[min_val,min_idx] = min(results_error)
%min_idx
fprintf('\n*****Output angles and error*****\nq1: %4.4f , q2: %4.4f, error: %4.4f\n',...
    results_q1(min_idx),results_q2(min_idx),results_error(min_idx));

f8=figure(8); clf(f8);
plot(x_semicircle,y_semicircle,'r.')
hold on
plot(x_ref,y_ref,'bx','MarkerSize',6, 'LineWidth', 2)
axis equal

if results_error(min_idx) == 1000
    close all;
    fprintf('\n!!!!!!!!!! No Possible Solution Found !!!!!!!!!!\n');
else
    % Plot final result
    q1 = results_q1(min_idx);
    q2 = results_q2(min_idx);
    %do forward kinematics with final q1 & q2
    a = 2*L3*(L1*cos(q1) - L2*cos(q2) - L5);
    b = 2*L3*(L1*sin(q1) - L2*sin(q2));
    c = L4^2 - L3^2 - L1^2 - L2^2 - L5^2 +2*L1*L2*sin(q1)*sin(q2) + 2*L1*cos(q1)*(L2*cos(q2) + L5) - 2*L2*L5*cos(q2);
    q3 = 2*(atan((b - sqrt(a^2 + b^2 - c^2))/(a+c)));
    q4 = pi - asin( (L1*sin(q1) + L3*sin(q3) - L2*sin(q2)) / (L4) );
    
    x3 = L1*cos(q1);         y3 = L1*sin(q1);
    x4 = L5 + L2*cos(q2);    y4 = L2*sin(q2);
    xp = x3 + L3*cos(q3);    yp = y3 + L3*sin(q3);
    angle = static_alpha - pi + q4;
    x = xp + L6*cos(angle);  y = yp + L6*sin(angle);
    
    x_plot = [x1 x3 xp x xp x4 x2];
    y_plot = [y1 y3 yp y yp y4 y2];
    
    plot(x_plot, y_plot,'k-o')
    fprintf('\n*****Output coords*****\nx3: %4.4f , y3: %4.4f\nx4: %4.4f , y4: %4.4f\nxp: %4.4f , yp: %4.4f\nx: %4.4f , y: %4.4f\n',...
        x3,y3,x4,y4,xp,yp,x,y);
    
    if abs(norm([x1-x3,y1-y3]) - norm([x2-x4,y2-y4])) > 0.0000000000001
        disp('Error! L1 != L2')
    elseif abs(norm([x3-xp,y3-yp]) - norm([x4-xp,y4-yp])) > 0.0000000000001
        disp('Error! L3 != L4')
    elseif abs(norm([x1-x3,y1-y3]) - L1) > 0.0000000000001
        disp('Error! L1 is not correct!')
    elseif abs(norm([x2-x4,y2-y4]) - L2) > 0.0000000000001
        disp('Error! L2 is not correct!')
    elseif abs(norm([x3-xp,y3-yp]) - L3) > 0.0000000000001
        disp('Error! L3 is not correct!')
    elseif abs(norm([x4-xp,y4-yp]) - L4) > 0.0000000000001
        disp('Error! L4 is not correct!')
    elseif abs(norm([xp-x,yp-y]) - L6) > 0.0000000000001
        disp('Error! L6 is not correct!')
    elseif abs(atan2(norm(cross([x3-x1, y3-y1, 0],[abs(x3-x1), abs(y1-y1), 0])),dot([x3-x1, y3-y1, 0],[abs(x3-x1), abs(y1-y1), 0])) - q1) > 0.0000000000001
        disp('Error! q1 is not correct!')
    elseif (atan2(norm(cross([x4-x2, y4-y2, 0],[abs(x4-x2), abs(y2-y2), 0])),dot([x4-x2, y4-y2, 0],[abs(x4-x2), abs(y2-y2), 0])) - q2) > 0.0000000000001
        disp('Error! q2 is not correct!')
    elseif (atan2(norm(cross([x-xp, y-yp, 0],[x4-xp, y4-yp, 0])),dot([x-xp, y-yp, 0],[x4-xp, y4-yp, 0])) - static_alpha) > 0.0000000000001
        disp('Error! alpha is not correct!')
    end
end

h = scatter(x_possible,y_possible,50,'g','filled');
alpha(h,0.3);
grid on; grid minor;

hold off