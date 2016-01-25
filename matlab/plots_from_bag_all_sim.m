clear all

%% Load the bag file into memory
disp('Loading the bag file...')
filename = 'sim_3_cut';
file = [filename,'.bag'];
filefolder = '/home/kelly/Desktop/master_thesis/bags_simulator'
filepath = fullfile(filefolder, file)
bag = rosbag(filepath)
bag.AvailableTopics

%% *****Set Params

set(0,'defaulttextinterpreter','none');
forest_green = [3 163 255] / 255;
pink = [255 0 127] / 255;

% The properties we've been using in the figures
set(0,'defaultLineLineWidth',1.5);   % set the default line width to lw
set(0,'defaultLineMarkerSize',1); % set the default line marker size to msz

width = 10;     % Width in inches
height = 5;    % Height in inches

% Set the default Size for display
defpos = get(0,'defaultFigurePosition');
set(0,'defaultFigurePosition', [defpos(1) defpos(2) width*100, height*100]);

% Set the defaults for saving/printing to a file
set(0,'defaultFigureInvertHardcopy','on'); % This is the default anyway
set(0,'defaultFigurePaperUnits','inches'); % This is the default anyway
defsize = get(gcf, 'PaperSize');
left = (defsize(1)- width)/2;
bottom = (defsize(2)- height)/2;
defsize = [left, bottom, width, height];
set(0, 'defaultFigurePaperPosition', defsize);

gt_cavity_pos_world = [1.0,0,1.5];

desired_pitching_angle = -0.1;
robot_dist_to_tree = 0.53;
cavity_depth = 0.05;

arm_to_robot_translation = [0 , 0 , -0.1];
robot_offset = [-0.09 , 0 , 0.02];

camera_to_robot_translation = [0.1224 , 0 , -0.052];
camera_to_robot_rotation_quat = [-0.474386 , 0.524365 , -0.524365 , 0.474386];
camera_to_robot_rotation_rotm = [0 -0.0998334   0.995004 ; -1 0 0 ; 0  -0.995004 -0.0998334]


disp('Set params!')
        
%% ********Get gt robot pose & desired robot pose & (with offset)******* 
disp('figure 1...')

clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/firefly/command/pose'});
msgs = readMessages(bagselect);

disp('Loaded /firefly/command/pose from the bag file!')

% Do Calculations
desired_robot_pos_world_x = []; desired_robot_pos_world_y = []; desired_robot_pos_world_z = [];
desired_robot_or_world_x = []; desired_robot_or_world_y = []; desired_robot_or_world_z = []; desired_robot_or_world_w = [];
desired_robot_pos_world_time = [];

for m = 1:length(msgs)  
    desired_robot_pos_world_time_temp = double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9;
    if desired_robot_pos_world_time_temp ~= 0
        desired_robot_pos_world_time = [desired_robot_pos_world_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
        desired_robot_pos_world_x = [desired_robot_pos_world_x, msgs{m}.Pose.Position.X];
        desired_robot_pos_world_y = [desired_robot_pos_world_y, msgs{m}.Pose.Position.Y];
        desired_robot_pos_world_z = [desired_robot_pos_world_z, msgs{m}.Pose.Position.Z];
        desired_robot_or_world_x = [desired_robot_or_world_x, msgs{m}.Pose.Orientation.X];
        desired_robot_or_world_y = [desired_robot_or_world_y, msgs{m}.Pose.Orientation.Y];
        desired_robot_or_world_z = [desired_robot_or_world_z, msgs{m}.Pose.Orientation.Z];
        desired_robot_or_world_w = [desired_robot_or_world_w, msgs{m}.Pose.Orientation.W];
    end
end
disp('Saved /firefly/command/pose to variables!')

clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/firefly/rci/controller/output/robot_goal_debug_pose'});
msgs = readMessages(bagselect);
disp('Loaded /firefly/rci/controller/output/robot_goal_debug_pose from the bag file!')

% Do Calculations
desired_no_offset_robot_pos_world_x = []; desired_no_offset_robot_pos_world_y = []; desired_no_offset_robot_pos_world_z = [];
desired_no_offset_robot_or_world_x = []; desired_no_offset_robot_or_world_y = []; desired_no_offset_robot_or_world_z = []; desired_no_offset_robot_or_world_w = [];
desired_no_offset_robot_pos_world_time = [];

for m = 1:length(msgs)   
    desired_no_offset_robot_pos_world_time = [desired_no_offset_robot_pos_world_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    desired_no_offset_robot_pos_world_x = [desired_no_offset_robot_pos_world_x, msgs{m}.Pose.Position.X];
    desired_no_offset_robot_pos_world_y = [desired_no_offset_robot_pos_world_y, msgs{m}.Pose.Position.Y];
    desired_no_offset_robot_pos_world_z = [desired_no_offset_robot_pos_world_z, msgs{m}.Pose.Position.Z];
    desired_no_offset_robot_or_world_x = [desired_no_offset_robot_or_world_x, msgs{m}.Pose.Orientation.X];
    desired_no_offset_robot_or_world_y = [desired_no_offset_robot_or_world_y, msgs{m}.Pose.Orientation.Y];
    desired_no_offset_robot_or_world_z = [desired_no_offset_robot_or_world_z, msgs{m}.Pose.Orientation.Z];
    desired_no_offset_robot_or_world_w = [desired_no_offset_robot_or_world_w, msgs{m}.Pose.Orientation.W];
end
disp('Saved /firefly/rci/controller/output/robot_goal_debug_pose to variables!')

clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/firefly/ground_truth/odometry'});
msgs = readMessages(bagselect);
disp('Loaded /firefly/ground_truth/odometry from the bag file!')

robot_pos_world_x = []; robot_pos_world_y = []; robot_pos_world_z = [];
robot_or_world_x = []; robot_or_world_y = []; robot_or_world_z = []; robot_or_world_w = [];
robot_pos_world_time = [];

for m = 1:length(msgs)   
    robot_pos_world_time = [robot_pos_world_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    robot_pos_world_x = [robot_pos_world_x, msgs{m}.Pose.Pose.Position.X];
    robot_pos_world_y = [robot_pos_world_y, msgs{m}.Pose.Pose.Position.Y];
    robot_pos_world_z = [robot_pos_world_z, msgs{m}.Pose.Pose.Position.Z];
    robot_or_world_x = [robot_or_world_x, msgs{m}.Pose.Pose.Orientation.X];
    robot_or_world_y = [robot_or_world_y, msgs{m}.Pose.Pose.Orientation.Y];
    robot_or_world_z = [robot_or_world_z, msgs{m}.Pose.Pose.Orientation.Z];
    robot_or_world_w = [robot_or_world_w, msgs{m}.Pose.Pose.Orientation.W];
end
disp('Saved /firefly/ground_truth/odometry to variable!')

clearvars bagselect msgs

%% ********Plot gt robot position & desired robot position & with offset******* 
f1=figure(1);
clf(f1);
x_robot = subplot(3,1,1);       % add first plot in 3 x 1 grid
plot(desired_robot_pos_world_time, desired_robot_pos_world_x, 'LineWidth',1, 'Color','r', 'LineStyle','-');
hold on
plot(desired_no_offset_robot_pos_world_time, desired_no_offset_robot_pos_world_x, 'LineWidth',1, 'Color','b', 'LineStyle','-');
plot(robot_pos_world_time, robot_pos_world_x, 'LineWidth',1, 'Color','g', 'LineStyle','-');

title('X coordinate of robot vs. desired robot vs desired no offset robot in world frame');
xlabel('time [s]'); ylabel('coordinate [m]');
legend('desired robot pos with offset','desired robot pos', 'robot pos','Location', 'northeastoutside')
set(gca,'XMinorTick', 'on', 'YMinorTick', 'on','YGrid', 'on', 'YMinorGrid', 'on');
ylim_x=get(gca,'YLim');
xlim_x=get(gca,'XLim')
hold off

y_robot = subplot(3,1,2);       % add second plot in 3 x 1 grid
plot(desired_robot_pos_world_time, desired_robot_pos_world_y, 'LineWidth',1, 'Color','r', 'LineStyle','-');
hold on
plot(desired_no_offset_robot_pos_world_time, desired_no_offset_robot_pos_world_y, 'LineWidth',1, 'Color','b', 'LineStyle','-');
plot(robot_pos_world_time, robot_pos_world_y, 'LineWidth',1, 'Color','g', 'LineStyle','-');

title('Y coordinate of robot vs. desired robot vs desired no offset robot in world frame');
xlabel('time [s]'); ylabel('coordinate [m]');
legend('desired robot pos with offset','desired robot pos', 'robot pos','Location', 'northeastoutside')
set(gca,'XMinorTick', 'on', 'YMinorTick', 'on','YGrid', 'on', 'YMinorGrid', 'on');
ylim_y=get(gca,'YLim');
xlim_y=get(gca,'XLim')
hold off

z_robot = subplot(3,1,3);       % add third plot in 3 x 1 grid
plot(desired_robot_pos_world_time, desired_robot_pos_world_z, 'LineWidth',1, 'Color','r', 'LineStyle','-');
hold on
plot(desired_no_offset_robot_pos_world_time, desired_no_offset_robot_pos_world_z, 'LineWidth',1, 'Color','b', 'LineStyle','-');
plot(robot_pos_world_time, robot_pos_world_z, 'LineWidth',1, 'Color','g', 'LineStyle','-');

title('Z coordinate of robot vs. desired robot vs desired no offset robot in world frame');
xlabel('time [s]'); ylabel('coordinate [m]');
legend('desired robot pos with offset','desired robot pos', 'robot pos','Location', 'northeastoutside')
set(gca,'XMinorTick', 'on', 'YMinorTick', 'on','YGrid', 'on', 'YMinorGrid', 'on');
ylim_z=get(gca,'YLim');
xlim_z=get(gca,'XLim')
hold off

new_desired_robot_pos_world_x = [desired_robot_pos_world_time', desired_robot_pos_world_x'];
new_desired_robot_pos_world_y = [desired_robot_pos_world_time', desired_robot_pos_world_y'];
new_desired_robot_pos_world_z = [desired_robot_pos_world_time', desired_robot_pos_world_z'];

new_desired_no_offset_robot_pos_world_x = [desired_no_offset_robot_pos_world_time', desired_no_offset_robot_pos_world_x'];
new_desired_no_offset_robot_pos_world_y = [desired_no_offset_robot_pos_world_time', desired_no_offset_robot_pos_world_y'];
new_desired_no_offset_robot_pos_world_z = [desired_no_offset_robot_pos_world_time', desired_no_offset_robot_pos_world_z'];

start_time_commands = desired_no_offset_robot_pos_world_time(1)
start_idx = find(robot_pos_world_time >= start_time_commands);
start_idx(1)

new_robot_pos_world_x = [robot_pos_world_time(start_idx:end)', robot_pos_world_x(start_idx:end)'];
new_robot_pos_world_y = [robot_pos_world_time(start_idx:end)', robot_pos_world_y(start_idx:end)'];
new_robot_pos_world_z = [robot_pos_world_time(start_idx:end)', robot_pos_world_z(start_idx:end)'];

error_desired_robot_pos_world_x = pdist2(new_desired_robot_pos_world_x, new_robot_pos_world_x, 'euclidean', 'smallest', 1);
mean_error_desired_robot_pos_world_x = mean(error_desired_robot_pos_world_x);
error_desired_robot_pos_world_y = pdist2(new_desired_robot_pos_world_y, new_robot_pos_world_y, 'euclidean', 'smallest', 1);
mean_error_desired_robot_pos_world_y = mean(error_desired_robot_pos_world_y);
error_desired_robot_pos_world_z = pdist2(new_desired_robot_pos_world_z, new_robot_pos_world_z, 'euclidean', 'smallest', 1);
mean_error_desired_robot_pos_world_z = mean(error_desired_robot_pos_world_z);

error_desired_no_offset_robot_pos_world_x = pdist2(new_desired_no_offset_robot_pos_world_x, new_robot_pos_world_x, 'euclidean', 'smallest', 1);
mean_error_desired_no_offset_robot_pos_world_x = mean(error_desired_no_offset_robot_pos_world_x);
error_desired_no_offset_robot_pos_world_y = pdist2(new_desired_no_offset_robot_pos_world_y, new_robot_pos_world_y, 'euclidean', 'smallest', 1);
mean_error_desired_no_offset_robot_pos_world_y = mean(error_desired_no_offset_robot_pos_world_y);
error_desired_no_offset_robot_pos_world_z = pdist2(new_desired_no_offset_robot_pos_world_z, new_robot_pos_world_z, 'euclidean', 'smallest', 1);
mean_error_desired_no_offset_robot_pos_world_z = mean(error_desired_no_offset_robot_pos_world_z);


str_desired_x = {'Mean absolute error of:','- desired robot positon with offset vs robot position: ', num2str(mean_error_desired_robot_pos_world_x), ...
                 '- desired robot positon vs robot position: ', num2str(mean_error_desired_no_offset_robot_pos_world_x)};
text(xlim_x(2)+0.2,ylim_x(1),str_desired_x, 'VerticalAlignment','bottom', 'HorizontalAlignment','left', 'Parent', x_robot); 
  
str_desired_y = {'Mean absolute error of:','- desired robot positon with offset vs robot position: ', num2str(mean_error_desired_robot_pos_world_y), ...
                 '- desired robot positon vs robot position: ', num2str(mean_error_desired_no_offset_robot_pos_world_y)};
text(xlim_y(2)+0.2,ylim_y(1),str_desired_y, 'VerticalAlignment','bottom', 'HorizontalAlignment','left', 'Parent', y_robot); 

str_desired_z = {'Mean absolute error of:','- desired robot positon with offset vs robot position: ', num2str(mean_error_desired_robot_pos_world_z), ...
                 '- desired robot positon vs robot position: ', num2str(mean_error_desired_no_offset_robot_pos_world_z)};
text(xlim_z(2)+0.2,ylim_z(1),str_desired_z, 'VerticalAlignment','bottom', 'HorizontalAlignment','left', 'Parent', z_robot); 

disp('Saving...')
filename1 = ['robot_coords_world'];
filepath1 = fullfile(fileparts(which(file)), filename,filename1)
print(f1, filepath1, '-dpng', '-r300');
disp('Finished!')

%% ********Plot gt robot position & desired robot position******* 
f2=figure(2);
clf(f2);
x_robot = subplot(3,1,1);       % add first plot in 3 x 1 grid
plot(desired_no_offset_robot_pos_world_time, desired_no_offset_robot_pos_world_x, 'LineWidth',1, 'Color','b', 'LineStyle','-');
hold on
plot(robot_pos_world_time, robot_pos_world_x, 'LineWidth',1, 'Color','r', 'LineStyle','-');

title('X coordinate of robot vs. desired robot position in world frame');
xlabel('time [s]'); ylabel('coordinate [m]');
legend('desired robot pos', 'robot pos','Location', 'northeastoutside')
set(gca,'XMinorTick', 'on', 'YMinorTick', 'on','YGrid', 'on', 'YMinorGrid', 'on');
ylim_x=get(gca,'YLim');
xlim_x=get(gca,'XLim')
hold off

y_robot = subplot(3,1,2);       % add second plot in 3 x 1 grid
plot(desired_no_offset_robot_pos_world_time, desired_no_offset_robot_pos_world_y, 'LineWidth',1, 'Color','b', 'LineStyle','-');
hold on
plot(robot_pos_world_time, robot_pos_world_y, 'LineWidth',1, 'Color','r', 'LineStyle','-');

title('Y coordinate of robot vs. desired robot position in world frame');
xlabel('time [s]'); ylabel('coordinate [m]');
legend('desired robot pos', 'robot pos','Location', 'northeastoutside')
set(gca,'XMinorTick', 'on', 'YMinorTick', 'on','YGrid', 'on', 'YMinorGrid', 'on');
ylim_y=get(gca,'YLim');
xlim_y=get(gca,'XLim')
hold off

z_robot = subplot(3,1,3);       % add third plot in 3 x 1 grid
plot(desired_no_offset_robot_pos_world_time, desired_no_offset_robot_pos_world_z, 'LineWidth',1, 'Color','b', 'LineStyle','-');
hold on
plot(robot_pos_world_time, robot_pos_world_z, 'LineWidth',1, 'Color','r', 'LineStyle','-');

title('Z coordinate of robot vs. desired robot position in world frame');
xlabel('time [s]'); ylabel('coordinate [m]');
legend('desired robot pos', 'robot pos','Location', 'northeastoutside')
set(gca,'XMinorTick', 'on', 'YMinorTick', 'on','YGrid', 'on', 'YMinorGrid', 'on');
ylim_z=get(gca,'YLim');
xlim_z=get(gca,'XLim')
hold off

new_desired_no_offset_robot_pos_world_x = [desired_no_offset_robot_pos_world_time', desired_no_offset_robot_pos_world_x'];
new_desired_no_offset_robot_pos_world_y = [desired_no_offset_robot_pos_world_time', desired_no_offset_robot_pos_world_y'];
new_desired_no_offset_robot_pos_world_z = [desired_no_offset_robot_pos_world_time', desired_no_offset_robot_pos_world_z'];

start_time_commands = desired_no_offset_robot_pos_world_time(1)
start_idx = find(robot_pos_world_time >= start_time_commands);
start_idx(1)

new_robot_pos_world_x = [robot_pos_world_time(start_idx:end)', robot_pos_world_x(start_idx:end)'];
new_robot_pos_world_y = [robot_pos_world_time(start_idx:end)', robot_pos_world_y(start_idx:end)'];
new_robot_pos_world_z = [robot_pos_world_time(start_idx:end)', robot_pos_world_z(start_idx:end)'];

error_desired_no_offset_robot_pos_world_x = pdist2(new_desired_no_offset_robot_pos_world_x, new_robot_pos_world_x, 'euclidean', 'smallest', 1);
mean_error_desired_no_offset_robot_pos_world_x = mean(error_desired_no_offset_robot_pos_world_x);
error_desired_no_offset_robot_pos_world_y = pdist2(new_desired_no_offset_robot_pos_world_y, new_robot_pos_world_y, 'euclidean', 'smallest', 1);
mean_error_desired_no_offset_robot_pos_world_y = mean(error_desired_no_offset_robot_pos_world_y);
error_desired_no_offset_robot_pos_world_z = pdist2(new_desired_no_offset_robot_pos_world_z, new_robot_pos_world_z, 'euclidean', 'smallest', 1);
mean_error_desired_no_offset_robot_pos_world_z = mean(error_desired_no_offset_robot_pos_world_z);


str_desired_x = {'Mean absolute error:', num2str(mean_error_desired_no_offset_robot_pos_world_x)};
text(xlim_x(2)+2,ylim_x(1),str_desired_x, 'VerticalAlignment','bottom', 'HorizontalAlignment','left', 'Parent', x_robot); 
  
str_desired_y = {'Mean absolute error:', num2str(mean_error_desired_no_offset_robot_pos_world_y)};
text(xlim_y(2)+2,ylim_y(1),str_desired_y, 'VerticalAlignment','bottom', 'HorizontalAlignment','left', 'Parent', y_robot); 

str_desired_z = {'Mean absolute error:', num2str(mean_error_desired_no_offset_robot_pos_world_z)};
text(xlim_z(2)+2,ylim_z(1),str_desired_z, 'VerticalAlignment','bottom', 'HorizontalAlignment','left', 'Parent', z_robot); 

disp('Saving...')
filename1 = ['robot_coords_world_reduced'];
filepath1 = fullfile(fileparts(which(file)), filename,filename1)
print(f2, filepath1, '-dpng', '-r300');
disp('Finished!')
        
      
%% ********Get gt, measured & estimated cavity position in robot frame******* 
disp('figure 3...')
clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/firefly/rci/kalman/output/cavity_pose'});
msgs = readMessages(bagselect);

disp('Loaded /firefly/rci/kalman/output/cavity_pose from the bag file!')

% Do Calculations
est_cavity_pos_robot_x = []; est_cavity_pos_robot_y = []; est_cavity_pos_robot_z = [];
est_cavity_pos_robot_time = [];

for m = 1:length(msgs)  
    est_cavity_pos_robot_time = [est_cavity_pos_robot_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    est_cavity_pos_robot_x = [est_cavity_pos_robot_x, msgs{m}.Pose.Position.X];
    est_cavity_pos_robot_y = [est_cavity_pos_robot_y, msgs{m}.Pose.Position.Y];
    est_cavity_pos_robot_z = [est_cavity_pos_robot_z, msgs{m}.Pose.Position.Z];
end
disp('Saved /firefly/rci/kalman/output/cavity_pose to variables!')

clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/firefly/rci/cavity_normal/output/cavity_pose'});
msgs = readMessages(bagselect);
disp('Loaded /firefly/rci/cavity_normal/output/cavity_pose from the bag file!')

% Do Calculations
meas_cavity_pos_robot_x = []; meas_cavity_pos_robot_y = []; meas_cavity_pos_robot_z = [];
meas_cavity_pos_robot_time = [];
no_meas_cavity_pos_robot_time = [];

for m = 1:length(msgs)   
    if msgs{m}.Pose.Position.X ~= 0 && msgs{m}.Pose.Position.Y ~= 0 && msgs{m}.Pose.Position.Y ~= 0
        meas_cavity_pos_robot_time = [meas_cavity_pos_robot_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
        meas_cavity_pos_camera = [msgs{m}.Pose.Position.X, msgs{m}.Pose.Position.Y, msgs{m}.Pose.Position.Z];
        meas_cavity_pos_robot = camera_to_robot_rotation_rotm * meas_cavity_pos_camera';
        meas_cavity_pos_robot = meas_cavity_pos_robot + camera_to_robot_translation';
        meas_cavity_pos_robot_x = [meas_cavity_pos_robot_x, meas_cavity_pos_robot(1)];
        meas_cavity_pos_robot_y = [meas_cavity_pos_robot_y, meas_cavity_pos_robot(2)];
        meas_cavity_pos_robot_z = [meas_cavity_pos_robot_z, meas_cavity_pos_robot(3)];
    else
        no_meas_cavity_pos_robot_time = [no_meas_cavity_pos_robot_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    end
end
disp('Saved /firefly/rci/cavity_normal/output/cavity_pose to variables!')

gt_cavity_pos_robot_x = []; gt_cavity_pos_robot_y = []; gt_cavity_pos_robot_z = [];
for m = 1:length(robot_pos_world_time)   
    robot_pos_world = [robot_pos_world_x(m), robot_pos_world_y(m), robot_pos_world_z(m)];
    robot_or_world = [robot_or_world_w(m), robot_or_world_x(m), robot_or_world_y(m), robot_or_world_z(m)];
    gt_cavity_pos_robot = gt_cavity_pos_world - robot_pos_world;
    gt_cavity_pos_robot = quatrotate(robot_or_world, gt_cavity_pos_robot);
                    
    gt_cavity_pos_robot_x = [gt_cavity_pos_robot_x, gt_cavity_pos_robot(1)];
    gt_cavity_pos_robot_y = [gt_cavity_pos_robot_y, gt_cavity_pos_robot(2)];
    gt_cavity_pos_robot_z = [gt_cavity_pos_robot_z, gt_cavity_pos_robot(3)];
end
disp('Calculated cavity gt!')

%% ********Plot gt, meas & est cavity position in robot frame******* 
f3=figure(3);
clf(f3);
x_robot = subplot(3,1,1);       % add first plot in 3 x 1 grid
plot(robot_pos_world_time, gt_cavity_pos_robot_x, 'LineWidth',1, 'Color','k', 'LineStyle','-');
hold on
plot(est_cavity_pos_robot_time, est_cavity_pos_robot_x, 'LineWidth',1, 'Color','r', 'LineStyle','-');
plot(meas_cavity_pos_robot_time, meas_cavity_pos_robot_x, 'LineWidth',1, 'Color','b', 'LineStyle','none','Marker','*');

plot(no_meas_cavity_pos_robot_time, ones(1, length(no_meas_cavity_pos_robot_time))*1.1, 'Color','m', 'LineStyle','none','Marker','*');

title('X coord of  est vs meas vs. gt cavity position in robot frame');
xlabel('time [s]'); ylabel('coordinate [m]');
legend('Ground Truth','Kalman filter estimation','Measurement (from vision)','Lost cavity in vision','Location', 'northeastoutside')
set(gca,'XMinorTick', 'on', 'YMinorTick', 'on','YGrid', 'on', 'YMinorGrid', 'on');
ylim_x=get(gca,'YLim');
xlim_x=get(gca,'XLim');
hold off

y_robot = subplot(3,1,2);       % add second plot in 3 x 1 grid
plot(robot_pos_world_time, gt_cavity_pos_robot_y, 'LineWidth',1, 'Color','k', 'LineStyle','-');
hold on
plot(est_cavity_pos_robot_time, est_cavity_pos_robot_y, 'LineWidth',1, 'Color','r', 'LineStyle','-');
plot(meas_cavity_pos_robot_time, meas_cavity_pos_robot_y, 'LineWidth',1, 'Color','b', 'LineStyle','none','Marker','*');

plot(no_meas_cavity_pos_robot_time, ones(1, length(no_meas_cavity_pos_robot_time))*0.14, 'Color','m', 'LineStyle','none','Marker','*');

title('Y coord of  est vs meas vs. gt cavity position in robot frame');
xlabel('time [s]'); ylabel('coordinate [m]');
legend('Ground Truth','Kalman filter estimation','Measurement (from vision)','Lost cavity in vision','Location', 'northeastoutside')
set(gca,'XMinorTick', 'on', 'YMinorTick', 'on','YGrid', 'on', 'YMinorGrid', 'on');
ylim_y=get(gca,'YLim');
xlim_y=get(gca,'XLim');
hold off

z_robot = subplot(3,1,3);       % add third plot in 3 x 1 grid
plot(robot_pos_world_time, gt_cavity_pos_robot_z, 'LineWidth',1, 'Color','k', 'LineStyle','-');
hold on
plot(est_cavity_pos_robot_time, est_cavity_pos_robot_z, 'LineWidth',1, 'Color','r', 'LineStyle','-');
plot(meas_cavity_pos_robot_time, meas_cavity_pos_robot_z, 'LineWidth',1, 'Color','b', 'LineStyle','none','Marker','*');

plot(no_meas_cavity_pos_robot_time, ones(1, length(no_meas_cavity_pos_robot_time))*1.7, 'Color','m', 'LineStyle','none','Marker','*');

title('Z coord of est vs meas vs. gt cavity position in robot frame');
xlabel('time [s]'); ylabel('coordinate [m]');
legend('Ground Truth','Kalman filter estimation','Measurement (from vision)','Lost cavity in vision','Location', 'northeastoutside')
set(gca,'XMinorTick', 'on', 'YMinorTick', 'on','YGrid', 'on', 'YMinorGrid', 'on');
ylim_z=get(gca,'YLim');
xlim_z=get(gca,'XLim');
hold off

new_est_cavity_pos_robot_x = [est_cavity_pos_robot_time', est_cavity_pos_robot_x'];
new_est_cavity_pos_robot_y = [est_cavity_pos_robot_time', est_cavity_pos_robot_y'];
new_est_cavity_pos_robot_z = [est_cavity_pos_robot_time', est_cavity_pos_robot_z'];

new_meas_cavity_pos_robot_x = [meas_cavity_pos_robot_time', meas_cavity_pos_robot_x'];
new_meas_cavity_pos_robot_y = [meas_cavity_pos_robot_time', meas_cavity_pos_robot_y'];
new_meas_cavity_pos_robot_z = [meas_cavity_pos_robot_time', meas_cavity_pos_robot_z'];

start_time_commands = est_cavity_pos_robot_time(1);
start_idx = find(robot_pos_world_time >= start_time_commands);

new_gt_cavity_pos_robot_x = [robot_pos_world_time(start_idx:end)', gt_cavity_pos_robot_x(start_idx:end)'];
new_gt_cavity_pos_robot_y = [robot_pos_world_time(start_idx:end)', gt_cavity_pos_robot_y(start_idx:end)'];
new_gt_cavity_pos_robot_z = [robot_pos_world_time(start_idx:end)', gt_cavity_pos_robot_z(start_idx:end)'];

error_est_cavity_pos_x = pdist2(new_est_cavity_pos_robot_x, new_gt_cavity_pos_robot_x, 'euclidean', 'smallest', 1);
mean_error_est_cavity_pos_x = mean(error_est_cavity_pos_x);
error_est_cavity_pos_y = pdist2(new_est_cavity_pos_robot_y, new_gt_cavity_pos_robot_y, 'euclidean', 'smallest', 1);
mean_error_est_cavity_pos_y = mean(error_est_cavity_pos_y);
error_est_cavity_pos_z = pdist2(new_est_cavity_pos_robot_z, new_gt_cavity_pos_robot_z, 'euclidean', 'smallest', 1);
mean_error_est_cavity_pos_z = mean(error_est_cavity_pos_z);

str_desired_x = {'Mean absolute error of','Kalman filter estimation','compared to ground truth', num2str(mean_error_est_cavity_pos_x)};
text(xlim_x(2)+2,0.25,str_desired_x, 'VerticalAlignment','bottom', 'HorizontalAlignment','left', 'Parent', x_robot); 
  
str_desired_y = {'Mean absolute error of','Kalman filter estimation','compared to ground truth', num2str(mean_error_est_cavity_pos_y)};
text(xlim_y(2)+2,-0.75,str_desired_y, 'VerticalAlignment','bottom', 'HorizontalAlignment','left', 'Parent', y_robot); 

str_desired_z = {'Mean absolute error of','Kalman filter estimation','compared to ground truth', num2str(mean_error_est_cavity_pos_z)};
text(xlim_z(2)+2,-1.5,str_desired_z, 'VerticalAlignment','bottom', 'HorizontalAlignment','left', 'Parent', z_robot); 

disp('Saving...')
filename1 = ['cavity_coords_robot'];
filepath1 = fullfile(fileparts(which(file)), filename,filename1)
width = 12;     % Width in inches
height = 8;    % Height in inches
set(gcf,'InvertHardcopy','on');
set(gcf,'PaperUnits', 'inches');
papersize = get(gcf, 'PaperSize');
left = (papersize(1)- width)/2;
bottom = (papersize(2)- height)/2;
myfiguresize = [left, bottom, width, height];
set(gcf,'PaperPosition', myfiguresize);
print(f3, filepath1, '-dpng', '-r300');
disp('Finished!')

%%
close all