clear all

%% Load the bag file into memory
disp('Loading the bag file...')
filename = 'ex_sim3'
file = [filename,'.bag'];
filepath = fullfile(fileparts(which(file)), file);
bag = rosbag(filepath);

%% ********Plot ground truth endeffector position, gt robot position******* 
%% ********and estimate robot position in world frame**********************
clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/euroc6/vrpn_client/raw_transform'});
msgs = readMessages(bagselect);

disp('Loaded /euroc6/vrpn_client/raw_transform from the bag file!')

% Do Calculations
gt_robot_pos_world_x = []; gt_robot_pos_world_y = []; gt_robot_pos_world_z = [];
gt_robot_or_world_x = []; gt_robot_or_world_y = []; gt_robot_or_world_z = []; gt_robot_or_world_w = [];
gt_robot_pos_world_time = [];

for m = 1:length(msgs)   
    gt_robot_pos_world_time = [gt_robot_pos_world_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    gt_robot_pos_world_x = [gt_robot_pos_world_x, msgs{m}.Transform.Translation.X];
    gt_robot_pos_world_y = [gt_robot_pos_world_y, msgs{m}.Transform.Translation.Y];
    gt_robot_pos_world_z = [gt_robot_pos_world_z, msgs{m}.Transform.Translation.Z];
    gt_robot_or_world_x = [gt_robot_or_world_x, msgs{m}.Transform.Rotation.X];
    gt_robot_or_world_y = [gt_robot_or_world_y, msgs{m}.Transform.Rotation.Y];
    gt_robot_or_world_z = [gt_robot_or_world_z, msgs{m}.Transform.Rotation.Z];
    gt_robot_or_world_w = [gt_robot_or_world_w, msgs{m}.Transform.Rotation.W];
end
disp('Saved /euroc6/vrpn_client/raw_transform to variables!')
%%
clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/euroc6/msf_core/odometry'});
msgs = readMessages(bagselect);
disp('Loaded /euroc6/msf_core/odometry from the bag file!')

% Do Calculations
robot_pos_world_x = []; robot_pos_world_y = []; robot_pos_world_z = [];
robot_pos_world_time = [];

for m = 1:length(msgs)   
    robot_pos_world_time = [robot_pos_world_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    robot_pos_world_x = [robot_pos_world_x, msgs{m}.Pose.Pose.Position.X];
    robot_pos_world_y = [robot_pos_world_y, msgs{m}.Pose.Pose.Position.Y];
    robot_pos_world_z = [robot_pos_world_z, msgs{m}.Pose.Pose.Position.Z];
end
disp('Saved /euroc6/msf_core/odometry to variables!')

clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/ethz_manipulator/vrpn_client/estimated_odometry'});
msgs = readMessages(bagselect);
disp('Loaded /ethz_manipulator/vrpn_client/estimated_odometry from the bag file!')

% Do Calculations
end_pos_world_x = []; end_pos_world_y = []; end_pos_world_z = [];
end_pos_world_time = [];

for m = 1:length(msgs)   
    end_pos_world_time = [end_pos_world_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    end_pos_world_x = [end_pos_world_x, msgs{m}.Pose.Pose.Position.X];
    end_pos_world_y = [end_pos_world_y, msgs{m}.Pose.Pose.Position.Y];
    end_pos_world_z = [end_pos_world_z, msgs{m}.Pose.Pose.Position.Z];
end
disp('Saved /ethz_manipulator/vrpn_client/estimated_odometry to variable!')

% plot
f1=figure(1);
clf(f1);
subplot(3,1,1);       % add first plot in 3 x 1 grid
title('X coordinate of endeffector in world frame compared to ground truth robot X coordinate'   );
xlabel('time [s]'); ylabel('coordinate [m]');
plot(gt_robot_pos_world_time, gt_robot_pos_world_x, 'LineWidth',1, 'Color','r', 'LineStyle','-');
hold on
plot(robot_pos_world_time, robot_pos_world_x, 'LineWidth',1, 'Color','b', 'LineStyle','-');
plot(end_pos_world_time, end_pos_world_x, 'LineWidth',1, 'Color','g', 'LineStyle','-');
legend('gt robot pos','robot pos', 'gt end pos','Location', 'eastoutside')
hold off

subplot(3,1,2);       % add second plot in 3 x 1 grid
title('Y coordinate of endeffector in world frame compared to ground truth robot Y coordinate'   );
xlabel('time [s]'); ylabel('coordinate [m]');
plot(gt_robot_pos_world_time, gt_robot_pos_world_y, 'LineWidth',1, 'Color','r', 'LineStyle','-');
hold on
plot(robot_pos_world_time, robot_pos_world_y, 'LineWidth',1, 'Color','b', 'LineStyle','-');
plot(end_pos_world_time, end_pos_world_y, 'LineWidth',1, 'Color','g', 'LineStyle','-');
legend('gt robot pos','robot pos', 'gt end pos','Location', 'eastoutside')
hold off

subplot(3,1,3);       % add third plot in 3 x 1 grid
title('Z coordinate of endeffector in world frame compared to ground truth robot Z coordinate'   );
xlabel('time [s]'); ylabel('coordinate [m]');
plot(gt_robot_pos_world_time, gt_robot_pos_world_z, 'LineWidth',1, 'Color','r', 'LineStyle','-');
hold on
plot(robot_pos_world_time, robot_pos_world_z, 'LineWidth',1, 'Color','b', 'LineStyle','-');
plot(end_pos_world_time, end_pos_world_z, 'LineWidth',1, 'Color','g', 'LineStyle','-');
legend('gt robot pos','robot pos', 'gt end pos','Location', 'eastoutside')
hold off

arm_offset_z = gt_robot_pos_world_z - end_pos_world_z;
avg_arm_offset_z = mean(arm_offset_z)
disp('Finished!')

%% ********Plot ground truth endeffector position************************** 
%% ********and endeffector goal commands in robot frame********************
with_kalman_input = true;
clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/euroc6/rci/controller/output/arm_goal_pose'});
msgs = readMessages(bagselect);
disp('Loaded /euroc6/rci/controller/output/arm_goal_pose from the bag file!')

% Do Calculations
command_end_pos_robot_x = []; command_end_pos_robot_y = []; command_end_pos_robot_z = [];
command_end_pos_robot_time = [];

for m = 1:length(msgs)   
    command_end_pos_robot_time = [command_end_pos_robot_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    command_end_pos_robot_x = [command_end_pos_robot_x, msgs{m}.Pose.Position.X];
    command_end_pos_robot_y = [command_end_pos_robot_y, msgs{m}.Pose.Position.Y];
    command_end_pos_robot_z = [command_end_pos_robot_z, msgs{m}.Pose.Position.Z];
end
disp('Saved /euroc6/rci/controller/output/arm_goal_pose to variables!')

if(with_kalman_input)
clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/euroc6/rci/kalman/output/cavity_pose'});
msgs = readMessages(bagselect);
disp('Loaded /euroc6/rci/kalman/output/cavity_pose from the bag file!')

% Do Calculations
final_command_end_pos_robot_x = []; final_command_end_pos_robot_y = []; final_command_end_pos_robot_z = [];
final_command_end_pos_robot_time = [];

for m = 1:length(msgs)   
    final_command_end_pos_robot_time = [final_command_end_pos_robot_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    final_command_end_pos_robot_x = [final_command_end_pos_robot_x, msgs{m}.Pose.Position.X];
    final_command_end_pos_robot_y = [final_command_end_pos_robot_y, msgs{m}.Pose.Position.Y];
    final_command_end_pos_robot_z = [final_command_end_pos_robot_z, msgs{m}.Pose.Position.Z];
end
disp('Saved /euroc6/rci/kalman/output/cavity_pose to variables!')
end

% Do Calcuations
end_pos_robot_x = []; end_pos_robot_y = []; end_pos_robot_z = [];

for(i=1:length(gt_robot_pos_world_time))
    pos_end = [end_pos_world_x(i) , end_pos_world_y(i), end_pos_world_z(i)];
    pos_robot = [gt_robot_pos_world_x(i), gt_robot_pos_world_y(i), gt_robot_pos_world_z(i)];
    or_robot = [gt_robot_or_world_w(i), gt_robot_or_world_x(i), gt_robot_or_world_y(i), gt_robot_or_world_z(i)];
    rot = quat2rotm(or_robot);
    end_pos_robot = pos_end - pos_robot;
    end_pos_robot = inv(rot) * end_pos_robot';
    end_pos_robot_x = [end_pos_robot_x, end_pos_robot(1)]; 
    end_pos_robot_y = [end_pos_robot_y, end_pos_robot(2)]; 
    end_pos_robot_z = [end_pos_robot_z, end_pos_robot(3)]; 
end
disp('Calculated endeffector positio in robot frame!')

%plot
f2=figure(2);
clf(f2);
subplot(3,1,1);       % add first plot in 3 x 1 grid
title('X coordinate of endeffector in robot frame vs endeffector command');
xlabel('time [s]'); ylabel('coordinate [m]');
plot(gt_robot_pos_world_time, end_pos_robot_x, 'LineWidth',1, 'Color','r', 'LineStyle','-');
hold on
if(command_end_pos_robot_time(1) ~= 0)
    plot(command_end_pos_robot_time, command_end_pos_robot_x, 'LineWidth',1, 'Color','b', 'LineStyle','-');
    legend('end pos','command end pos','Location', 'eastoutside')
end
if(with_kalman_input)
    plot(final_command_end_pos_robot_time, final_command_end_pos_robot_x, 'LineWidth',1, 'Color','g', 'LineStyle','-');
    legend('end pos','command end pos', 'final_command end pos','Location', 'eastoutside')
end
hold off

subplot(3,1,2);       % add second plot in 3 x 1 grid
title('Y coordinate of endeffector in robot frame');
xlabel('time [s]'); ylabel('coordinate [m]');
plot(gt_robot_pos_world_time, end_pos_robot_y, 'LineWidth',1, 'Color','r', 'LineStyle','-');
hold on
if(command_end_pos_robot_time(1) ~= 0)
    plot(command_end_pos_robot_time, command_end_pos_robot_y, 'LineWidth',1, 'Color','b', 'LineStyle','-');
    legend('end pos','command end pos','Location', 'eastoutside')
end
if(with_kalman_input)
    plot(final_command_end_pos_robot_time, final_command_end_pos_robot_y, 'LineWidth',1, 'Color','g', 'LineStyle','-');
    legend('end pos','command end pos', 'final_command end pos','Location', 'eastoutside')
end
hold off

subplot(3,1,3);       % add third plot in 3 x 1 grid
title('Z coordinate of endeffector in robot frame');
xlabel('time [s]'); ylabel('coordinate [m]');
plot(gt_robot_pos_world_time, end_pos_robot_z, 'LineWidth',1, 'Color','r', 'LineStyle','-');
hold on
if(command_end_pos_robot_time(1) ~= 0)
    plot(command_end_pos_robot_time, command_end_pos_robot_z, 'LineWidth',1, 'Color','b', 'LineStyle','-');
    legend('end pos','command end pos','Location', 'eastoutside')
end
if(with_kalman_input)
    plot(final_command_end_pos_robot_time, final_command_end_pos_robot_z, 'LineWidth',1, 'Color','g', 'LineStyle','-');
    legend('end pos','command end pos', 'final_command end pos','Location', 'eastoutside')
end
hold off
disp('Finished!')

%% ********Plot joint state (arm angles) and jont commands*****************
clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/euroc6/joint_0_state'});
msgs = readMessages(bagselect);
disp('Loaded /euroc6/joint_0_state from the bag file!')

% Do Calculations
joint_state_0 = []; joint_state_0_time = [];

for m = 1:length(msgs)   
    joint_state_0_time = [joint_state_0_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    joint_state_0 = [joint_state_0, msgs{m}.Position];
end
disp('Saved /euroc6/joint_0_state to variables!')


clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/euroc6/joint_1_state'});
msgs = readMessages(bagselect);
disp('Loaded /euroc6/joint_1_state from the bag file!')

% Do Calculations
joint_state_1 = []; joint_state_1_time = [];

for m = 1:length(msgs)   
    joint_state_1_time = [joint_state_1_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    joint_state_1 = [joint_state_1, msgs{m}.Position];
end
disp('Saved /euroc6/joint_1_state to variables!')


clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/euroc6/joint_2_state'});
msgs = readMessages(bagselect);
disp('Loaded /euroc6/joint_2_state from the bag file!')

% Do Calculations
joint_state_2 = []; joint_state_2_time = [];

for m = 1:length(msgs)   
    joint_state_2_time = [joint_state_2_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    joint_state_2 = [joint_state_2, msgs{m}.Position];
end
disp('Saved /euroc6/joint_2_state to variables!')


clearvars bagselect msgs
bagselect = select(bag, 'Topic', {'/euroc6/joints_trajectory'});
msgs = readMessages(bagselect);
disp('Loaded /euroc6/joints_trajectory from the bag file!')

% Do Calculations
command_joint_state_1 = []; command_joint_state_2 = [];
command_joint_state_time = [];

for m = 1:length(msgs)   
    command_joint_state_time = [command_joint_state_time, double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9];
    [joint1,joint2] = msgs{m}.Points.Positions;
    command_joint_state_1 = [command_joint_state_1, joint1];
    command_joint_state_2 = [command_joint_state_2, joint2];
end
disp('Saved /euroc6/joints_trajectory to variables!')

%Plot
f3 = figure(3)
clf(f3);
subplot(3,1,1);       % add first plot in 3 x 1 grid
title('Joint state 0');
xlabel('time [s]'); ylabel('angle [rad]');
plot(joint_state_0_time, joint_state_0, 'LineWidth',1, 'Color','r', 'LineStyle','-');

subplot(3,1,2);       % add second plot in 3 x 1 grid
title('Joint state 1 vs Joint state 1 command');
xlabel('time [s]'); ylabel('angle [rad]');
plot(joint_state_1_time, joint_state_1, 'LineWidth',1, 'Color','r', 'LineStyle','-');
hold on
plot(command_joint_state_time, command_joint_state_1, 'LineWidth',1, 'Color','b', 'LineStyle','-');
legend('state_1','command_state_1','Location', 'eastoutside')
hold off

subplot(3,1,3);       % add third plot in 3 x 1 grid
title('Joint state 2 vs Joint state 2 command');
xlabel('time [s]'); ylabel('angle [rad]');
plot(joint_state_2_time, joint_state_2, 'LineWidth',1, 'Color','r', 'LineStyle','-');
hold on
plot(command_joint_state_time, command_joint_state_2, 'LineWidth',1, 'Color','b', 'LineStyle','-');
legend('state_2','command_state_2','Location', 'eastoutside')
hold off
disp('Finished!')

%%
clearvars bagselect msgs
%%
close all