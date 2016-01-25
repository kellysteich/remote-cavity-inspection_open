clear all

%% Load the bag file into memory
%filename = 'test_bag3';
filename = 'test_bag_more_lost';
file = [filename,'.bag'];
filepath = fullfile(fileparts(which(file)), file)
bag = rosbag(filepath)
bag.AvailableTopics
bagselect = select(bag, ...
    'Topic', {'/cavity/pose','/firefly/ground_truth/odometry','/firefly/odometry_sensor1/odometry'});
msgs = readMessages(bagselect);
size(msgs);
length(msgs)

set(0,'defaulttextinterpreter','latex');
forest_green = [3 163 255] / 255;
pink = [255 0 127] / 255;

ground_truth_cavity_position_world = [1.0,0,1.5];

camera_to_robot_translation = [0.145, 0.0, -0.07];
camera_to_robot_rotation =  [0 0 1; -1 0 0; 0 -1 0];
disp('Loaded the bag file!')

%% Set up figures
set(0,'DefaultFigureVisible','off');

errors_x = [];
errors_y = [];
errors_z = [];
errors = [];
errors_avg = [];

errors_x_gt = [];
errors_y_gt = [];
errors_z_gt = [];
errors_gt = [];
errors_avg_gt = [];
        
nrun = 0;
q_idx = 0;
r_idx = 0;
% Q = [100, 10, 1, 0.1, 0.01, 0.001, 0.0001, 0];
% R = [100, 10, 1, 0.1, 0.01, 0.001, 0.0001];
Q = [0];
R = [10];
for q = Q
    q_idx = q_idx + 1;
    r_idx = 0;
     for r = R
        r_idx = r_idx+1;
        nrun = nrun+1;
        output_text = ['Run ', num2str(nrun), '/56, Q=',num2str(q),' and R=', num2str(r), '!'];
        disp(output_text)

        disp('Setting up figures!')
        %% Figure 1: Cavity coords (groundtruth, measured and estimated) in robot frame
        f1=figure(1);
        clf(f1);
        x_coord_robot = subplot(3,1,1);       % add first plot in 3 x 1 grid
        title1_x  = title('\textbf{X coord of cavity in robot frame}'   );
        xlabel1_x = xlabel('time [s]'                                   );
        ylabel1_x = ylabel('x coordinate [m]'                           );
        
        y_coord_robot = subplot(3,1,2);       % add second plot in 3 x 1 grid
        title1_y  = title('\textbf{Y coord of cavity in robot frame}'   );
        xlabel1_y = xlabel('time [s]'                                   );
        ylabel1_y = ylabel('y coordinate [m]'                           );
        
        z_coord_robot = subplot(3,1,3);       % add third plot in 3 x 1 grid
        title1_z  = title('\textbf{Z coord of cavity in robot frame}'   );
        xlabel1_z = xlabel('time [s]'                                   );
        ylabel1_z = ylabel('z coordinate [m]'                           );
        
        plot_x_groundtruth_robot = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_robot   );
        plot_y_groundtruth_robot = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_robot   );
        plot_z_groundtruth_robot = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_robot   );
        
        plot_x_measured_robot = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_robot   );
        plot_y_measured_robot = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_robot   );
        plot_z_measured_robot = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_robot   );
        
        plot_x_estimated_robot = animatedline(      ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_robot   );
        plot_y_estimated_robot = animatedline(      ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_robot   );
        plot_z_estimated_robot = animatedline(      ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_robot   );
               
        uistack(plot_x_measured_robot,'top');
        uistack(plot_y_measured_robot,'top');
        uistack(plot_z_measured_robot,'top');
        uistack(plot_x_groundtruth_robot,'top');
        uistack(plot_y_groundtruth_robot,'top');
        uistack(plot_z_groundtruth_robot,'top');

        legend1_x = legend(x_coord_robot, ...
            [plot_x_groundtruth_robot, plot_x_measured_robot, plot_x_estimated_robot], ...
            'groundtruth','measured','estimated', 'Location','eastoutside');
        legend1_y = legend(y_coord_robot, ...
            [plot_y_groundtruth_robot, plot_y_measured_robot, plot_y_estimated_robot], ...
            'groundtruth','measured','estimated', 'Location','eastoutside');
        legend1_z = legend(z_coord_robot, ...
            [plot_z_groundtruth_robot, plot_z_measured_robot, plot_z_estimated_robot], ...
            'groundtruth','measured','estimated', 'Location','eastoutside');
        
        %% Figure 2: Cavity coords (groundtruth, measured and estimated) in world frame
        f2=figure(2);
        clf(f2);
        x_coord_world = subplot(3,1,1);       % add first plot in 3 x 1 grid
        title2_x  = title('\textbf{X coord of cavity in world frame}'   );
        xlabel2_x = xlabel('time [s]'                                   );
        ylabel2_x = ylabel('x coordinate [m]'                           );
        
        y_coord_world = subplot(3,1,2);       % add second plot in 3 x 1 grid
        title2_y  = title('\textbf{Y coord of cavity in world frame}'   );
        xlabel2_y = xlabel('time [s]'                                   );
        ylabel2_y = ylabel('y coordinate [m]'                           );
        
        z_coord_world = subplot(3,1,3);       % add third plot in 3 x 1 grid
        title2_z  = title('\textbf{Z coord of cavity in world frame}'   );
        xlabel2_z = xlabel('time [s]'                                   );
        ylabel2_z = ylabel('z coordinate [m]'                           );
        
        plot_x_groundtruth_world = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_world   );
        plot_y_groundtruth_world = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_world   );
        plot_z_groundtruth_world = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_world   );
        
        plot_x_measured_world = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_world   );
        plot_y_measured_world = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_world   );
        plot_z_measured_world = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_world   );
        
        plot_x_estimated_world = animatedline(      ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_world   );
        plot_y_estimated_world = animatedline(      ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_world   );
        plot_z_estimated_world = animatedline(      ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_world   );
        
        uistack(plot_x_measured_world,'top');
        uistack(plot_y_measured_world,'top');
        uistack(plot_z_measured_world,'top');
        uistack(plot_x_groundtruth_world,'top');
        uistack(plot_y_groundtruth_world,'top');
        uistack(plot_z_groundtruth_world,'top');
        
        legend2_x = legend(x_coord_world, ...
            [plot_x_groundtruth_world, plot_x_measured_world, plot_x_estimated_world], ...
            'groundtruth','measured','estimated', 'Location','eastoutside');
        legend2_y = legend(y_coord_world, ...
            [plot_y_groundtruth_world, plot_y_measured_world, plot_y_estimated_world], ...
            'groundtruth','measured','estimated', 'Location','eastoutside');
        legend2_z = legend(z_coord_world, ...
            [plot_z_groundtruth_world, plot_z_measured_world, plot_z_estimated_world], ...
            'groundtruthe','measured','estimated', 'Location','eastoutside');       
        
        %% Figure 3: Robot coords (groundtruth and measured) in world frame **
        f3=figure(3);
        clf(f3);
        x_coord = subplot(3,1,1);       % add first plot in 3 x 1 grid
        title3_x  = title('\textbf{X coord of robot in world frame}'    );
        xlabel3_x = xlabel('time [s]'                                   );
        ylabel3_x = ylabel('x coordinate [m]'                           );
        
        y_coord = subplot(3,1,2);       % add second plot in 3 x 1 grid
        title3_y  = title('\textbf{Y coord of robot in world frame}'    );
        xlabel3_y = xlabel('time [s]'                                   );
        ylabel3_y = ylabel('y coordinate [m]'                           );
        
        z_coord = subplot(3,1,3);       % add third plot in 3 x 1 grid
        title3_z  = title('\textbf{Z coord of robot in world frame}'    );
        xlabel3_z = xlabel('time [s]'                                   );
        ylabel3_z = ylabel('z coordinate [m]'                           );
        
        plot_x_groundtruth = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord         );
        plot_y_groundtruth = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord         );
        plot_z_groundtruth = animatedline(          ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord         );
        
        plot_x_measured = animatedline(             ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord         );
        plot_y_measured = animatedline(             ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord         );
        plot_z_measured = animatedline(             ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord         );
        
        uistack(plot_x_groundtruth,'top');
        uistack(plot_y_groundtruth,'top');
        uistack(plot_z_groundtruth,'top');
        
        legend3_x = legend(x_coord, ...
            [plot_x_groundtruth, plot_x_measured], ...
            'groundtruth','measured', 'Location','eastoutside');
        legend3_y = legend(y_coord, ...
            [plot_y_groundtruth, plot_y_measured], ...
            'groundtruth','measured', 'Location','eastoutside');
        legend3_z = legend(z_coord, ...
            [plot_z_groundtruth, plot_z_measured], ...
            'groundtruthe','measured', 'Location','eastoutside');
        
        %% Figure 4: Cavity coords (groundtruth, measured, estimated and cov) in robot frame
        f4=figure(4);
        clf(f4);
        x_coord_robot_all = subplot(3,1,1);       % add first plot in 3 x 1 grid
        title4_x  = title('\textbf{X coord of cavity in robot frame}'   );
        xlabel4_x = xlabel('time [s]'                                   );
        ylabel4_x = ylabel('x coordinate [m]'                           );
        
        y_coord_robot_all = subplot(3,1,2);       % add second plot in 3 x 1 grid
        title4_y  = title('\textbf{Y coord of cavity in robot frame}'   );
        xlabel4_y = xlabel('time [s]'                                   );
        ylabel4_y = ylabel('y coordinate [m]'                           );
        
        z_coord_robot_all = subplot(3,1,3);       % add third plot in 3 x 1 grid
        title4_z  = title('\textbf{Z coord of cavity in robot frame}'   );
        xlabel4_z = xlabel('time [s]'                                   );
        ylabel4_z = ylabel('z coordinate [m]'                           );
        
        plot_x_groundtruth_robot_all = animatedline(...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_robot_all);
        plot_y_groundtruth_robot_all = animatedline(...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_robot_all);
        plot_z_groundtruth_robot_all = animatedline(...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_robot_all);
        
        plot_x_measured_robot_all = animatedline(   ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_robot_all);
        plot_y_measured_robot_all = animatedline(   ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_robot_all);
        plot_z_measured_robot_all = animatedline(   ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_robot_all);
        
        plot_x_estimated_robot_all = animatedline(  ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_robot_all);
        plot_y_estimated_robot_all = animatedline(  ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_robot_all);
        plot_z_estimated_robot_all = animatedline(  ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_robot_all);
        
        plot_x_pos_covariance = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                pink,           ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_robot_all);
        plot_y_pos_covariance = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                pink,           ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_robot_all);
        plot_z_pos_covariance = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                pink,           ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_robot_all);
        
        plot_x_neg_covariance = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                pink,           ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_robot_all);
        plot_y_neg_covariance = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                pink,           ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_robot_all);
        plot_z_neg_covariance = animatedline(       ...
            'LineWidth',            1,              ...
            'Color',                pink,           ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_robot_all);
               
        uistack(plot_x_measured_robot_all,'top');
        uistack(plot_y_measured_robot_all,'top');
        uistack(plot_z_measured_robot_all,'top');
        uistack(plot_x_groundtruth_robot_all,'top');
        uistack(plot_y_groundtruth_robot_all,'top');
        uistack(plot_z_groundtruth_robot_all,'top');
        uistack(plot_x_pos_covariance,'top');
        uistack(plot_y_pos_covariance,'top');
        uistack(plot_z_pos_covariance,'top');
        uistack(plot_x_neg_covariance,'top');
        uistack(plot_y_neg_covariance,'top');
        uistack(plot_z_neg_covariance,'top');
        
        legend4_x = legend(x_coord_robot_all, ...
            [plot_x_groundtruth_robot_all, plot_x_measured_robot_all, plot_x_estimated_robot_all, plot_x_pos_covariance], ...
            'groundtruth','measured','estimated','\pm std dev', 'Location','eastoutside');
        legend4_y = legend(y_coord_robot_all, ...
            [plot_y_groundtruth_robot_all, plot_y_measured_robot_all, plot_y_estimated_robot_all, plot_y_pos_covariance], ...
            'groundtruth','measured','estimated','\pm std dev', 'Location','eastoutside');
        legend4_z = legend(z_coord_robot_all, ...
            [plot_z_groundtruth_robot_all, plot_z_measured_robot_all, plot_z_estimated_robot_all, plot_z_pos_covariance], ...
            'groundtruth','measured','estimated','\pm std dev', 'Location','eastoutside');
        
        %% Figure 5: Cavity coords (groundtruth, measured and groundtruth_estimated) in robot frame
        f5=figure(5);
        clf(f5);
        x_coord_robot_gt = subplot(3,1,1);       % add first plot in 3 x 1 grid
        title5_x  = title('\textbf{X coord of cavity in robot frame}'   );
        xlabel5_x = xlabel('time [s]'                                   );
        ylabel5_x = ylabel('x coordinate [m]'                           );
        
        y_coord_robot_gt = subplot(3,1,2);       % add second plot in 3 x 1 grid
        title5_y  = title('\textbf{Y coord of cavity in robot frame}'   );
        xlabel5_y = xlabel('time [s]'                                   );
        ylabel5_y = ylabel('y coordinate [m]'                           );
        
        z_coord_robot_gt = subplot(3,1,3);       % add third plot in 3 x 1 grid
        title5_z  = title('\textbf{Z coord of cavity in robot frame}'   );
        xlabel5_z = xlabel('time [s]'                                   );
        ylabel5_z = ylabel('z coordinate [m]'                           );
        
        plot_x_groundtruth_robot_gt = animatedline( ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_robot_gt);
        plot_y_groundtruth_robot_gt = animatedline( ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_robot_gt);
        plot_z_groundtruth_robot_gt = animatedline( ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_robot_gt);
        
        plot_x_measured_robot_gt = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_robot_gt);
        plot_y_measured_robot_gt = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_robot_gt);
        plot_z_measured_robot_gt = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_robot_gt);
        
        plot_x_estimated_robot_gt = animatedline(   ...
            'LineWidth',            2,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_robot_gt);
        plot_y_estimated_robot_gt = animatedline(   ...
            'LineWidth',            2,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_robot_gt);
        plot_z_estimated_robot_gt = animatedline(   ...
            'LineWidth',            2,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_robot_gt);
               
        uistack(plot_x_measured_robot_gt,'top');
        uistack(plot_y_measured_robot_gt,'top');
        uistack(plot_z_measured_robot_gt,'top');
        uistack(plot_x_groundtruth_robot_gt,'top');
        uistack(plot_y_groundtruth_robot_gt,'top');
        uistack(plot_z_groundtruth_robot_gt,'top');

        legend5_x = legend(x_coord_robot_gt, ...
            [plot_x_groundtruth_robot_gt, plot_x_measured_robot_gt, plot_x_estimated_robot_gt], ...
            'groundtruth','measured','estimated with gt', 'Location','eastoutside');
        legend5_y = legend(y_coord_robot_gt, ...
            [plot_y_groundtruth_robot_gt, plot_y_measured_robot_gt, plot_y_estimated_robot_gt], ...
            'groundtruth','measured','estimated with gt', 'Location','eastoutside');
        legend5_z = legend(z_coord_robot_gt, ...
            [plot_z_groundtruth_robot_gt, plot_z_measured_robot_gt, plot_z_estimated_robot_gt], ...
            'groundtruth','measured','estimated with gt', 'Location','eastoutside');
        
        %% Figure 6: Cavity coords (groundtruth, measured and groundtruth_estimated) in world frame
        f6=figure(6);
        clf(f6);
        x_coord_world_gt = subplot(3,1,1);       % add first plot in 3 x 1 grid
        title6_x  = title('\textbf{X coord of cavity in world frame}'   );
        xlabel6_x = xlabel('time [s]'                                   );
        ylabel6_x = ylabel('x coordinate [m]'                           );
        
        y_coord_world_gt = subplot(3,1,2);       % add second plot in 3 x 1 grid
        title6_y  = title('\textbf{Y coord of cavity in world frame}'   );
        xlabel6_y = xlabel('time [s]'                                   );
        ylabel6_y = ylabel('y coordinate [m]'                           );
        
        z_coord_world_gt = subplot(3,1,3);       % add third plot in 3 x 1 grid
        title6_z  = title('\textbf{Z coord of cavity in world frame}'   );
        xlabel6_z = xlabel('time [s]'                                   );
        ylabel6_z = ylabel('z coordinate [m]'                           );
        
        plot_x_groundtruth_world_gt = animatedline( ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_world_gt);
        plot_y_groundtruth_world_gt = animatedline( ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_world_gt);
        plot_z_groundtruth_world_gt = animatedline( ...
            'LineWidth',            1,              ...
            'Color',                'k',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_world_gt);
        
        plot_x_measured_world_gt = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_world_gt);
        plot_y_measured_world_gt = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_world_gt);
        plot_z_measured_world_gt = animatedline(    ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_world_gt);
        
        plot_x_estimated_world_gt = animatedline(   ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_coord_world_gt);
        plot_y_estimated_world_gt = animatedline(   ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_coord_world_gt);
        plot_z_estimated_world_gt = animatedline(   ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_coord_world_gt);
               
        uistack(plot_x_measured_world_gt,'top');
        uistack(plot_y_measured_world_gt,'top');
        uistack(plot_z_measured_world_gt,'top');
        uistack(plot_x_groundtruth_world_gt,'top');
        uistack(plot_y_groundtruth_world_gt,'top');
        uistack(plot_z_groundtruth_world_gt,'top');

        legend6_x = legend(x_coord_world_gt, ...
            [plot_x_groundtruth_world_gt, plot_x_measured_world_gt, plot_x_estimated_world_gt], ...
            'groundtruth','measured','estimated with gt', 'Location','eastoutside');
        legend6_y = legend(y_coord_world_gt, ...
            [plot_y_groundtruth_world_gt, plot_y_measured_world_gt, plot_y_estimated_world_gt], ...
            'groundtruth','measured','estimated with gt', 'Location','eastoutside');
        legend6_z = legend(z_coord_world_gt, ...
            [plot_z_groundtruth_world_gt, plot_z_measured_world_gt, plot_z_estimated_world_gt], ...
            'groundtruth','measured','estimated with gt', 'Location','eastoutside');
        
        %% Figure 7: Error (groundtruth vs. measured and groundtruth vs. estimated) in robot frame
        f7=figure(7);
        clf(f7);
        x_error = subplot(4,1,1);       % add first plot in 3 x 1 grid
        title7_x  = title('\textbf{Error of X coord of cavity in robot frame}'   );
        xlabel7_x = xlabel('time [s]'                                   );
        ylabel7_x = ylabel('error [m]'                           );
        
        y_error = subplot(4,1,2);       % add second plot in 3 x 1 grid
        title7_y  = title('\textbf{Error of Y coord of cavity in robot frame}'   );
        xlabel7_y = xlabel('time [s]'                                   );
        ylabel7_y = ylabel('error [m]'                           );
        
        z_error = subplot(4,1,3);       % add third plot in 3 x 1 grid
        title7_z  = title('\textbf{Error of Z coord of cavity in robot frame}'   );
        xlabel7_z = xlabel('time [s]'                                   );
        ylabel7_z = ylabel('error [m]'                           );
        
        error = subplot(4,1,4);       % add fourth plot in 4 x 1 grid
        title7  = title('\textbf{Error of coord of cavity in robot frame}'   );
        xlabel7 = xlabel('time [s]'                                   );
        ylabel7 = ylabel('error [m]'                           );
        
        plot_x_error_est = animatedline(            ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_error);
        plot_y_error_est = animatedline(            ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_error);
        plot_z_error_est = animatedline(            ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_error);
        plot_error_est = animatedline(              ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               error);
        
        plot_x_error_meas = animatedline(           ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_error);
        plot_y_error_meas = animatedline(           ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_error);
        plot_z_error_meas = animatedline(           ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_error);
        plot_error_meas = animatedline(             ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               error);

        legend7_x = legend(x_error, ...
            [plot_x_error_est, plot_x_error_meas], ...
            'estimated state error','measured state error', 'Location','eastoutside');
        legend7_y = legend(y_error, ...
            [plot_y_error_est, plot_y_error_meas], ...
            'estimated state error','measured state error', 'Location','eastoutside');
        legend7_z = legend(z_error, ...
            [plot_z_error_est, plot_z_error_meas], ...
            'estimated state error','measured state error', 'Location','eastoutside');
        legend7 = legend(error, ...
            [plot_error_est, plot_error_meas], ...
            'estimated state error','measured state error', 'Location','eastoutside');
        
        %% Figure 8: Error (groundtruth vs. measured and groundtruth vs. ground_truth_estimated) in robot frame
        f8=figure(8);
        clf(f8);
        x_error_gt = subplot(4,1,1);       % add first plot in 3 x 1 grid
        title8_x  = title('\textbf{Error of X coord of cavity in robot frame}'   );
        xlabel8_x = xlabel('time [s]'                                   );
        ylabel8_x = ylabel('error [m]'                           );
        
        y_error_gt = subplot(4,1,2);       % add second plot in 3 x 1 grid
        title8_y  = title('\textbf{Error of Y coord of cavity in robot frame}'   );
        xlabel8_y = xlabel('time [s]'                                   );
        ylabel8_y = ylabel('error [m]'                           );
        
        z_error_gt = subplot(4,1,3);       % add third plot in 3 x 1 grid
        title8_z  = title('\textbf{Error of Z coord of cavity in robot frame}'   );
        xlabel8_z = xlabel('time [s]'                                   );
        ylabel8_z = ylabel('error [m]'                           );
        
        error_gt = subplot(4,1,4);       % add fourth plot in 4 x 1 grid
        title8  = title('\textbf{Error of coord of cavity in robot frame}'   );
        xlabel8 = xlabel('time [s]'                                   );
        ylabel8 = ylabel('error [m]'                           );
        
        plot_x_error_est_gt = animatedline(            ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_error_gt);
        plot_y_error_est_gt = animatedline(            ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_error_gt);
        plot_z_error_est_gt = animatedline(            ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_error_gt);
        plot_error_est_gt = animatedline(              ...
            'LineWidth',            1,              ...
            'Color',                'r',            ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               error_gt);
        
        plot_x_error_meas_gt = animatedline(           ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               x_error_gt);
        plot_y_error_meas_gt = animatedline(           ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               y_error_gt);
        plot_z_error_meas_gt = animatedline(           ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               z_error_gt);
        plot_error_meas_gt = animatedline(             ...
            'LineWidth',            1,              ...
            'Color',                forest_green,   ...
            'LineStyle',            '-',            ...
            'MaximumNumPoints',     length(msgs),   ...
            'Parent',               error_gt);

        legend8_x = legend(x_error_gt, ...
            [plot_x_error_est_gt, plot_x_error_meas_gt], ...
            'estimated state error','measured state error', 'Location','eastoutside');
        legend8_y = legend(y_error_gt, ...
            [plot_y_error_est_gt, plot_y_error_meas_gt], ...
            'estimated state error','measured state error', 'Location','eastoutside');
        legend8_z = legend(z_error_gt, ...
            [plot_z_error_est_gt, plot_z_error_meas_gt], ...
            'estimated state error','measured state error', 'Location','eastoutside');
        legend8 = legend(error_gt, ...
            [plot_error_est_gt, plot_error_meas_gt], ...
            'estimated state error','measured state error', 'Location','eastoutside');
        
        %% Do Calculations
        
        [gotFirstMeas, gotFirstPose, gotFirstGTPose] = deal(false);
        
        [estimated_cavity_position_robot, estimated_cavity_position_world] = deal([]);       
        cavity_position_covariance = eye(3);
        
        [estimated_cavity_position_robot_gt, estimated_cavity_position_world_gt] = deal([]);       
        cavity_position_covariance_gt = eye(3);
        
        [measured_cavity_position_camera, measured_cavity_orientation_camera] = deal([]);       
        [measured_cavity_position_robot, measured_cavity_position_world, measured_cavity_position_world_gt] = deal([]);
        
        ground_truth_cavity_position_robot = [];
        
        
        [ground_truth_robot_position_world, ground_truth_robot_orientation_world] = deal([]);              
        [measured_robot_position_world, measured_robot_orientation_world] = deal([]);        
        [new_measured_robot_position_world, new_measured_robot_orientation_world] = deal([]);
        
        
        [time_ground_truth,  time_meas, time_first_meas] = deal(0);
               
        
        no_detected_cavity = [];
        
        %set white process noise covariance matrix to q
        Kalman_Q = q*eye(3);
        %set white measurement noise covariance matrix to r
        Kalman_R = r*eye(3);
        
        %%
        disp('Doing Calculations')
        for m = 1:length(msgs)
            
            if strcmp(msgs{m}.MessageType, 'nav_msgs/Odometry')
         
                if strcmp(msgs{m}.ChildFrameId,'firefly/base_link')
                    %% The msg is a grundtruth robot pose
                    time_ground_truth = double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9;
                    
                    new_ground_truth_robot_position_world = ...
                        [msgs{m}.Pose.Pose.Position.X, ...
                         msgs{m}.Pose.Pose.Position.Y, ...
                         msgs{m}.Pose.Pose.Position.Z];
                    new_ground_truth_robot_orientation_world = ...
                        [msgs{m}.Pose.Pose.Orientation.W, ...
                         msgs{m}.Pose.Pose.Orientation.X, ...
                         msgs{m}.Pose.Pose.Orientation.Y, ...
                         msgs{m}.Pose.Pose.Orientation.Z];
                    
                    %compute groundtruth cavity pose in robot frame
                    ground_truth_cavity_position_robot = ground_truth_cavity_position_world - new_ground_truth_robot_position_world;
                    ground_truth_cavity_position_robot = quatrotate(new_ground_truth_robot_orientation_world, ground_truth_cavity_position_robot);
                    
                    %Figure 1: Plot for ground truth cavity position in robot frame
                    addpoints(plot_x_groundtruth_robot, time_ground_truth, ground_truth_cavity_position_robot(1));
                    addpoints(plot_y_groundtruth_robot, time_ground_truth, ground_truth_cavity_position_robot(2));
                    addpoints(plot_z_groundtruth_robot, time_ground_truth, ground_truth_cavity_position_robot(3));
                    
                    %Figure 2: Plot for ground truth cavity position in world frame
                    addpoints(plot_x_groundtruth_world, time_ground_truth, ground_truth_cavity_position_world(1));
                    addpoints(plot_y_groundtruth_world, time_ground_truth, ground_truth_cavity_position_world(2));
                    addpoints(plot_z_groundtruth_world, time_ground_truth, ground_truth_cavity_position_world(3));
                    
                    %Fiure 3: Plot for ground truth robot position in world frame
                    addpoints(plot_x_groundtruth, time_ground_truth, new_ground_truth_robot_position_world(1));
                    addpoints(plot_y_groundtruth, time_ground_truth, new_ground_truth_robot_position_world(2));
                    addpoints(plot_z_groundtruth, time_ground_truth, new_ground_truth_robot_position_world(3));
                    
                    %Figure 4: Plot for ground truth cavity position in robot frame
                    addpoints(plot_x_groundtruth_robot_all, time_ground_truth, ground_truth_cavity_position_robot(1));
                    addpoints(plot_y_groundtruth_robot_all, time_ground_truth, ground_truth_cavity_position_robot(2));
                    addpoints(plot_z_groundtruth_robot_all, time_ground_truth, ground_truth_cavity_position_robot(3));
                    
                    %Figure 5: Plot for ground truth cavity position in robot frame
                    addpoints(plot_x_groundtruth_robot_gt, time_ground_truth, ground_truth_cavity_position_robot(1));
                    addpoints(plot_y_groundtruth_robot_gt, time_ground_truth, ground_truth_cavity_position_robot(2));
                    addpoints(plot_z_groundtruth_robot_gt, time_ground_truth, ground_truth_cavity_position_robot(3));
                    
                    %Figure 6: Plot for ground truth cavity position in world frame
                    addpoints(plot_x_groundtruth_world_gt, time_ground_truth, ground_truth_cavity_position_world(1));
                    addpoints(plot_y_groundtruth_world_gt, time_ground_truth, ground_truth_cavity_position_world(2));
                    addpoints(plot_z_groundtruth_world_gt, time_ground_truth, ground_truth_cavity_position_world(3))
                    
                    if gotFirstGTPose
                        %compute transformation from world to robot frame k-1
                        gt_rotation_kminus1 = quat2rotm(ground_truth_robot_orientation_world);
                        gt_translation_kminus1 = ground_truth_robot_position_world';
                        
                        %compute transformation from world to robot k
                        gt_rotation_k = quat2rotm(new_ground_truth_robot_orientation_world);
                        gt_translation_k = new_ground_truth_robot_position_world';
                        
                        %compute transformation from robot frame k-1 to robot frame k
                        gt_rotation_kminus12k = gt_rotation_k \ gt_rotation_kminus1;
                        gt_translation_kminus12k = gt_translation_kminus1 - gt_translation_k;
                    else
                        gotFirstGTPose = true;
                    end
                    
                    %update robot pose k to robot pose k+1
                    ground_truth_robot_position_world = new_ground_truth_robot_position_world;
                    ground_truth_robot_orientation_world = new_ground_truth_robot_orientation_world;
                    
                    if gotFirstMeas
                        %Do Kalman filter predition update (compute estimated cavity pose with gt in robot frame)
                        estimated_cavity_position_robot_gt = ...
                            gt_rotation_kminus12k * estimated_cavity_position_robot_gt + gt_translation_kminus12k;
                        if q == 0 %if q=0 use the covariance (of the rotation) given by the robot pose
                            gt_robot_cov = msgs{m}.Pose.Covariance;
                            Kalman_Q = [gt_robot_cov(22:24) gt_robot_cov(28:30) gt_robot_cov(34:36)];
                        end
                        cavity_position_covariance_gt = ...
                            gt_rotation_kminus12k * cavity_position_covariance_gt * gt_rotation_kminus12k.' + Kalman_Q;
                        
                        %compute estimated cavity pose with gt in world frame
                        estimated_cavity_position_world_gt = quatrotate(quatinv(ground_truth_robot_orientation_world), estimated_cavity_position_robot_gt');
                        estimated_cavity_position_world_gt = estimated_cavity_position_world_gt + ground_truth_robot_position_world;
                        
                        notPoseMsg = true;
                        showInPlot = true;
                        c = 1;
                        %check if there will be a non zero cavity pose msg between this msg
                        %and the next groundtruth robot pose msg
                        while notPoseMsg && m+c < length(msgs)
                            if strcmp(msgs{m+c}.MessageType, 'nav_msgs/Odometry')
                                if strcmp(msgs{m+c}.ChildFrameId,'firefly/base_link')
                                    notPoseMsg = false;
                                    break
                                end
                            elseif strcmp(msgs{m+c}.MessageType, 'geometry_msgs/PoseStamped')
                                if msgs{m+c}.Pose.Position.X ~= 0 && msgs{m+c}.Pose.Position.Y ~= 0 && msgs{m+c}.Pose.Position.Z ~= 0
                                    showInPlot = false;
                                    break
                                end
                            end
                            c = c+1;
                        end
                        if showInPlot
                            %Figure 5: Plot for estimated cavity position using groundtruth in robot frame
                            addpoints(plot_x_estimated_robot_gt, time_ground_truth, estimated_cavity_position_robot_gt(1));
                            addpoints(plot_y_estimated_robot_gt, time_ground_truth, estimated_cavity_position_robot_gt(2));
                            addpoints(plot_z_estimated_robot_gt, time_ground_truth, estimated_cavity_position_robot_gt(3));
                            
                            %Figure 6: Plot for estimated cavity position using groundtruth in world frame
                            addpoints(plot_x_estimated_world_gt, time_ground_truth, estimated_cavity_position_world_gt(1));
                            addpoints(plot_y_estimated_world_gt, time_ground_truth, estimated_cavity_position_world_gt(2));
                            addpoints(plot_z_estimated_world_gt, time_ground_truth, estimated_cavity_position_world_gt(3));
                            
                            %Figure 8: Error for gt estimated state in robot frame
                            ex = abs(ground_truth_cavity_position_robot(1) - estimated_cavity_position_robot_gt(1));
                            ey = abs(ground_truth_cavity_position_robot(2) - estimated_cavity_position_robot_gt(2));
                            ez = abs(ground_truth_cavity_position_robot(3) - estimated_cavity_position_robot_gt(3));
                            e = sqrt(ex*ex + ey*ey + ez*ez);
                            addpoints(plot_x_error_est_gt, time_ground_truth, ex);
                            addpoints(plot_y_error_est_gt, time_ground_truth, ey);
                            addpoints(plot_z_error_est_gt, time_ground_truth, ez);
                            addpoints(plot_error_est_gt, time_ground_truth, e);
                        else
                        end
                    end
                %%
                else
                    %% The msg is a measured robot pose
                    time_meas = double(msgs{m}.Header.Stamp.Sec)+double(msgs{m}.Header.Stamp.Nsec)*10^-9;
                    
                    new_measured_robot_position_world = ...
                        [msgs{m}.Pose.Pose.Position.X, ...
                         msgs{m}.Pose.Pose.Position.Y, ...
                         msgs{m}.Pose.Pose.Position.Z];
                    new_measured_robot_orientation_world = ...
                        [msgs{m}.Pose.Pose.Orientation.W, ...
                         msgs{m}.Pose.Pose.Orientation.X, ...
                         msgs{m}.Pose.Pose.Orientation.Y, ...
                         msgs{m}.Pose.Pose.Orientation.Z];
                    
                    if gotFirstPose
                        %compute transformation from world to robot frame k-1
                        rotation_kminus1 = quat2rotm(measured_robot_orientation_world);
                        translation_kminus1 = measured_robot_position_world';
                        
                        %compute transformation from world to robot k
                        rotation_k = quat2rotm(new_measured_robot_orientation_world);
                        translation_k = new_measured_robot_position_world';
                        
                        %compute transformation from robot frame k-1 to robot frame k
                        rotation_kminus12k = rotation_k \ rotation_kminus1;
                        translation_kminus12k = translation_kminus1 - translation_k;
                    else
                        gotFirstPose = true;
                    end
                    
                    %update robot pose k to robot pose k+1
                    measured_robot_position_world = new_measured_robot_position_world;
                    measured_robot_orientation_world = new_measured_robot_orientation_world;
                    
                    if gotFirstMeas
                        %Do Kalman filter predition update (compute estimated cavity pose in robot frame)
                        estimated_cavity_position_robot = ...
                            rotation_kminus12k * estimated_cavity_position_robot + translation_kminus12k;
                        if q == 0 %if q=0 use the covariance (of the rotation) given by the robot pose
                            meas_robot_cov = msgs{m}.Pose.Covariance;
                            Kalman_Q = [meas_robot_cov(22:24) meas_robot_cov(28:30) meas_robot_cov(34:36)];
                        end
                        cavity_position_covariance = ...
                            rotation_kminus12k * cavity_position_covariance * rotation_kminus12k.' + Kalman_Q;
                        
                        %compute estimated cavity pose in world frame
                        estimated_cavity_position_world = quatrotate(quatinv(measured_robot_orientation_world), estimated_cavity_position_robot');
                        estimated_cavity_position_world = estimated_cavity_position_world + measured_robot_position_world;
                        
                        notPoseMsg = true;
                        showInPlot = true;
                        c = 1;
                        %check if there will be a non zero cavity pose msg between this msg
                        %and the next robot pose msg
                        while notPoseMsg && m+c < length(msgs)
                            if strcmp(msgs{m+c}.MessageType, 'nav_msgs/Odometry')
                                if strcmp(msgs{m+c}.ChildFrameId,'firefly/odometry_sensor1')
                                    notPoseMsg = false;
                                    break
                                end
                            elseif strcmp(msgs{m+c}.MessageType, 'geometry_msgs/PoseStamped')
                                if msgs{m+c}.Pose.Position.X ~= 0 && msgs{m+c}.Pose.Position.Y ~= 0 && msgs{m+c}.Pose.Position.Z ~= 0
                                    showInPlot = false;
                                    break
                                end
                            end
                            c = c+1;
                        end
                        if showInPlot
                            %Figure 1: Plot for estimated cavity position in robot frame
                            addpoints(plot_x_estimated_robot, time_meas, estimated_cavity_position_robot(1));
                            addpoints(plot_y_estimated_robot, time_meas, estimated_cavity_position_robot(2));
                            addpoints(plot_z_estimated_robot, time_meas, estimated_cavity_position_robot(3));
                            
                            %Figure 2: Plot for estimated cavity position in world frame
                            addpoints(plot_x_estimated_world, time_meas, estimated_cavity_position_world(1));
                            addpoints(plot_y_estimated_world, time_meas, estimated_cavity_position_world(2));
                            addpoints(plot_z_estimated_world, time_meas, estimated_cavity_position_world(3));
                            
                            %Figure 4: Plot for estimated cavity position in robot frame
                            addpoints(plot_x_estimated_robot_all, time_meas, estimated_cavity_position_robot(1));
                            addpoints(plot_y_estimated_robot_all, time_meas, estimated_cavity_position_robot(2));
                            addpoints(plot_z_estimated_robot_all, time_meas, estimated_cavity_position_robot(3));
                            
                            %Figure 4: Plot for positive standard deviation
                            addpoints(plot_x_pos_covariance, time_meas, sqrt(cavity_position_covariance(1,1)));
                            addpoints(plot_y_pos_covariance, time_meas, sqrt(cavity_position_covariance(2,2)));
                            addpoints(plot_z_pos_covariance, time_meas, sqrt(cavity_position_covariance(3,3)));
                            
                            %Figure 4: Plot for negative standard deviation
                            addpoints(plot_x_neg_covariance, time_meas, -1*sqrt(cavity_position_covariance(1,1)));
                            addpoints(plot_y_neg_covariance, time_meas, -1*sqrt(cavity_position_covariance(2,2)));
                            addpoints(plot_z_neg_covariance, time_meas, -1*sqrt(cavity_position_covariance(3,3)));
                            
                            %Figure 7: Error for estimated state in robot frame
                            ex = abs(ground_truth_cavity_position_robot(1) - estimated_cavity_position_robot(1));
                            ey = abs(ground_truth_cavity_position_robot(2) - estimated_cavity_position_robot(2));
                            ez = abs(ground_truth_cavity_position_robot(3) - estimated_cavity_position_robot(3));
                            e = sqrt(ex*ex + ey*ey + ez*ez);
                            addpoints(plot_x_error_est, time_meas, ex);
                            addpoints(plot_y_error_est, time_meas, ey);
                            addpoints(plot_z_error_est, time_meas, ez);
                            addpoints(plot_error_est, time_meas, e);
                        else
                        end
                    end
                    
                    %Figure 3: Plot for measured robot position in world frame
                    addpoints(plot_x_measured, time_meas, measured_robot_position_world(1));
                    addpoints(plot_y_measured, time_meas, measured_robot_position_world(2));
                    addpoints(plot_z_measured, time_meas, measured_robot_position_world(3));
                    
                    %%For movie of plot uncomment
                    %drawnow
                    %%
                end
            else
                %% The msg is a cavity pose
                measured_cavity_position_camera = ...
                    [msgs{m}.Pose.Position.X, ...
                     msgs{m}.Pose.Position.Y, ...
                     msgs{m}.Pose.Position.Z];
                measured_cavity_orientation_camera = ...
                    [msgs{m}.Pose.Orientation.W, ...
                     msgs{m}.Pose.Orientation.X, ...
                     msgs{m}.Pose.Orientation.Y, ...
                     msgs{m}.Pose.Orientation.Z];
                
                if ~isequal(measured_cavity_position_camera, [0 0 0])
                    %compute measured cavity pose in robot frame
                    measured_cavity_position_robot = camera_to_robot_rotation * measured_cavity_position_camera';
                    measured_cavity_position_robot = measured_cavity_position_robot + camera_to_robot_translation';
                    
                    %compute measured cavity pose in world frame
                    measured_cavity_position_world = quatrotate(quatinv(measured_robot_orientation_world), measured_cavity_position_robot');
                    measured_cavity_position_world = measured_cavity_position_world + measured_robot_position_world;
                    
                    %compute measured cavity pose with gt in world frame
                    measured_cavity_position_world_gt = quatrotate(quatinv(ground_truth_robot_orientation_world), measured_cavity_position_robot');
                    measured_cavity_position_world_gt = measured_cavity_position_world_gt + ground_truth_robot_position_world;
                    
                    if gotFirstMeas
                        %Do Kalman filter correction update (compute estimated cavity pose in robot frame)
                        Kalman_Gain = cavity_position_covariance / (cavity_position_covariance + Kalman_R);
                        estimated_cavity_position_robot = estimated_cavity_position_robot + Kalman_Gain * (measured_cavity_position_robot- estimated_cavity_position_robot);
                        cavity_position_covariance = (eye(3) - Kalman_Gain) * cavity_position_covariance;
                        
                        %compute estimated cavity pose in world frame
                        estimated_cavity_position_world = quatrotate(quatinv(measured_robot_orientation_world), estimated_cavity_position_robot');
                        estimated_cavity_position_world = estimated_cavity_position_world + measured_robot_position_world;
                        
                        %Do Kalman filter correction update (compute estimated cavity pose with gt in robot frame)
                        Kalman_Gain_gt = cavity_position_covariance_gt / (cavity_position_covariance_gt + Kalman_R);
                        estimated_cavity_position_robot_gt = estimated_cavity_position_robot_gt + Kalman_Gain_gt * (measured_cavity_position_robot- estimated_cavity_position_robot_gt);
                        cavity_position_covariance_gt = (eye(3) - Kalman_Gain_gt) * cavity_position_covariance_gt;
                        
                        %compute estimated cavity pose with gt in world frame
                        estimated_cavity_position_world_gt = quatrotate(quatinv(ground_truth_robot_orientation_world), estimated_cavity_position_robot_gt');
                        estimated_cavity_position_world_gt = estimated_cavity_position_world_gt + ground_truth_robot_position_world;
                    else
                        %initialize Kalman filter with first measurement
                        gotFirstMeas = true;
                        estimated_cavity_position_robot = measured_cavity_position_robot;
                        estimated_cavity_position_world = measured_cavity_position_world;
                        estimated_cavity_position_robot_gt = measured_cavity_position_robot;
                        estimated_cavity_position_world_gt = measured_cavity_position_world_gt;
                        time_first_meas = time_meas;
                    end
                    
                    %Figure 1: Plot for measured cavity position in robot frame
                    addpoints(plot_x_measured_robot, time_meas, measured_cavity_position_robot(1));
                    addpoints(plot_y_measured_robot, time_meas, measured_cavity_position_robot(2));
                    addpoints(plot_z_measured_robot, time_meas, measured_cavity_position_robot(3));
                    
                    %Figure 2: Plot for measured cavity position in world frame
                    addpoints(plot_x_measured_world, time_meas, measured_cavity_position_world(1));
                    addpoints(plot_y_measured_world, time_meas, measured_cavity_position_world(2));
                    addpoints(plot_z_measured_world, time_meas, measured_cavity_position_world(3));
                    
                    %Figure 1: Plot for estimated cavity position in robot frame
                    addpoints(plot_x_estimated_robot, time_meas, estimated_cavity_position_robot(1));
                    addpoints(plot_y_estimated_robot, time_meas, estimated_cavity_position_robot(2));
                    addpoints(plot_z_estimated_robot, time_meas, estimated_cavity_position_robot(3));
                    
                    %Figure 2: Plot for estimated cavity position in world frame
                    addpoints(plot_x_estimated_world, time_meas, estimated_cavity_position_world(1));
                    addpoints(plot_y_estimated_world, time_meas, estimated_cavity_position_world(2));
                    addpoints(plot_z_estimated_world, time_meas, estimated_cavity_position_world(3));
                    
                    %Figure 4: Plot for measured cavity position in robot frame
                    addpoints(plot_x_measured_robot_all, time_meas, measured_cavity_position_robot(1));
                    addpoints(plot_y_measured_robot_all, time_meas, measured_cavity_position_robot(2));
                    addpoints(plot_z_measured_robot_all, time_meas, measured_cavity_position_robot(3));
                    
                    %Figure 4: Plot for estimated cavity position in robot frame
                    addpoints(plot_x_estimated_robot_all, time_meas, estimated_cavity_position_robot(1));
                    addpoints(plot_y_estimated_robot_all, time_meas, estimated_cavity_position_robot(2));
                    addpoints(plot_z_estimated_robot_all, time_meas, estimated_cavity_position_robot(3));
                    
                    %Figure 4: Plot for positive standard deviation
                    addpoints(plot_x_pos_covariance, time_meas, sqrt(cavity_position_covariance(1,1)));
                    addpoints(plot_y_pos_covariance, time_meas, sqrt(cavity_position_covariance(2,2)));
                    addpoints(plot_z_pos_covariance, time_meas, sqrt(cavity_position_covariance(3,3)));
                    
                    %Figure 4: Plot for negative standard deviation
                    addpoints(plot_x_neg_covariance, time_meas, -1*sqrt(cavity_position_covariance(1,1)));
                    addpoints(plot_y_neg_covariance, time_meas, -1*sqrt(cavity_position_covariance(2,2)));
                    addpoints(plot_z_neg_covariance, time_meas, -1*sqrt(cavity_position_covariance(3,3)));
                    
                    %Figure 5: Plot for measured cavity position in robot frame
                    addpoints(plot_x_measured_robot_gt, time_meas, measured_cavity_position_robot(1));
                    addpoints(plot_y_measured_robot_gt, time_meas, measured_cavity_position_robot(2));
                    addpoints(plot_z_measured_robot_gt, time_meas, measured_cavity_position_robot(3));
                    
                    %Figure 6: Plot for measured cavity position with gt in world frame
                    addpoints(plot_x_measured_world_gt, time_meas, measured_cavity_position_world_gt(1));
                    addpoints(plot_y_measured_world_gt, time_meas, measured_cavity_position_world_gt(2));
                    addpoints(plot_z_measured_world_gt, time_meas, measured_cavity_position_world_gt(3));
                    
                    %Figure 5: Plot for estimated cavity position with gt in robot frame
                    addpoints(plot_x_estimated_robot_gt, time_meas, estimated_cavity_position_robot_gt(1));
                    addpoints(plot_y_estimated_robot_gt, time_meas, estimated_cavity_position_robot_gt(2));
                    addpoints(plot_z_estimated_robot_gt, time_meas, estimated_cavity_position_robot_gt(3));
                    
                    %Figure 6: Plot for estimated cavity position with gt in world frame
                    addpoints(plot_x_estimated_world_gt, time_meas, estimated_cavity_position_world_gt(1));
                    addpoints(plot_y_estimated_world_gt, time_meas, estimated_cavity_position_world_gt(2));
                    addpoints(plot_z_estimated_world_gt, time_meas, estimated_cavity_position_world_gt(3));
                    
                    %Figure 7: Error for estimated state in robot frame
                    ex = abs(ground_truth_cavity_position_robot(1) - estimated_cavity_position_robot(1));
                    ey = abs(ground_truth_cavity_position_robot(2) - estimated_cavity_position_robot(2));
                    ez = abs(ground_truth_cavity_position_robot(3) - estimated_cavity_position_robot(3));
                    e = sqrt(ex*ex + ey*ey + ez*ez);
                    addpoints(plot_x_error_est, time_meas, ex);
                    addpoints(plot_y_error_est, time_meas, ey);
                    addpoints(plot_z_error_est, time_meas, ez);
                    addpoints(plot_error_est, time_meas, e);
                    
                    %Figure 7: Error for measured state in robot frame
                    ex = abs(ground_truth_cavity_position_robot(1) - measured_cavity_position_robot(1));
                    ey = abs(ground_truth_cavity_position_robot(2) - measured_cavity_position_robot(2));
                    ez = abs(ground_truth_cavity_position_robot(3) - measured_cavity_position_robot(3));
                    e = sqrt(ex*ex + ey*ey + ez*ez);
                    addpoints(plot_x_error_meas, time_meas, ex);
                    addpoints(plot_y_error_meas, time_meas, ey);
                    addpoints(plot_z_error_meas, time_meas, ez);
                    addpoints(plot_error_meas, time_meas, e);
                    
                    %Figure 8: Error for measured state in robot frame
                    addpoints(plot_x_error_meas_gt, time_meas, ex);
                    addpoints(plot_y_error_meas_gt, time_meas, ey);
                    addpoints(plot_z_error_meas_gt, time_meas, ez);
                    addpoints(plot_error_meas_gt, time_meas, e);
                    
                    %Figure 8: Error for gt estimated state in robot frame
                    ex = abs(ground_truth_cavity_position_robot(1) - estimated_cavity_position_robot_gt(1));
                    ey = abs(ground_truth_cavity_position_robot(2) - estimated_cavity_position_robot_gt(2));
                    ez = abs(ground_truth_cavity_position_robot(3) - estimated_cavity_position_robot_gt(3));
                    e = sqrt(ex*ex + ey*ey + ez*ez);
                    addpoints(plot_x_error_est_gt, time_meas, ex);
                    addpoints(plot_y_error_est_gt, time_meas, ey);
                    addpoints(plot_z_error_est_gt, time_meas, ez);
                    addpoints(plot_error_est_gt, time_meas, e);
                else
                    %save the times where the cavity was lost, i.e. cavity
                    %position = [0 0 0]
                    no_detected_cavity = [no_detected_cavity, time_meas];
                end
                %%
            end
        end
        
        %% Set up figure for saving
        disp('Setting up for saving')
        %% Add times when cavity position was not detected
        lrobotx = get(x_coord_robot, 'YLim');
        lroboty = get(y_coord_robot, 'YLim');
        lrobotz = get(z_coord_robot, 'YLim');
        
        lworldx = get(x_coord_world, 'YLim');
        lworldy = get(y_coord_world, 'YLim');
        lworldz = get(z_coord_world, 'YLim');
        
        lrobotx_all = get(x_coord_robot_all, 'YLim');
        lroboty_all = get(y_coord_robot_all, 'YLim');
        lrobotz_all = get(z_coord_robot_all, 'YLim');
        
        lrobotx_gt = get(x_coord_robot_gt, 'YLim');
        lroboty_gt = get(y_coord_robot_gt, 'YLim');
        lrobotz_gt = get(z_coord_robot_gt, 'YLim');
        
        lworldx_gt = get(x_coord_world_gt, 'YLim');
        lworldy_gt = get(y_coord_world_gt, 'YLim');
        lworldz_gt = get(z_coord_world_gt, 'YLim');
        
        lerrorx = get(x_error, 'YLim');
        lerrory = get(y_error, 'YLim');
        lerrorz = get(z_error, 'YLim');
        lerror = get(error, 'YLim');
        
        lerrorx_gt = get(x_error_gt, 'YLim');
        lerrory_gt = get(y_error_gt, 'YLim');
        lerrorz_gt = get(z_error_gt, 'YLim');
        lerror_gt = get(error_gt, 'YLim');
        
        for i = 1:length(no_detected_cavity)
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lrobotx(2)-lrobotx(2)*0.01 lrobotx(2)-lrobotx(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', x_coord_robot);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lroboty(2)-lroboty(2)*0.01 lroboty(2)-lroboty(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', y_coord_robot);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lrobotz(2)-lrobotz(2)*0.01 lrobotz(2)-lrobotz(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', z_coord_robot);
            
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lworldx(2)-lworldx(2)*0.01 lworldx(2)-lworldx(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', x_coord_world);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lworldy(2)-lworldy(2)*0.01 lworldy(2)-lworldy(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', y_coord_world);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lworldz(2)-lworldz(2)*0.01 lworldz(2)-lworldz(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', z_coord_world);
            
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lrobotx_all(2)-lrobotx_all(2)*0.01 lrobotx_all(2)-lrobotx_all(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', x_coord_robot_all);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lroboty_all(2)-lroboty_all(2)*0.01 lroboty_all(2)-lroboty_all(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', y_coord_robot_all);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lrobotz_all(2)-lrobotz_all(2)*0.01 lrobotz_all(2)-lrobotz_all(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', z_coord_robot_all);
            
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lrobotx_gt(2)-lrobotx_gt(2)*0.01 lrobotx_gt(2)-lrobotx_gt(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', x_coord_robot_gt);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lroboty_gt(2)-lroboty_gt(2)*0.01 lroboty_gt(2)-lroboty_gt(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', y_coord_robot_gt);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lrobotz_gt(2)-lrobotz_gt(2)*0.01 lrobotz_gt(2)-lrobotz_gt(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', z_coord_robot_gt);
            
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lworldx_gt(2)-lworldx_gt(2)*0.01 lworldx_gt(2)-lworldx_gt(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', x_coord_world_gt);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lworldy_gt(2)-lworldy_gt(2)*0.01 lworldy_gt(2)-lworldy_gt(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', y_coord_world_gt);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lworldz_gt(2)-lworldz_gt(2)*0.01 lworldz_gt(2)-lworldz_gt(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', z_coord_world_gt);
            
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lerrorx(2)-lerrorx(2)*0.01 lerrorx(2)-lerrorx(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', x_error);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lerrory(2)-lerrory(2)*0.01 lerrory(2)-lerrory(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', y_error);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lerrorz(2)-lerrorz(2)*0.01 lerrorz(2)-lerrorz(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', z_error);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lerror(2)-lerror(2)*0.01 lerror(2)-lerror(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', error);
            
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lerrorx_gt(2)-lerrorx_gt(2)*0.01 lerrorx_gt(2)-lerrorx_gt(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', x_error_gt);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lerrory_gt(2)-lerrory_gt(2)*0.01 lerrory_gt(2)-lerrory_gt(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', y_error_gt);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lerrorz_gt(2)-lerrorz_gt(2)*0.01 lerrorz_gt(2)-lerrorz_gt(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', z_error_gt);
            line('XData', [no_detected_cavity(i)-0.04 no_detected_cavity(i)+0.04], 'YData', [lerror_gt(2)-lerror_gt(2)*0.01 lerror_gt(2)-lerror_gt(2)*0.01], 'LineStyle', '-', ...
                'LineWidth', 5, 'Color','m', 'Parent', error_gt);
        end
        %%
        %% Set up font sizes
        set([x_coord_robot, y_coord_robot, z_coord_robot,               ...
            x_coord_world, y_coord_world, z_coord_world,                ...
            x_coord, y_coord, z_coord,                                  ...
            x_coord_robot_all, y_coord_robot_all, z_coord_robot_all,    ...
            x_coord_robot_gt, y_coord_robot_gt, z_coord_robot_gt,       ...
            x_coord_world_gt, y_coord_world_gt, z_coord_world_gt,       ...
            x_error, y_error, z_error, error,                           ...
            x_error_gt, y_error_gt, z_error_gt, error_gt], ...
            'XMinorTick', 'on', 'YMinorTick', 'on',...
            'XLim', [time_first_meas - 5, time_meas + 1],...
            'YGrid', 'on', 'YMinorGrid', 'on');
        
        fontsize_legend = 8;
        fontsize_label = 10;
        fontsize_title = 12;
        
        set([legend1_x, legend1_y, legend1_z,                           ...
             x_coord_robot, y_coord_robot, z_coord_robot,               ...
             legend2_x, legend2_y, legend2_z,                           ...
             x_coord_world, y_coord_world, z_coord_world,               ...
             legend3_x, legend3_y, legend3_z,                           ...
             x_coord, y_coord, z_coord,                                 ...
             legend4_x, legend4_y, legend4_z,                           ...
             x_coord_robot_all, y_coord_robot_all, z_coord_robot_all,   ...
             legend5_x, legend5_y, legend5_z,                           ...
             x_coord_robot_gt, y_coord_robot_gt, z_coord_robot_gt,      ...
             legend6_x, legend6_y, legend6_z,                           ...
             x_coord_world_gt, y_coord_world_gt, z_coord_world_gt,      ...
             legend7_x, legend7_y, legend7_z, legend7,                  ...
             x_error, y_error, z_error, error,                          ...
             legend8_x, legend8_y, legend8_z, legend8,                  ...
             x_error_gt, y_error_gt, z_error_gt, error_gt], ...
            'FontSize'   , fontsize_legend      );
        
        set([xlabel1_x, xlabel1_y, xlabel1_z,               ...
            ylabel1_x, ylabel1_y, ylabel1_z,                ...
            xlabel2_x, xlabel2_y, xlabel2_z,                ...
            ylabel2_x, ylabel2_y, ylabel2_z,                ...
            xlabel3_x, xlabel3_y, xlabel3_z,                ...
            ylabel3_x, ylabel3_y, ylabel3_z,                ...
            xlabel4_x, xlabel4_y, xlabel4_z,                ...
            ylabel4_x, ylabel4_y, ylabel4_z,                ...
            xlabel5_x, xlabel5_y, xlabel5_z,                ...
            ylabel5_x, ylabel5_y, ylabel5_z,                ...
            xlabel6_x, xlabel6_y, xlabel6_z,                ...
            ylabel6_x, ylabel6_y, ylabel6_z,                ...
            xlabel7_x, xlabel7_y, xlabel7_z, xlabel7,       ...
            ylabel7_x, ylabel7_y, ylabel7_z, ylabel7,       ...
            xlabel8_x, xlabel8_y, xlabel8_z, xlabel8,       ...
            ylabel8_x, ylabel8_y, ylabel8_z, ylabel8], ...
            'FontSize'   , fontsize_label       );
        
        set([title1_x, title1_y, title1_z,          ...
             title2_x, title2_y, title2_z,          ...
             title3_x, title3_y, title3_z,          ...
             title4_x, title4_y, title4_z,          ...
             title5_x, title5_y, title5_z,          ...
             title6_x, title6_y, title6_z,          ...
             title7_x, title7_y, title7_z, title7,  ...
             title8_x, title8_y, title8_z, title8],...
            'FontSize'   , fontsize_title       );
        %%
        %% Set up mean error display
        error_avg = 0;
        error_avg_gt = 0;
        
        [~, error_m] = getpoints(plot_x_error_meas);
        [~, error_e] = getpoints(plot_x_error_est);
        [~, error_e_gt] = getpoints(plot_x_error_est_gt);
        mean_error_m = mean(error_m);
        mean_error_e = mean(error_e);
        mean_error_e_gt = mean(error_e_gt);       
        
        str_meas = ['Mean absolute error of measurement: ', num2str(mean_error_m)];
        str_est = ['Mean absolute error of estimate: ', num2str(mean_error_e)];
        str_est_gt = ['Mean absolute error of estimate: ', num2str(mean_error_e_gt)];
        
        str = {str_meas, str_est};
        lx = get(x_error, 'XLim');
        ly = get(x_error, 'YLim');
        text(lx(1) + lx(1)*0.01, ly(2) - ly(2)*0.2, str, 'Fontsize', fontsize_legend, 'Parent', x_error);
        
        str_gt = {str_meas, str_est_gt};
        lx = get(x_error_gt, 'XLim');
        ly = get(x_error_gt, 'YLim');
        text(lx(1) + lx(1)*0.01, ly(2) - ly(2)*0.2, str_gt, 'Fontsize', fontsize_legend, 'Parent', x_error_gt);
        
        errors_x(q_idx, r_idx) = mean_error_e;
        errors_x_gt(q_idx, r_idx) = mean_error_e_gt;
        error_avg = error_avg + mean_error_e;
        error_avg_gt = error_avg_gt + mean_error_e_gt;
        
        
        [~, error_m] = getpoints(plot_y_error_meas);
        [~, error_e] = getpoints(plot_y_error_est);
        [~, error_e_gt] = getpoints(plot_y_error_est_gt);
        mean_error_m = mean(error_m);
        mean_error_e = mean(error_e);
        mean_error_e_gt = mean(error_e_gt);       
        
        str_meas = ['Mean absolute error of measurement: ', num2str(mean_error_m)];
        str_est = ['Mean absolute error of estimate: ', num2str(mean_error_e)];
        str_est_gt = ['Mean absolute error of estimate: ', num2str(mean_error_e_gt)];
        
        str = {str_meas, str_est};
        lx = get(y_error, 'XLim');
        ly = get(y_error, 'YLim');
        text(lx(1) + lx(1)*0.01, ly(2) - ly(2)*0.2, str, 'Fontsize', fontsize_legend, 'Parent', y_error);
        
        str_gt = {str_meas, str_est_gt};
        lx = get(y_error_gt, 'XLim');
        ly = get(y_error_gt, 'YLim');
        text(lx(1) + lx(1)*0.01, ly(2) - ly(2)*0.2, str_gt, 'Fontsize', fontsize_legend, 'Parent', y_error_gt);
        
        errors_y(q_idx, r_idx) = mean_error_e;
        errors_y_gt(q_idx, r_idx) = mean_error_e_gt;
        error_avg = error_avg + mean_error_e;
        error_avg_gt = error_avg_gt + mean_error_e_gt;
        
        
        [~, error_m] = getpoints(plot_z_error_meas);
        [~, error_e] = getpoints(plot_z_error_est);
        [~, error_e_gt] = getpoints(plot_z_error_est_gt);
        mean_error_m = mean(error_m);
        mean_error_e = mean(error_e);
        mean_error_e_gt = mean(error_e_gt);       
        
        str_meas = ['Mean absolute error of measurement: ', num2str(mean_error_m)];
        str_est = ['Mean absolute error of estimate: ', num2str(mean_error_e)];
        str_est_gt = ['Mean absolute error of estimate: ', num2str(mean_error_e_gt)];
        
        str = {str_meas, str_est};
        lx = get(z_error, 'XLim');
        ly = get(z_error, 'YLim');
        text(lx(1) + lx(1)*0.01, ly(2) - ly(2)*0.2, str, 'Fontsize', fontsize_legend, 'Parent', z_error);
        
        str_gt = {str_meas, str_est_gt};
        lx = get(z_error_gt, 'XLim');
        ly = get(z_error_gt, 'YLim');
        text(lx(1) + lx(1)*0.01, ly(2) - ly(2)*0.2, str_gt, 'Fontsize', fontsize_legend, 'Parent', z_error_gt);
        
        errors_z(q_idx, r_idx) = mean_error_e;
        errors_z_gt(q_idx, r_idx) = mean_error_e_gt;
        error_avg = error_avg + mean_error_e;
        error_avg_gt = error_avg_gt + mean_error_e_gt;
        
        
        [~, error_m] = getpoints(plot_error_meas);
        [~, error_e] = getpoints(plot_error_est);
        [~, error_e_gt] = getpoints(plot_error_est_gt);
        mean_error_m = mean(error_m);
        mean_error_e = mean(error_e);
        mean_error_e_gt = mean(error_e_gt);       
        
        str_meas = ['Mean absolute error of measurement: ', num2str(mean_error_m)];
        str_est = ['Mean absolute error of estimate: ', num2str(mean_error_e)];
        str_est_gt = ['Mean absolute error of estimate: ', num2str(mean_error_e_gt)];
        
        str = {str_meas, str_est};
        lx = get(error, 'XLim');
        ly = get(error, 'YLim');
        text(lx(1) + lx(1)*0.01, ly(2) - ly(2)*0.2, str, 'Fontsize', fontsize_legend, 'Parent', error);
        
        str_gt = {str_meas, str_est_gt};
        lx = get(error_gt, 'XLim');
        ly = get(error_gt, 'YLim');
        text(lx(1) + lx(1)*0.01, ly(2) - ly(2)*0.2, str_gt, 'Fontsize', fontsize_legend, 'Parent', error_gt);

        errors(q_idx, r_idx) = mean_error_e;
        errors_gt(q_idx, r_idx) = mean_error_e_gt;
        error_avg = error_avg + mean_error_e;
        error_avg_gt = error_avg_gt + mean_error_e_gt;
        
        errors_avg(q_idx, r_idx) = error_avg;
        errors_avg_gt(q_idx, r_idx) = error_avg_gt;
        
        %% Save the figures
        disp('Saving...')
        filename_addition = strcat('_',filename,'_Q_',num2str(q),'_R_',num2str(r));
        
        set(f1, 'PaperUnits', 'centimeters');
        set(f1, 'PaperSize', [25.6 19.2]);
        set(f1, 'PaperPosition', [0, 0, 25.6, 19.2]);
        set(f1, 'PaperPositionMode', 'Manual');
        filename1 = ['cavity_coords_robot_gt_meas_est',filename_addition];
        filepath1 = fullfile(fileparts(which(file)), filename,'cavity_coords_robot',filename1);
        print(f1, filepath1, '-dpdf', '-r300');
        
        set(f2, 'PaperUnits', 'centimeters');
        set(f2, 'PaperSize', [25.6 19.2]);
        set(f2, 'PaperPosition', [0, 0, 25.6, 19.2]);
        set(f2, 'PaperPositionMode', 'Manual');
        filename2 = ['cavity_coords_world_gt_meas_est',filename_addition];
        filepath2 = fullfile(fileparts(which(file)), filename, 'cavity_coords_world',filename2);
        print(f2, filepath2, '-dpdf', '-r300');
        
        set(f3, 'PaperUnits', 'centimeters');
        set(f3, 'PaperSize', [25.6 19.2]);
        set(f3, 'PaperPosition', [0, 0, 25.6, 19.2]);
        set(f3, 'PaperPositionMode', 'Manual');
        filename3 = ['robot_coords_world_gt_meas','_',filename];
        filepath3 = fullfile(fileparts(which(file)), filename, 'robot_coords_world',filename3);
        print(f3, filepath3, '-dpdf', '-r300');
        
        set(f4, 'PaperUnits', 'centimeters');
        set(f4, 'PaperSize', [25.6 19.2]);
        set(f4, 'PaperPosition', [0, 0, 25.6, 19.2]);
        set(f4, 'PaperPositionMode', 'Manual');
        filename4 = ['cavity_coords_robot_gt_meas_est_cov',filename_addition];
        filepath4 = fullfile(fileparts(which(file)), filename, 'cavity_coords_robot_all',filename4);
        print(f4, filepath4, '-dpdf', '-r300');
        
        set(f5, 'PaperUnits', 'centimeters');
        set(f5, 'PaperSize', [25.6 19.2]);
        set(f5, 'PaperPosition', [0, 0, 25.6, 19.2]);
        set(f5, 'PaperPositionMode', 'Manual');
        filename5 = ['cavity_coords_robot_gt_meas_gtest',filename_addition];
        filepath5 = fullfile(fileparts(which(file)), filename, 'cavity_coords_robot_gt',filename5);
        print(f5, filepath5, '-dpdf', '-r300');
        
        set(f6, 'PaperUnits', 'centimeters');
        set(f6, 'PaperSize', [25.6 19.2]);
        set(f6, 'PaperPosition', [0, 0, 25.6, 19.2]);
        set(f6, 'PaperPositionMode', 'Manual');
        filename6 = ['cavity_coords_world_gt_meas_gtest',filename_addition];
        filepath6 = fullfile(fileparts(which(file)), filename, 'cavity_coords_world_gt',filename6);
        print(f6, filepath6, '-dpdf', '-r300');
        
        set(f7, 'PaperUnits', 'centimeters');
        set(f7, 'PaperSize', [25.6 19.2]);
        set(f7, 'PaperPosition', [0, 0, 25.6, 19.2]);
        set(f7, 'PaperPositionMode', 'Manual');
        filename7 = ['error',filename_addition];
        filepath7 = fullfile(fileparts(which(file)), filename, 'error',filename7);
        print(f7, filepath7, '-dpdf', '-r300');
        
        set(f8, 'PaperUnits', 'centimeters');
        set(f8, 'PaperSize', [25.6 19.2]);
        set(f8, 'PaperPosition', [0, 0, 25.6, 19.2]);
        set(f8, 'PaperPositionMode', 'Manual');
        filename8 = ['error_gt',filename_addition];
        filepath8 = fullfile(fileparts(which(file)), filename, 'error_gt',filename8);
        print(f8, filepath8, '-dpdf', '-r300');
        
        %%
        disp('Finished this run')
    end
end
disp('Finished ALL!')

%%
Q_str = {};
for q_val = Q
    string = ['Q=',num2str(q_val)];
    Q_str{end+1} = string;
end
R_str = {};
for r_val = R
    string = ['R=',num2str(r_val)];
    R_str{end+1} = string;
end

%%
disp('Saving Mean Error plots!')
filename_addition = strcat('_',filename);

% Figure 9: Heatmap of avg x errors
f9=figure(9);
clf(f9);
heatmap(errors_x, R_str, Q_str, '%0.6f', 'TickFontSize', fontsize_label,'ShowAllTicks', true, 'FontSize', fontsize_legend, 'TextColor', 'k', 'Colorbar', true)
colormap(flipud(parula))
title('\textbf{Mean Absolute Error of estimated x coord for different q and r}');

set(f9, 'PaperUnits', 'centimeters');
set(f9, 'PaperSize', [20 20]);
set(f9, 'PaperPosition', [0, 0, 20, 20]);
set(f9, 'PaperPositionMode', 'Manual');
filename9 = ['mean_errors_x',filename_addition];
filepath9 = fullfile(fileparts(which(file)), filename,'mean_errors',filename9);
print(f9, filepath9, '-dpdf', '-r300');

% Figure 10: Heatmap of avg y errors
f10=figure(10);
clf(f10);
heatmap(errors_y, R_str, Q_str, '%0.6f', 'TickFontSize', fontsize_label,'ShowAllTicks', true, 'FontSize', fontsize_legend, 'TextColor', 'k', 'Colorbar', true)
colormap(flipud(parula))
title('\textbf{Mean Absolute Error of estimated y coord for different q and r}'   );

set(f10, 'PaperUnits', 'centimeters');
set(f10, 'PaperSize', [20 20]);
set(f10, 'PaperPosition', [0, 0, 20, 20]);
set(f10, 'PaperPositionMode', 'Manual');
filename10 = ['mean_errors_y',filename_addition];
filepath10 = fullfile(fileparts(which(file)), filename,'mean_errors',filename10);
print(f10, filepath10, '-dpdf', '-r300');

% Figure 11: Heatmap of avg z errors
f11=figure(11);
clf(f11);
heatmap(errors_z, R_str, Q_str, '%0.6f', 'TickFontSize', fontsize_label,'ShowAllTicks', true, 'FontSize', fontsize_legend, 'TextColor', 'k', 'Colorbar', true)
colormap(flipud(parula))
title('\textbf{Mean Absolute Error of estimated z coord for different q and r}'   );

set(f11, 'PaperUnits', 'centimeters');
set(f11, 'PaperSize', [20 20]);
set(f11, 'PaperPosition', [0, 0, 20, 20]);
set(f11, 'PaperPositionMode', 'Manual');
filename11 = ['mean_errors_z',filename_addition];
filepath11 = fullfile(fileparts(which(file)), filename,'mean_errors',filename11);
print(f11, filepath11, '-dpdf', '-r300');

% Figure 12: Heatmap of avg all errors
f12=figure(12);
clf(f12);
heatmap(errors, R_str, Q_str, '%0.6f', 'TickFontSize', fontsize_label,'ShowAllTicks', true, 'FontSize', fontsize_legend, 'TextColor', 'k', 'Colorbar', true)
colormap(flipud(parula))
title('\textbf{Mean Absolute Error of estimated coords for different q and r}'   );

set(f12, 'PaperUnits', 'centimeters');
set(f12, 'PaperSize', [20 20]);
set(f12, 'PaperPosition', [0, 0, 20, 20]);
set(f12, 'PaperPositionMode', 'Manual');
filename12 = ['mean_errors',filename_addition];
filepath12 = fullfile(fileparts(which(file)), filename,'mean_errors',filename12);
print(f12, filepath12, '-dpdf', '-r300');

% Figure 13: Heatmap of avg avg errors
f13=figure(13);
clf(f13);
heatmap(errors_avg, R_str, Q_str, '%0.6f', 'TickFontSize', fontsize_label,'ShowAllTicks', true, 'FontSize', fontsize_legend, 'TextColor', 'k', 'Colorbar', true)
colormap(flipud(parula))
title('\textbf{Avg Mean Absolute Error of estimated coords for different q and r}'   );

set(f13, 'PaperUnits', 'centimeters');
set(f13, 'PaperSize', [20 20]);
set(f13, 'PaperPosition', [0, 0, 20, 20]);
set(f13, 'PaperPositionMode', 'Manual');
filename13 = ['mean_errors_avg',filename_addition];
filepath13 = fullfile(fileparts(which(file)), filename,'mean_errors',filename13);
print(f13, filepath13, '-dpdf', '-r300');

% Figure 14: Heatmap of avg x errors with gt
f14=figure(14);
clf(f14);
heatmap(errors_x_gt, R_str, Q_str, '%0.6f', 'TickFontSize', fontsize_label,'ShowAllTicks', true, 'FontSize', fontsize_legend, 'TextColor', 'k', 'Colorbar', true)
colormap(flipud(parula))
title('\textbf{Mean Absolute Error of estimated x coord with gt for different q and r}'   );

set(f14, 'PaperUnits', 'centimeters');
set(f14, 'PaperSize', [20 20]);
set(f14, 'PaperPosition', [0, 0, 20, 20]);
set(f14, 'PaperPositionMode', 'Manual');
filename14 = ['mean_errors_x_gt',filename_addition];
filepath14 = fullfile(fileparts(which(file)), filename,'mean_errors',filename14);
print(f14, filepath14, '-dpdf', '-r300');

% Figure 15: Heatmap of avg y errors with gt
f15=figure(15);
clf(f15);
heatmap(errors_y_gt, R_str, Q_str, '%0.6f', 'TickFontSize', fontsize_label,'ShowAllTicks', true, 'FontSize', fontsize_legend, 'TextColor', 'k', 'Colorbar', true)
colormap(flipud(parula))
title('\textbf{Mean Absolute Error of estimated y coord with gt for different q and r}'   );

set(f15, 'PaperUnits', 'centimeters');
set(f15, 'PaperSize', [20 20]);
set(f15, 'PaperPosition', [0, 0, 20, 20]);
set(f15, 'PaperPositionMode', 'Manual');
filename15 = ['mean_errors_y_gt',filename_addition];
filepath15 = fullfile(fileparts(which(file)), filename,'mean_errors',filename15);
print(f15, filepath15, '-dpdf', '-r300');

% Figure 16: Heatmap of avg z errors with gt
f16=figure(16);
clf(f16);
heatmap(errors_z_gt, R_str, Q_str, '%0.6f', 'TickFontSize', fontsize_label,'ShowAllTicks', true, 'FontSize', fontsize_legend, 'TextColor', 'k', 'Colorbar', true)
colormap(flipud(parula))
title('\textbf{Mean Absolute Error of estimated z coord with gt for different q and r}'   );

set(f16, 'PaperUnits', 'centimeters');
set(f16, 'PaperSize', [20 20]);
set(f16, 'PaperPosition', [0, 0, 20, 20]);
set(f16, 'PaperPositionMode', 'Manual');
filename16 = ['mean_errors_z_gt',filename_addition];
filepath16 = fullfile(fileparts(which(file)), filename,'mean_errors',filename16);
print(f16, filepath16, '-dpdf', '-r300');

% Figure 17: Heatmap of avg all errors with gt
f17=figure(17);
clf(f17);
heatmap(errors_gt, R_str, Q_str, '%0.6f', 'TickFontSize', fontsize_label,'ShowAllTicks', true, 'FontSize', fontsize_legend, 'TextColor', 'k', 'Colorbar', true)
colormap(flipud(parula))
title('\textbf{Mean Absolute Error of estimated coords with gt for different q and r}'   );

set(f17, 'PaperUnits', 'centimeters');
set(f17, 'PaperSize', [20 20]);
set(f17, 'PaperPosition', [0, 0, 20, 20]);
set(f17, 'PaperPositionMode', 'Manual');
filename17 = ['mean_errors_gt',filename_addition];
filepath17 = fullfile(fileparts(which(file)), filename,'mean_errors',filename17);
print(f17, filepath17, '-dpdf', '-r300');

% Figure 18: Heatmap of avg avg errors with gt
f18=figure(18);
clf(f18);
heatmap(errors_avg_gt, R_str, Q_str, '%0.6f', 'TickFontSize', fontsize_label,'ShowAllTicks', true, 'FontSize', fontsize_legend, 'TextColor', 'k', 'Colorbar', true)
colormap(flipud(parula))
title('\textbf{Avg Mean Absolute Error of estimated coords with gt for different q and r}'   );

set(f18, 'PaperUnits', 'centimeters');
set(f18, 'PaperSize', [20 20]);
set(f18, 'PaperPosition', [0, 0, 20, 20]);
set(f18, 'PaperPositionMode', 'Manual');
filename18 = ['mean_errors_avg_gt',filename_addition];
filepath18 = fullfile(fileparts(which(file)), filename,'mean_errors',filename18);
print(f18, filepath18, '-dpdf', '-r300');
disp('Finished Mean Error Plots!')
%%
close all