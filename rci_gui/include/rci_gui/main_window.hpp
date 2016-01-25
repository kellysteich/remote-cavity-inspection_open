/**
 * @file /include/rci_gui/main_window.hpp
 *
 * @brief Qt based gui for rci_gui.
 *
 **/
#ifndef rci_gui_MAIN_WINDOW_H
#define rci_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>

#include <rci_comm/InitSeedPoint.h>
#include <rci_comm/InitTwoSeedPoints.h>
#include <rci_comm/GetTreeDetectionParams.h>
#include <rci_comm/SetTreeDetectionParams.h>
#include <rci_comm/GetFilterPipelineParams.h>
#include <rci_comm/SetFilterPipelineParams.h>
#include <rci_comm/GetBoxCheckParams.h>
#include <rci_comm/SetBoxCheckParams.h>
#include <rci_comm/GetCavityNormalParams.h>
#include <rci_comm/SetCavityNormalParams.h>
#include <rci_comm/GetKalmanParams.h>
#include <rci_comm/SetKalmanParams.h>
#include <rci_comm/GetControllerParams.h>
#include <rci_comm/SetControllerParams.h>
#include <rci_comm/GetPdParams.h>
#include <rci_comm/SetPdParams.h>
#include <rci_comm/LiftArm.h>
#include <rci_comm/StartController.h>
#include <rci_comm/ResetKalman.h>

#include <QtGui/QMainWindow>
#include <QMetaType>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkInteractorStyleImage.h>

#include "ui_main_window.h"

#include "structures.h"

#include "sensor_image_view.hpp"
#include "sensor_pointcloud_view.hpp"
#include "cavity_pose_view.hpp"
#include "debug_view.hpp"

#include <string>
#include <iostream>


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rci_gui
{

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void closeEvent(QCloseEvent *event); // Overloaded function

    void showNoMasterMessage();
    void showNoPixmapMessage();
    void showNoSeedPointMessage();

    geometry_msgs::Point convertGuiToRosImg(QPoint q_point, QSize p_size);
    QPoint convertGuiImg(QPoint initial_point, QSize initial_size, QSize new_size);
    QPoint convertRealGui(double x, double y, double z);

    void updateStats(int processing_time, std::vector<int> &fps, int &avg_fps, int &n_frames);

    void setupPlots();

public Q_SLOTS:
    void log( const struct Log &log);

    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check );
    void on_button_useSeedPoint_clicked(bool check );
    void on_button_lift_arm_clicked(bool check );
    void on_button_start_controller_clicked(bool check );


    /******************************************
    ** Manual connections
    *******************************************/

    void updateFrame(const QImage &frame, const int &frame_width, const int &frame_height);

    void updatePointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc,
                          const QVector3D &min_pt_tree, const QVector3D &max_pt_tree, const QVector3D &cavity_center,
                          const QVector3D &min_pt_cavity, const QVector3D &max_pt_cavity,
                          const QVector3D &cavity_normal, const float &radius);

    void updateCavityPoses(const QVector<double> &cavity_x, const QVector<double> &cavity_y, const QVector<double> &cavity_z, const QVector<double> &time);
    void updateCavityPose(const double &cavity_x, const double &cavity_y, const double &cavity_z);

    void updateRobotPoses(const QVector<double> &robot_x, const QVector<double> &robot_y, const QVector<double> &robot_z, const QVector<double> &robot_time);
    void updateDesiredRobotPoses(const QVector<double> &desired_robot_x, const QVector<double> &desired_robot_y, const QVector<double> &desired_robot_z, const QVector<double> &desired_robot_time);
    void updateGtRobotPoses(const QVector<double> &gt_robot_x, const QVector<double> &gt_robot_y, const QVector<double> &gt_robot_z, const QVector<double> &gt_robot_time);
    void updateEndeffectorPoses(const QVector<double> &endeffector_x, const QVector<double> &endeffector_y, const QVector<double> &endeffector_z, const QVector<double> &endeffector_time);
    void updateDesiredEndeffectorPoses(const QVector<double> &desired_endeffector_x, const QVector<double> &cdesired_endeffector_y, const QVector<double> &desired_endeffector_z, const QVector<double> &desired_endeffector_time);
    void updateGtCavityPoses(const QVector<double> &gt_cavity_x, const QVector<double> &gt_cavity_y, const QVector<double> &gt_cavity_z, const QVector<double> &gt_cavity_time);

    void lostTree();
    void foundTree();

protected:
    bool eventFilter(QObject *some_obj, QEvent *ev);

private Q_SLOTS:
    void on_button_refresh_tree_clicked(bool check);
    void on_lineEdit_min_depth_editingFinished();
    void on_lineEdit_thresh_seed_editingFinished();
    void on_lineEdit_thresh_depth_editingFinished();
    void on_lineEdit_max_scaling_editingFinished();
    void on_lineEdit_min_cavity_width_editingFinished();
    void on_lineEdit_min_cavity_height_editingFinished();
    void on_lineEdit_kmean_k_editingFinished();
    void on_lineEdit_kmeans_tol_editingFinished();
    void on_lineEdit_kmeans_max_trys_editingFinished();

    void on_button_refresh_filters_clicked(bool checked);
    void on_radioButton_passthrough_toggled(bool checked);
    void on_lineEdit_passthrough_min_editingFinished();
    void on_lineEdit_passthrough_max_editingFinished();
    void on_radioButton_voxelgrid_toggled(bool checked);
    void on_lineEdit_voxelgrid_leafsize_editingFinished();
    void on_radioButton_radial_toggled(bool checked);
    void on_lineEdit_radial_min_neighbors_editingFinished();
    void on_lineEdit_radial_radius_search_editingFinished();

    void on_button_refresh_box_normal_clicked(bool checked);
    void on_lineEdit_octree_resolution_editingFinished();
    void on_lineEdit_initial_step_editingFinished();
    void on_lineEdit_max_inliers_editingFinished();
    void on_lineEdit_start_box_width_editingFinished();
    void on_lineEdit_start_box_height_editingFinished();
    void on_lineEdit_start_box_length_editingFinished();
    void on_lineEdit_band_width_editingFinished();

    void on_button_refresh_kalman_clicked(bool checked);
    void on_radioButton_fixed_q_toggled(bool checked);
    void on_lineEdit_qx_editingFinished();
    void on_lineEdit_qy_editingFinished();
    void on_lineEdit_qz_editingFinished();
    void on_lineEdit_rx_editingFinished();
    void on_lineEdit_ry_editingFinished();
    void on_lineEdit_rz_editingFinished();

    void on_button_refresh_controller_clicked(bool checked);
    void on_lineEdit_desired_pitching_angle_editingFinished();
    void on_lineEdit_robot_dist_to_tree_editingFinished();
    void on_lineEdit_arm_pitching_tol_editingFinished();
    void on_lineEdit_robot_position_tol_x_editingFinished();
    void on_lineEdit_robot_position_tol_y_editingFinished();
    void on_lineEdit_robot_position_tol_z_editingFinished();
    void on_lineEdit_robot_orientation_tol_editingFinished();
    void on_lineEdit_robot_offset_x_editingFinished();
    void on_lineEdit_robot_offset_y_editingFinished();
    void on_lineEdit_robot_offset_z_editingFinished();

private:
    Ui::MainWindowDesign ui_;

    QStringListModel logging_model_;
    QStringListModel* loggingModel() { return &logging_model_; }

    SensorImageView sensor_image_view_;
    bool has_pixmap_set_;
    QTime t_sensor_image_view_;
    std::vector<int> fps_sensor_image_view_;
    int avg_fps_sensor_image_view_, n_frames_sensor_image_view_;

    int img_width_, img_height_;

    QPoint seedpoint_tree_; //, seedpoint_cavity_;
    QSize pixmap_size_tree_; //, pixmap_size_cavity_;

    SensorPointcloudView sensor_pointcloud_view_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    QTime t_sensor_pointcloud_view_;
    std::vector<int> fps_sensor_pointcloud_view_;
    int avg_fps_sensor_pointcloud_view_, n_frames_sensor_pointcloud_view_;
    bool first_cloud_;

    bool lost_tree_;

    CavityPoseView cavity_pose_view_;
    QTime t_cavity_pose_view_;
    std::vector<int> fps_cavity_pose_view_;
    int avg_fps_cavity_pose_view_, n_frames_cavity_pose_view_;

    double cavity_coord_x_, cavity_coord_y_, cavity_coord_z_;

    DebugView debug_view_;
};

}  // namespace rci_gui

#endif // rci_gui_MAIN_WINDOW_H
