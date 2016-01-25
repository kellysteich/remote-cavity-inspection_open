/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <QEvent>
#include <iostream>
#include "../include/rci_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , sensor_image_view_(argc, argv, std::string("sensor_image_view"))
    , sensor_pointcloud_view_(argc, argv, std::string("sensor_pointcloud_view"))
    , cavity_pose_view_(argc, argv, std::string("cavity_pose_view"))
    , debug_view_(argc, argv, std::string("debug_view"))
    , has_pixmap_set_(false)
    , n_frames_sensor_image_view_(0)
    , avg_fps_sensor_image_view_(0)
    , n_frames_sensor_pointcloud_view_(0)
    , avg_fps_sensor_pointcloud_view_(0)
    , n_frames_cavity_pose_view_(0)
    , avg_fps_cavity_pose_view_(0)

    , seedpoint_tree_(0,0)
    , pixmap_size_tree_(0,0)
    , cavity_coord_x_(0.0)
    , cavity_coord_y_(0.0)
    , cavity_coord_z_(0.0)
//    , seedpoint_cavity_(0,0)
//    , pixmap_size_cavity_(0,0)
    , lost_tree_(true)
    , first_cloud_(true)
{
    ui_.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    setWindowIcon(QIcon(":/images/icon.png"));
    ui_.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

//    const char *qt_version = qVersion();
//    ROS_INFO_STREAM("Compiled with Qt Version " << QT_VERSION_STR << " and run with Qt Version " << qt_version);
    ROS_INFO_STREAM("Main window thread: " << QThread::currentThreadId());

    ui_.sensor_img->installEventFilter(this);

    /*********************
    ** Clean up threads
    **********************/
    QObject::connect(&sensor_image_view_, SIGNAL(finished()), &sensor_image_view_, SLOT(deleteLater()));
    QObject::connect(&sensor_pointcloud_view_, SIGNAL(finished()), &sensor_pointcloud_view_, SLOT(deleteLater()));
    QObject::connect(&cavity_pose_view_, SIGNAL(finished()), &cavity_pose_view_, SLOT(deleteLater()));
    QObject::connect(&debug_view_, SIGNAL(finished()), &debug_view_, SLOT(deleteLater()));

    /*********************
    ** Logging
    **********************/
    ui_.view_logging->setModel(loggingModel());
    qRegisterMetaType<struct Log>("Log");
    QObject::connect(&sensor_image_view_, SIGNAL(loggingUpdated(struct Log)), this, SLOT(log(struct Log)));
    QObject::connect(&sensor_pointcloud_view_, SIGNAL(loggingUpdated(struct Log)), this, SLOT(log(struct Log)));
    QObject::connect(&cavity_pose_view_, SIGNAL(loggingUpdated(struct Log)), this, SLOT(log(struct Log)));
    QObject::connect(&debug_view_, SIGNAL(loggingUpdated(struct Log)), this, SLOT(log(struct Log)));

    /*********************
    ** Sensor Image View
    **********************/
    QObject::connect(&sensor_image_view_, SIGNAL(newFrame(QImage, int, int)), this, SLOT(updateFrame(QImage, int, int)));
    QObject::connect(&sensor_image_view_, SIGNAL(lostTree()), this, SLOT(lostTree()));
    QObject::connect(&sensor_image_view_, SIGNAL(foundTree()), this, SLOT(foundTree()));

    /*********************
    ** Set up the QVTK window
    **********************/
    viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // Define a translation of 0 meters on the x axis, 0 meters on the y axis and 0 meters on the z axis.
    float theta = M_PI; // The angle of rotation in radians
    transform.translation() << 0.0, 0.0, 0.0;
    // rotate theta radians arround Z axis
    transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
    viewer_->addCoordinateSystem (0.05);
    viewer_->initCameraParameters();

    ui_.qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());

    QVTKInteractor *interactor = ui_.qvtkWidget->GetInteractor();
    interactor->SetRenderWindow(viewer_->getRenderWindow());
    vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
    interactor->SetInteractorStyle(style);
    interactor->Initialize();
    viewer_->setupInteractor(interactor, ui_.qvtkWidget->GetRenderWindow(), style);
    ui_.qvtkWidget->update();

    /*********************
    ** Sensor Poincloud View
    **********************/
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>("pcl::PointCloud<pcl::PointXYZ>::Ptr");
    QObject::connect(&sensor_pointcloud_view_, SIGNAL(newPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr,QVector3D,QVector3D,QVector3D,QVector3D,QVector3D,QVector3D,float)),
                     this, SLOT(updatePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr,QVector3D,QVector3D,QVector3D,QVector3D,QVector3D,QVector3D,float)));

    /*********************
    ** Cavity Pose View
    **********************/
    qRegisterMetaType<QVector<double> >("QVector<double>");
    QObject::connect(&cavity_pose_view_, SIGNAL(newPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)), this, SLOT(updateCavityPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)));
    QObject::connect(&cavity_pose_view_, SIGNAL(newPose(double, double, double)), this, SLOT(updateCavityPose(double, double, double)));

    /*********************
    ** Debug View
    **********************/
    QObject::connect(&debug_view_, SIGNAL(newRobotPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)), this, SLOT(updateRobotPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)));
    QObject::connect(&debug_view_, SIGNAL(newDesiredRobotPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)), this, SLOT(updateDesiredRobotPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)));
    QObject::connect(&debug_view_, SIGNAL(newGtRobotPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)), this, SLOT(updateGtRobotPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)));
    QObject::connect(&debug_view_, SIGNAL(newEndeffectorPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)), this, SLOT(updateEndeffectorPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)));
    QObject::connect(&debug_view_, SIGNAL(newDesiredEndeffectorPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)), this, SLOT(updateDesiredEndeffectorPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)));
    QObject::connect(&debug_view_, SIGNAL(newGtCavityPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)), this, SLOT(updateGtCavityPoses(QVector<double>, QVector<double>, QVector<double>, QVector<double>)));

    setupPlots();
}

MainWindow::~MainWindow()
{
    sleep(1);
}


void MainWindow::setupPlots()
{
    QPalette p = ui_.label_legend_err_to_gt->palette();
    p.setColor(QPalette::WindowText, Qt::darkMagenta);

    ui_.label_legend_err_to_gt->setPalette(p);
    ui_.label_legend_err_to_robot_1->setPalette(p);
    ui_.label_legend_err_to_robot_2->setPalette(p);
    ui_.label_legend_err_to_end_1->setPalette(p);
    ui_.label_legend_err_to_end_2->setPalette(p);

    ui_.label_err_x->setPalette(p);
    ui_.label_err_y->setPalette(p);
    ui_.label_err_z->setPalette(p);

    ui_.label_err_x_desired_robot->setPalette(p);
    ui_.label_err_y_desired_robot->setPalette(p);
    ui_.label_err_z_desired_robot->setPalette(p);
    ui_.label_err_x_desired_end->setPalette(p);
    ui_.label_err_y_desired_end->setPalette(p);
    ui_.label_err_z_desired_end->setPalette(p);

    ui_.label_err_x_gt_robot->setPalette(p);
    ui_.label_err_y_gt_robot->setPalette(p);
    ui_.label_err_z_gt_robot->setPalette(p);
    ui_.label_err_x_gt_cavity->setPalette(p);
    ui_.label_err_y_gt_cavity->setPalette(p);
    ui_.label_err_z_gt_cavity->setPalette(p);

    //********** Cavity Pose **********
    //setup custom legend ouside of plot
    QPalette palette_x = ui_.line_x->palette();
    palette_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.line_x->setPalette(palette_x);
    QPalette palette_y = ui_.line_y->palette();
    palette_y.setColor(QPalette::WindowText, Qt::red);
    ui_.line_y->setPalette(palette_y);
    QPalette palette_z = ui_.line_z->palette();
    palette_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.line_z->setPalette(palette_z);

    QPalette p_x = ui_.label_x->palette();
    p_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.label_x->setPalette(p_x);
    QPalette p_y = ui_.label_y->palette();
    p_y.setColor(QPalette::WindowText, Qt::red);
    ui_.label_y->setPalette(p_y);
    QPalette p_z = ui_.label_z->palette();
    p_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.label_z->setPalette(p_z);

    //add estimated x-coord of cavity
    ui_.plot->addGraph(); // blue line
    ui_.plot->graph(0)->setPen(QPen(Qt::blue));

    ui_.plot->addGraph(); // blue dot
    ui_.plot->graph(1)->setPen(QPen(Qt::blue));
    ui_.plot->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui_.plot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add estimated y-coord of cavity
    ui_.plot->addGraph(); // red line
    ui_.plot->graph(2)->setPen(QPen(Qt::red));

    ui_.plot->addGraph(); // red dot
    ui_.plot->graph(3)->setPen(QPen(Qt::red));
    ui_.plot->graph(3)->setLineStyle(QCPGraph::lsNone);
    ui_.plot->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add estimated z-coord of cavity
    ui_.plot->addGraph(); // darkGreen line
    ui_.plot->graph(4)->setPen(QPen(Qt::darkGreen));

    ui_.plot->addGraph(); // darkGreen dot
    ui_.plot->graph(5)->setPen(QPen(Qt::darkGreen));
    ui_.plot->graph(5)->setLineStyle(QCPGraph::lsNone);
    ui_.plot->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);

    QPen pen;

    pen.setStyle(Qt::DashLine);
    //add gt x-coord
    ui_.plot->addGraph(); // blue line
    pen.setColor(Qt::blue);
    ui_.plot->graph(6)->setPen(pen);

    ui_.plot->addGraph(); // blue dot
    ui_.plot->graph(7)->setPen(QPen(Qt::blue));
    ui_.plot->graph(7)->setLineStyle(QCPGraph::lsNone);
    ui_.plot->graph(7)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add gt y-coord
    ui_.plot->addGraph(); // red line
    pen.setColor(Qt::red);
    ui_.plot->graph(8)->setPen(pen);

    ui_.plot->addGraph(); // red dot
    ui_.plot->graph(9)->setPen(QPen(Qt::red));
    ui_.plot->graph(9)->setLineStyle(QCPGraph::lsNone);
    ui_.plot->graph(9)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add gt z-coord
    ui_.plot->addGraph(); // darkGreen line
    pen.setColor(Qt::green);
    ui_.plot->graph(10)->setPen(pen);

    ui_.plot->addGraph(); // darkGreen dot
    ui_.plot->graph(11)->setPen(QPen(Qt::darkGreen));
    ui_.plot->graph(11)->setLineStyle(QCPGraph::lsNone);
    ui_.plot->graph(11)->setScatterStyle(QCPScatterStyle::ssDisc);

    ui_.plot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui_.plot->xAxis->setDateTimeFormat("ss");
    ui_.plot->xAxis->setAutoTickStep(false);
    ui_.plot->xAxis->setTickStep(2);
//    ui_.plot->yAxis->setAutoTickStep(false);
//    ui_.plot->yAxis->setTickStep(0.05);
    ui_.plot->axisRect()->setupFullAxesBox();
    ui_.plot->xAxis->setRange(30, 0, Qt::AlignRight);
//    ui_.plot->yAxis->setRange(-0.7, 0.7, Qt::AlignRight);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui_.plot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui_.plot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui_.plot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui_.plot->yAxis2, SLOT(setRange(QCPRange)));

    //********** Robot Pose **********
    //setup custom legend ouside of plot
    palette_x = ui_.line_x_robot->palette();
    palette_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.line_x_robot->setPalette(palette_x);
    palette_y = ui_.line_y_robot->palette();
    palette_y.setColor(QPalette::WindowText, Qt::red);
    ui_.line_y_robot->setPalette(palette_y);
    palette_z = ui_.line_z_robot->palette();
    palette_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.line_z_robot->setPalette(palette_z);

    palette_x = ui_.line_x_desired_robot->palette();
    palette_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.line_x_desired_robot->setPalette(palette_x);
    palette_y = ui_.line_y_desired_robot->palette();
    palette_y.setColor(QPalette::WindowText, Qt::red);
    ui_.line_y_desired_robot->setPalette(palette_y);
    palette_z = ui_.line_z_desired_robot->palette();
    palette_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.line_z_desired_robot->setPalette(palette_z);

    palette_x = ui_.line_x_gt_robot->palette();
    palette_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.line_x_gt_robot->setPalette(palette_x);
    palette_y = ui_.line_y_gt_robot->palette();
    palette_y.setColor(QPalette::WindowText, Qt::red);
    ui_.line_y_gt_robot->setPalette(palette_y);
    palette_z = ui_.line_z_gt_robot->palette();
    palette_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.line_z_gt_robot->setPalette(palette_z);

    p_x = ui_.label_x_robot->palette();
    p_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.label_x_robot->setPalette(p_x);
    p_y = ui_.label_y_robot->palette();
    p_y.setColor(QPalette::WindowText, Qt::red);
    ui_.label_y_robot->setPalette(p_y);
    p_z = ui_.label_z_robot->palette();
    p_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.label_z_robot->setPalette(p_z);

    p_x = ui_.label_x_desired_robot->palette();
    p_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.label_x_desired_robot->setPalette(p_x);
    p_y = ui_.label_y_desired_robot->palette();
    p_y.setColor(QPalette::WindowText, Qt::red);
    ui_.label_y_desired_robot->setPalette(p_y);
    p_z = ui_.label_z_desired_robot->palette();
    p_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.label_z_desired_robot->setPalette(p_z);

    p_x = ui_.label_x_gt_robot->palette();
    p_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.label_x_gt_robot->setPalette(p_x);
    p_y = ui_.label_y_gt_robot->palette();
    p_y.setColor(QPalette::WindowText, Qt::red);
    ui_.label_y_gt_robot->setPalette(p_y);
    p_z = ui_.label_z_gt_robot->palette();
    p_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.label_z_gt_robot->setPalette(p_z);

    //add x-coord of robot
    ui_.plot_robot->addGraph(); // blue line
    ui_.plot_robot->graph(0)->setPen(QPen(Qt::blue));

    ui_.plot_robot->addGraph(); // blue dot
    ui_.plot_robot->graph(1)->setPen(QPen(Qt::blue));
    ui_.plot_robot->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_robot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add y-coord of robot
    ui_.plot_robot->addGraph(); // red line
    ui_.plot_robot->graph(2)->setPen(QPen(Qt::red));

    ui_.plot_robot->addGraph(); // red dot
    ui_.plot_robot->graph(3)->setPen(QPen(Qt::red));
    ui_.plot_robot->graph(3)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_robot->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add z-coord of robot
    ui_.plot_robot->addGraph(); // darkGreen line
    ui_.plot_robot->graph(4)->setPen(QPen(Qt::darkGreen));

    ui_.plot_robot->addGraph(); // darkGreen dot
    ui_.plot_robot->graph(5)->setPen(QPen(Qt::darkGreen));
    ui_.plot_robot->graph(5)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_robot->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);

    pen.setStyle(Qt::DashLine);
    //add desired x-coord of robot
    ui_.plot_robot->addGraph(); // blue line
    pen.setColor(Qt::blue);
    ui_.plot_robot->graph(6)->setPen(pen);

    ui_.plot_robot->addGraph(); // blue dot
    ui_.plot_robot->graph(7)->setPen(QPen(Qt::blue));
    ui_.plot_robot->graph(7)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_robot->graph(7)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add desired  y-coord of robot
    ui_.plot_robot->addGraph(); // red line
    pen.setColor(Qt::red);
    ui_.plot_robot->graph(8)->setPen(pen);

    ui_.plot_robot->addGraph(); // red dot
    ui_.plot_robot->graph(9)->setPen(QPen(Qt::red));
    ui_.plot_robot->graph(9)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_robot->graph(9)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add desired  z-coord of robot
    ui_.plot_robot->addGraph(); // darkGreen line
    pen.setColor(Qt::green);
    ui_.plot_robot->graph(10)->setPen(pen);

    ui_.plot_robot->addGraph(); // darkGreen dot
    ui_.plot_robot->graph(11)->setPen(QPen(Qt::darkGreen));
    ui_.plot_robot->graph(11)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_robot->graph(11)->setScatterStyle(QCPScatterStyle::ssDisc);

    pen.setStyle(Qt::DotLine);
    //add gt x-coord of robot
    ui_.plot_robot->addGraph(); // blue line
    pen.setColor(Qt::blue);
    ui_.plot_robot->graph(12)->setPen(pen);

    ui_.plot_robot->addGraph(); // blue dot
    ui_.plot_robot->graph(13)->setPen(QPen(Qt::blue));
    ui_.plot_robot->graph(13)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_robot->graph(13)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add gt  y-coord of robot
    ui_.plot_robot->addGraph(); // red line
    pen.setColor(Qt::red);
    ui_.plot_robot->graph(14)->setPen(pen);

    ui_.plot_robot->addGraph(); // red dot
    ui_.plot_robot->graph(15)->setPen(QPen(Qt::red));
    ui_.plot_robot->graph(15)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_robot->graph(15)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add gt  z-coord of robot
    ui_.plot_robot->addGraph(); // darkGreen line
    pen.setColor(Qt::green);
    ui_.plot_robot->graph(16)->setPen(pen);

    ui_.plot_robot->addGraph(); // darkGreen dot
    ui_.plot_robot->graph(17)->setPen(QPen(Qt::darkGreen));
    ui_.plot_robot->graph(17)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_robot->graph(17)->setScatterStyle(QCPScatterStyle::ssDisc);

    ui_.plot_robot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui_.plot_robot->xAxis->setDateTimeFormat("ss");
    ui_.plot_robot->xAxis->setAutoTickStep(false);
    ui_.plot_robot->xAxis->setTickStep(2);;
    ui_.plot_robot->axisRect()->setupFullAxesBox();
    ui_.plot_robot->xAxis->setRange(30, 0, Qt::AlignRight);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui_.plot_robot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui_.plot_robot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui_.plot_robot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui_.plot_robot->yAxis2, SLOT(setRange(QCPRange)));

    //********** Endeffector Pose **********
    //setup custom legend ouside of plot
    palette_x = ui_.line_x_end->palette();
    palette_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.line_x_end->setPalette(palette_x);
    palette_y = ui_.line_y_end->palette();
    palette_y.setColor(QPalette::WindowText, Qt::red);
    ui_.line_y_end->setPalette(palette_y);
    palette_z = ui_.line_z_end->palette();
    palette_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.line_z_end->setPalette(palette_z);

    palette_x = ui_.line_x_desired_end->palette();
    palette_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.line_x_desired_end->setPalette(palette_x);
    palette_y = ui_.line_y_desired_end->palette();
    palette_y.setColor(QPalette::WindowText, Qt::red);
    ui_.line_y_desired_end->setPalette(palette_y);
    palette_z = ui_.line_z_desired_end->palette();
    palette_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.line_z_desired_end->setPalette(palette_z);

    palette_x = ui_.line_x_gt_cavity->palette();
    palette_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.line_x_gt_cavity->setPalette(palette_x);
    palette_y = ui_.line_y_gt_cavity->palette();
    palette_y.setColor(QPalette::WindowText, Qt::red);
    ui_.line_y_gt_cavity->setPalette(palette_y);
    palette_z = ui_.line_z_gt_cavity->palette();
    palette_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.line_z_gt_cavity->setPalette(palette_z);

    p_x = ui_.label_x_end->palette();
    p_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.label_x_end->setPalette(p_x);
    p_y = ui_.label_y_end->palette();
    p_y.setColor(QPalette::WindowText, Qt::red);
    ui_.label_y_end->setPalette(p_y);
    p_z = ui_.label_z_end->palette();
    p_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.label_z_end->setPalette(p_z);

    p_x = ui_.label_x_desired_end->palette();
    p_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.label_x_desired_end->setPalette(p_x);
    p_y = ui_.label_y_desired_end->palette();
    p_y.setColor(QPalette::WindowText, Qt::red);
    ui_.label_y_desired_end->setPalette(p_y);
    p_z = ui_.label_z_desired_end->palette();
    p_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.label_z_desired_end->setPalette(p_z);

    p_x = ui_.label_x_gt_cavity->palette();
    p_x.setColor(QPalette::WindowText, Qt::blue);
    ui_.label_x_gt_cavity->setPalette(p_x);
    p_y = ui_.label_y_gt_cavity->palette();
    p_y.setColor(QPalette::WindowText, Qt::red);
    ui_.label_y_gt_cavity->setPalette(p_y);
    p_z = ui_.label_z_gt_cavity->palette();
    p_z.setColor(QPalette::WindowText, Qt::darkGreen);
    ui_.label_z_gt_cavity->setPalette(p_z);

    //add x-coord of end
    ui_.plot_end->addGraph(); // blue line
    ui_.plot_end->graph(0)->setPen(QPen(Qt::blue));

    ui_.plot_end->addGraph(); // blue dot
    ui_.plot_end->graph(1)->setPen(QPen(Qt::blue));
    ui_.plot_end->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_end->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add y-coord of end
    ui_.plot_end->addGraph(); // red line
    ui_.plot_end->graph(2)->setPen(QPen(Qt::red));

    ui_.plot_end->addGraph(); // red dot
    ui_.plot_end->graph(3)->setPen(QPen(Qt::red));
    ui_.plot_end->graph(3)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_end->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add z-coord of end
    ui_.plot_end->addGraph(); // darkGreen line
    ui_.plot_end->graph(4)->setPen(QPen(Qt::darkGreen));

    ui_.plot_end->addGraph(); // darkGreen dot
    ui_.plot_end->graph(5)->setPen(QPen(Qt::darkGreen));
    ui_.plot_end->graph(5)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_end->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);

    pen.setStyle(Qt::DashLine);
    //add desired x-coord of end
    ui_.plot_end->addGraph(); // blue line
    pen.setColor(Qt::blue);
    ui_.plot_end->graph(6)->setPen(pen);

    ui_.plot_end->addGraph(); // blue dot
    ui_.plot_end->graph(7)->setPen(QPen(Qt::blue));
    ui_.plot_end->graph(7)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_end->graph(7)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add desired  y-coord of end
    ui_.plot_end->addGraph(); // red line
    pen.setColor(Qt::red);
    ui_.plot_end->graph(8)->setPen(pen);

    ui_.plot_end->addGraph(); // red dot
    ui_.plot_end->graph(9)->setPen(QPen(Qt::red));
    ui_.plot_end->graph(9)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_end->graph(9)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add desired  z-coord of end
    ui_.plot_end->addGraph(); // darkGreen line
    pen.setColor(Qt::green);
    ui_.plot_end->graph(10)->setPen(pen);

    ui_.plot_end->addGraph(); // darkGreen dot
    ui_.plot_end->graph(11)->setPen(QPen(Qt::darkGreen));
    ui_.plot_end->graph(11)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_end->graph(11)->setScatterStyle(QCPScatterStyle::ssDisc);

    pen.setStyle(Qt::DotLine);
    //add gt x-coord of cavity
    ui_.plot_end->addGraph(); // blue line
    pen.setColor(Qt::blue);
    ui_.plot_end->graph(12)->setPen(pen);

    ui_.plot_end->addGraph(); // blue dot
    ui_.plot_end->graph(13)->setPen(QPen(Qt::blue));
    ui_.plot_end->graph(13)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_end->graph(13)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add gt  y-coord of cavity
    ui_.plot_end->addGraph(); // red line
    pen.setColor(Qt::red);
    ui_.plot_end->graph(14)->setPen(pen);

    ui_.plot_end->addGraph(); // red dot
    ui_.plot_end->graph(15)->setPen(QPen(Qt::red));
    ui_.plot_end->graph(15)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_end->graph(15)->setScatterStyle(QCPScatterStyle::ssDisc);

    //add gt  z-coord of cavity
    ui_.plot_end->addGraph(); // darkGreen line
    pen.setColor(Qt::green);
    ui_.plot_end->graph(16)->setPen(pen);

    ui_.plot_end->addGraph(); // darkGreen dot
    ui_.plot_end->graph(17)->setPen(QPen(Qt::darkGreen));
    ui_.plot_end->graph(17)->setLineStyle(QCPGraph::lsNone);
    ui_.plot_end->graph(17)->setScatterStyle(QCPScatterStyle::ssDisc);

    ui_.plot_end->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui_.plot_end->xAxis->setDateTimeFormat("ss");
    ui_.plot_end->xAxis->setAutoTickStep(false);
    ui_.plot_end->xAxis->setTickStep(2);;
    ui_.plot_end->axisRect()->setupFullAxesBox();
    ui_.plot_end->xAxis->setRange(30, 0, Qt::AlignRight);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui_.plot_end->xAxis, SIGNAL(rangeChanged(QCPRange)), ui_.plot_end->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui_.plot_end->yAxis, SIGNAL(rangeChanged(QCPRange)), ui_.plot_end->yAxis2, SLOT(setRange(QCPRange)));
}

/*****************************************************************************
** Implementation [Event Filter]
*****************************************************************************/
bool MainWindow::eventFilter(QObject *some_obj, QEvent *ev)
{
    if(some_obj == ui_.sensor_img && ev->type() == QEvent::MouseButtonPress && has_pixmap_set_)
    {
        //std::cout << "Event Filter thread: " << QThread::currentThreadId() << std::endl;
        QMouseEvent *me = static_cast<QMouseEvent *>(ev);
        MouseButton button = me->button();
        QPixmap pixmap = *ui_.sensor_img->pixmap();
        QPainter *paint = new QPainter(&pixmap);
        if(button == Qt::LeftButton)
        {
            seedpoint_tree_ = me->pos();
            pixmap_size_tree_ = ui_.sensor_img->pixmap()->size();
            paint->setBrush(QColor(255,0,0,255));
            paint->drawEllipse(seedpoint_tree_, 5,5);
        }
//        else if(button == Qt::RightButton)
//        {
//            seedpoint_cavity_ = me->pos();
//            pixmap_size_cavity_ = ui_.sensor_img->pixmap()->size();
//            paint->setBrush(QColor(0,0,255,255));
//            paint->drawEllipse(seedpoint_cavity_, 5,5);
//        }
        delete paint;
        ui_.sensor_img->setPixmap(pixmap);
        return true;
    }
    else
        return false;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage()
{
    QMessageBox msg_box;
    msg_box.setText("Couldn't find the ros master.");
    msg_box.exec();

}

void MainWindow::on_button_connect_clicked(bool check )
{
    ROS_INFO_STREAM("Button connect thread: " << QThread::currentThreadId());
    if(ui_.button_connect->text() == "Connect")
    {
        if( !sensor_image_view_.init() || !sensor_pointcloud_view_.init() || !cavity_pose_view_.init() || !debug_view_.init())
        {
            showNoMasterMessage();
        }
        else
        {
            sensor_image_view_.startThread();
            sensor_pointcloud_view_.startThread();
            cavity_pose_view_.startThread();
            debug_view_.startThread();
            ui_.button_connect->setText("Stop Connection");
            ui_.button_useSeedPoint->setEnabled(true);
            ui_.button_lift_arm->setEnabled(true);
            ui_.button_start_controller->setEnabled(true);
        }
    }
    else if(ui_.button_connect->text() == "Stop Connection")
    {
        sensor_image_view_.finishThread();
        sensor_pointcloud_view_.finishThread();
        cavity_pose_view_.finishThread();
        debug_view_.finishThread();
        ui_.button_connect->setText("Reconnect");
        ui_.button_useSeedPoint->setEnabled(false);
        ui_.button_lift_arm->setEnabled(false);
        ui_.button_start_controller->setEnabled(false);
    }
    else if(ui_.button_connect->text() == "Reconnect")
    {
        sensor_image_view_.startThread();
        sensor_pointcloud_view_.startThread();
        cavity_pose_view_.startThread();
        debug_view_.startThread();
        ui_.button_connect->setText("Stop Connection");
        ui_.button_useSeedPoint->setEnabled(true);
        ui_.button_lift_arm->setEnabled(true);
        ui_.button_start_controller->setEnabled(true);
    }
}

void MainWindow::showNoPixmapMessage()
{
    QMessageBox msg_box;
    msg_box.setText("There isn't an image yet, can't set seed point!.");
    msg_box.exec();
}

void MainWindow::showNoSeedPointMessage()
{
    QMessageBox msg_box;
    msg_box.setText("You have to set a seed Point by clicking in the image first.");
    msg_box.exec();
}

geometry_msgs::Point MainWindow::convertGuiToRosImg(QPoint q_point, QSize p_size)
{
    int img_width = img_width_;
    int img_height = img_height_;

    float width_scale = (float)img_width / p_size.width();
    float height_scale = (float)img_height / p_size.height();

    geometry_msgs::Point img_point;
    img_point.x = q_point.x() * width_scale;
    img_point.y = q_point.y() * height_scale;
    return img_point;
}

void MainWindow::on_button_useSeedPoint_clicked(bool check )
{
    if(has_pixmap_set_)
    {
        std::cout << "Using img_width=" << img_width_ << " and img_height=" << img_height_ << std::endl;
        /*
        if(seedpoint_tree_.x() > 0 && seedpoint_tree_.y() > 0 && seedpoint_cavity_.x() > 0 && seedpoint_cavity_.y() > 0)
        {
            rci_comm::InitTwoSeedPoints srv;
            srv.request.seedpoint_tree = convertGuiToRosImg(seedpoint_tree_, pixmap_size_tree_);
            srv.request.seedpoint_cavity = convertGuiToRosImg(seedpoint_cavity_, pixmap_size_cavity_);
            std::cout << "Requesting service initTwoSeedPoints" << std::endl;
            if (ros::service::call("gui/init_two_seed_points", srv))
            {
                std::cout << "Requested service initTwoSeedPoints with qseedpoint_tree_[" << seedpoint_tree_.x() << "," << seedpoint_tree_.y()
                          << "] and imgseedpoint_tree_[" << srv.request.seedpoint_tree.x << "," << srv.request.seedpoint_tree.y
                          << "] and qseedpoint_cavity_[" << seedpoint_cavity_.x() << "," << seedpoint_cavity_.y()
                          << "] and imgseedpoint_cavity_[" << srv.request.seedpoint_cavity.x << "," << srv.request.seedpoint_cavity.y
                          << std::endl;
                std::cout << "Request initTwoSeedPoints was successfull!" << std::endl;
                seedpoint_tree_.setX(0);
                seedpoint_tree_.setY(0);
                pixmap_size_tree_.setWidth(0);
                pixmap_size_tree_.setHeight(0);
                seedpoint_cavity_.setX(0);
                seedpoint_cavity_.setY(0);
                pixmap_size_cavity_.setWidth(0);
                pixmap_size_cavity_.setHeight(0);
            }
            else
            {
                std::cout << "[Error] Failed to call service initTwoSeedPoints" << std::endl;
            }
        }
        else
        */
        if(seedpoint_tree_.x() > 0 && seedpoint_tree_.y() > 0)
        {
            rci_comm::InitSeedPoint srv;
            srv.request.seedpoint = convertGuiToRosImg(seedpoint_tree_, pixmap_size_tree_);
            std::cout << "Requesting service initSeedPoint" << std::endl;
            if (ros::service::call("gui/init_seed_point", srv))
            {
                std::cout << "Requested service initSeedPoint with qseedPoint[" << seedpoint_tree_.x() << "," << seedpoint_tree_.y() << "] and imgseedPoint[" << srv.request.seedpoint.x << "," << srv.request.seedpoint.y << "]" << std::endl;
                std::cout << "Request initSeedPoint was successfull!" << std::endl;
                seedpoint_tree_.setX(0);
                seedpoint_tree_.setY(0);
                pixmap_size_tree_.setWidth(0);
                pixmap_size_tree_.setHeight(0);

                //reset the kalman filter
                rci_comm::ResetKalman reset_srv;
                std::cout << "Requesting service resetKalman" << std::endl;
                if (ros::service::call("gui/reset_kalman", reset_srv))
                {
                    std::cout << "Request resetKalman was successfull!" << std::endl;

                    //clear plots
                    ui_.plot->graph(0)->clearData();
                    ui_.plot->graph(1)->clearData();
                    ui_.plot->graph(2)->clearData();
                    ui_.plot->graph(3)->clearData();
                    ui_.plot->graph(4)->clearData();
                    ui_.plot->graph(5)->clearData();
                    ui_.label_coord_x->setText("");
                    ui_.label_coord_y->setText("");
                    ui_.label_coord_z->setText("");
                }
                else
                {
                    std::cout << "[Error] Failed to call service resetKalman" << std::endl;
                }

            }
            else
            {
                std::cout << "[Error] Failed to call service initSeedPoint" << std::endl;
            }
        }
        else
        {
            showNoSeedPointMessage();
        }
    }
    else
    {
        showNoPixmapMessage();
    }
}

void MainWindow::on_button_lift_arm_clicked(bool check)
{
    rci_comm::LiftArm srv;
    std::cout << "Requesting service liftArm" << std::endl;
    if (ros::service::call("gui/lift_arm", srv))
    {
        std::cout << "Request liftArm was successfull!" << std::endl;
        QPalette palette;
        palette.setColor(QPalette::Window, Qt::green);
        palette.setColor(QPalette::WindowText, Qt::black);
        ui_.label_info_controller->setAutoFillBackground(true);
        ui_.label_info_controller->setPalette(palette);
        ui_.label_info_controller->setText("Lifting manipulator arm!");
    }
    else
    {
        std::cout << "[Error] Failed to call service liftArm" << std::endl;
        QPalette palette;
        palette.setColor(QPalette::Window, Qt::red);
        palette.setColor(QPalette::WindowText, Qt::black);
        ui_.label_info_controller->setAutoFillBackground(true);
        ui_.label_info_controller->setPalette(palette);
        ui_.label_info_controller->setText("Failed to lift manipulator arm!");
    }
}

void MainWindow::on_button_start_controller_clicked(bool check)
{
    rci_comm::StartController srv;
    if(ui_.button_start_controller->text() == "Start Auto Controller")
    {
       srv.request.start = true;
       std::cout << "Requesting service startController" << std::endl;
       if (ros::service::call("gui/start_controller", srv))
       {
           std::cout << "Request startController was successfull! Result = " << srv.response.result << std::endl;
           if(srv.response.result == 0)
           {
               std::cout << "StartController was successfull!" << std::endl;
               ui_.button_start_controller->setText("Stop Auto Controller");
               QPalette palette;
               palette.setColor(QPalette::Window, Qt::green);
               palette.setColor(QPalette::WindowText, Qt::black);
               ui_.label_info_controller->setAutoFillBackground(true);
               ui_.label_info_controller->setPalette(palette);
               ui_.label_info_controller->setText("Started auto controller!");
           }
           else if(srv.response.result == 1)
           {
               std::cout << "StartController failed!" << std::endl;
               QPalette palette;
               palette.setColor(QPalette::Window, Qt::red);
               palette.setColor(QPalette::WindowText, Qt::black);
               ui_.label_info_controller->setAutoFillBackground(true);
               ui_.label_info_controller->setPalette(palette);
               ui_.label_info_controller->setText("Can't start auto controller! No cavity has been found yet!");
           }
           else if(srv.response.result == 2)
           {
               std::cout << "StartController failed!" << std::endl;
               QPalette palette;
               palette.setColor(QPalette::Window, Qt::red);
               palette.setColor(QPalette::WindowText, Qt::black);
               ui_.label_info_controller->setAutoFillBackground(true);
               ui_.label_info_controller->setPalette(palette);
               ui_.label_info_controller->setText("Can't start auto controller! Please lift the arm first!");
           }
       }
       else
       {
           std::cout << "[Error] Failed to call service startController" << std::endl;
           QPalette palette;
           palette.setColor(QPalette::Window, Qt::red);
           palette.setColor(QPalette::WindowText, Qt::black);
           ui_.label_info_controller->setAutoFillBackground(true);
           ui_.label_info_controller->setPalette(palette);
           ui_.label_info_controller->setText("Failed to start auto controller!");
       }
    }
    else if(ui_.button_start_controller->text() == "Stop Auto Controller")
    {
        srv.request.start = false;
        std::cout << "Requesting service stopController" << std::endl;
        if (ros::service::call("gui/start_controller", srv))
        {
            std::cout << "Request stopController was successfull!" << std::endl;
            ui_.button_start_controller->setText("Start Auto Controller");
            QPalette palette;
            palette.setColor(QPalette::Window, Qt::green);
            palette.setColor(QPalette::WindowText, Qt::black);
            ui_.label_info_controller->setAutoFillBackground(true);
            ui_.label_info_controller->setPalette(palette);
            ui_.label_info_controller->setText("Stopped auto controller!");
        }
        else
        {
            std::cout << "[Error] Failed to call service stopController" << std::endl;
            QPalette palette;
            palette.setColor(QPalette::Window, Qt::red);
            palette.setColor(QPalette::WindowText, Qt::black);
            ui_.label_info_controller->setAutoFillBackground(true);
            ui_.label_info_controller->setPalette(palette);
            ui_.label_info_controller->setText("Failed to start auto controller!");
        }
    }
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/
void MainWindow::log( const struct Log &log)
{
    switch ( log.level )
    {
    case(Debug) :
    {
        //                logging_model_.insertRows(logging_model_.rowCount(),1);
        //                std::stringstream logging_model_msg;
        //                logging_model_msg << "[DEBUG] [" << QDateTime::currentDateTime().toTime_t() << "]: " << log.msg;
        //                std::cout << logging_model_msg.str() << std::endl;
        //                QVariant new_row(QString(logging_model_msg.str().c_str()));
        //                logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
        break;
    }
    case(Info) :
    {
        logging_model_.insertRows(logging_model_.rowCount(),1);
        std::stringstream logging_model_msg;
        ROS_INFO_STREAM(log.msg);
        logging_model_msg << "[INFO] [" << QDateTime::currentDateTime().toTime_t() << "]: " << log.msg;
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
        break;
    }
    case(Warn) :
    {
        logging_model_.insertRows(logging_model_.rowCount(),1);
        std::stringstream logging_model_msg;
        ROS_WARN_STREAM(log.msg);
        logging_model_msg << "[Warn] [" << QDateTime::currentDateTime().toTime_t() << "]: " << log.msg;
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
        break;
    }
    case(Error) :
    {
        logging_model_.insertRows(logging_model_.rowCount(),1);
        std::stringstream logging_model_msg;
        ROS_ERROR_STREAM(log.msg);
        logging_model_msg << "[ERROR] [" << QDateTime::currentDateTime().toTime_t() << "]: " << log.msg;
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
        break;
    }
    case(Message) :
    {
        logging_model_.insertRows(logging_model_.rowCount(),1);
        std::stringstream logging_model_msg;
        ROS_DEBUG_STREAM(log.msg);
        logging_model_msg << "[Message] [" << QDateTime::currentDateTime().toTime_t() << "]: " << log.msg;
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
        break;
    }
    default:
        break;
    }
    ui_.view_logging->scrollToBottom();
}

void MainWindow::updateStats(int processing_time, std::vector<int> &fps, int &avg_fps, int &n_frames)
{
    // Add instantaneous FPS value to queue
    if(processing_time>0)
    {
        fps.push_back((int)1000/processing_time);
    }
    // Maximum size of queue is length
    if (fps.size() == 100)
    {
        int sum = std::accumulate(fps.begin(),fps.end(),0);
        avg_fps = (int) sum / 100;
        fps.clear();
    }
    n_frames++;
}

/********************** Sensor Image View ***************************/
QPoint MainWindow::convertGuiImg(QPoint initial_point, QSize initial_size, QSize new_size)
{
    double width_scale = new_size.width() / initial_size.width();
    double height_scale = new_size.height() / initial_size.height();

    QPoint new_point;
    new_point.setX(initial_point.x() * width_scale);
    new_point.setY(initial_point.y() * height_scale);
    return new_point;
}

void MainWindow::updateFrame(const QImage &frame, const int &frame_width, const int &frame_height)
{
    ROS_INFO_STREAM_ONCE("Update Frame thread: " << QThread::currentThreadId());
    int processing_time = t_sensor_image_view_.elapsed();
    t_sensor_image_view_.start();

    img_width_ = frame_width;
    img_height_ = frame_height;

    // Display frame
    QPixmap pixmap = QPixmap::fromImage(frame).scaled(
                ui_.sensor_img->width(), ui_.sensor_img->height(),Qt::KeepAspectRatio);
    if(seedpoint_tree_.x() > 0 && seedpoint_tree_.y() > 0)
    {
        QPainter *paint = new QPainter(&pixmap);
        paint->setBrush(QColor(255,0,0,255));
        if(ui_.sensor_img->pixmap()->size() == pixmap_size_tree_)
        {
            //draw seedPoint
            paint->drawEllipse(seedpoint_tree_, 5,5);
        }
        else
        {
            QPoint draw_point = convertGuiImg(seedpoint_tree_, pixmap_size_tree_, ui_.sensor_img->pixmap()->size());
            //draw drawPoint
            paint->drawEllipse(draw_point, 5,5);
        }
        delete paint;
    }
//    if(seedpoint_cavity_.x() > 0 && seedpoint_cavity_.y() > 0)
//    {
//        QPainter *paint = new QPainter(&pixmap);
//        paint->setBrush(QColor(0,0,255,255));
//        if(ui_.sensor_img->pixmap()->size() == pixmap_size_cavity_)
//        {
//            //draw seedPoint
//            paint->drawEllipse(seedpoint_cavity_, 5,5);
//        }
//        else
//        {
//            QPoint draw_point = convertGuiImg(seedpoint_cavity_, pixmap_size_cavity_, ui_.sensor_img->pixmap()->size());
//            //draw drawPoint
//            paint->drawEllipse(draw_point, 5,5);
//        }
//        delete paint;
//    }
    ui_.sensor_img->setPixmap(pixmap);

    updateStats(processing_time, fps_sensor_image_view_, avg_fps_sensor_image_view_, n_frames_sensor_image_view_);
    ui_.label_display_avgFPS->setText(QString::number(avg_fps_sensor_image_view_));
    ui_.label_display_NFrames->setText(QString::number(n_frames_sensor_image_view_));

    has_pixmap_set_ = true;
}

void MainWindow::updatePointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, const QVector3D &min_pt_tree,
                                  const QVector3D &max_pt_tree, const QVector3D &cavity_center, const QVector3D &min_pt_cavity,
                                  const QVector3D &max_pt_cavity, const QVector3D &cavity_normal, const float &radius)
{
    ROS_INFO_STREAM_ONCE("Update Pointcloud thread: " << QThread::currentThreadId());
    int processing_time = t_sensor_pointcloud_view_.elapsed();
    t_sensor_pointcloud_view_.start();

    //std::cout << "Update Pointcloud thread: " << QThread::currentThreadId() << std::endl;

    // Display frame
    viewer_->removePointCloud("cloud");
    viewer_->addPointCloud<pcl::PointXYZ> (pc, "cloud");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    viewer_->removeAllShapes();

//    if(pc->points.size() > 19152)
//    {
//    std::cout << pc->points[19152] << std::endl;
//    viewer_->addSphere(pc->points[19152], 0.005, 1.0, 1.0, 0, "spheretest");
//    }

//    viewer_->addLine(pcl::PointXYZ(0,0,0), pcl::PointXYZ(0,0,5), "line");
//    viewer_->addSphere(pcl::PointXYZ(0,0,0.25), 0.005, 1.0, 0, 0, "sphere1");
//    viewer_->addSphere(pcl::PointXYZ(0,0,0.5), 0.005, 0, 1.0, 0, "sphere2");
//    viewer_->addSphere(pcl::PointXYZ(0,0,0.75), 0.005, 0, 0, 1.0, "sphere3");
//    viewer_->addSphere(pcl::PointXYZ(0,0,1.0), 0.005, 1.0, 1.0, 1.0, "sphere4");

//    viewer_->addSphere(pcl::PointXYZ(0,0,1.25), 0.005, 1.0, 0, 0, "sphere5");
//    viewer_->addSphere(pcl::PointXYZ(0,0,1.5), 0.005, 0, 1.0, 0, "sphere6");
//    viewer_->addSphere(pcl::PointXYZ(0,0,1.75), 0.005, 0, 0, 1.0, "sphere7");
//    viewer_->addSphere(pcl::PointXYZ(0,0,2.0), 0.005, 1.0, 1.0, 1.0, "sphere8");

//    viewer_->addSphere(pcl::PointXYZ(0,0,2.25), 0.005, 1.0, 0, 0, "sphere9");
//    viewer_->addSphere(pcl::PointXYZ(0,0,2.5), 0.005, 0, 1.0, 0, "sphere10");
//    viewer_->addSphere(pcl::PointXYZ(0,0,2.75), 0.005, 0, 0, 1.0, "sphere11");
//    viewer_->addSphere(pcl::PointXYZ(0,0,3.0), 0.005, 1.0, 1.0, 1.0, "sphere12");

//    viewer_->addSphere(pcl::PointXYZ(0,0,3.25), 0.005, 1.0, 0, 0, "sphere13");
//    viewer_->addSphere(pcl::PointXYZ(0,0,3.5), 0.005, 0, 1.0, 0, "sphere14");
//    viewer_->addSphere(pcl::PointXYZ(0,0,3.75), 0.005, 0, 0, 1.0, "sphere15");
//    viewer_->addSphere(pcl::PointXYZ(0,0,4.0), 0.005, 1.0, 1.0, 1.0, "sphere16");

//    viewer_->addSphere(pcl::PointXYZ(0,0,4.25), 0.005, 1.0, 0, 0, "sphere17");
//    viewer_->addSphere(pcl::PointXYZ(0,0,4.5), 0.005, 0, 1.0, 0, "sphere18");
//    viewer_->addSphere(pcl::PointXYZ(0,0,4.75), 0.005, 0, 0, 1.0, "sphere19");
//    viewer_->addSphere(pcl::PointXYZ(0,0,5.0), 0.005, 1.0, 1.0, 1.0, "sphere20");

    if(cavity_coord_x_ != 0 || cavity_coord_y_ != 0 || cavity_coord_z_ != 0)
    {
        viewer_->addSphere(pcl::PointXYZ(cavity_coord_x_, cavity_coord_y_, cavity_coord_z_), 0.01, 1.0, 0, 0, "estimated_cavity_center");
    }

    if(!lost_tree_)
    {
        viewer_->addCube(min_pt_tree.x(), max_pt_tree.x(), min_pt_tree.y(), max_pt_tree.y(), min_pt_tree.z()-0.01, max_pt_tree.z()+0.01, 0.0, 0.0, 1.0, "tree");
        if(min_pt_cavity.x() > -100 && max_pt_cavity.x() > -100 && min_pt_cavity.y() > -100 && max_pt_cavity.y() > -100 && min_pt_cavity.z() > -100 && max_pt_cavity.z() > -100)
        {
            viewer_->addCube(min_pt_cavity.x(), max_pt_cavity.x(), min_pt_cavity.y(), max_pt_cavity.y(), min_pt_cavity.z(), max_pt_cavity.z(), 0.0, 1.0, 0.0, "cavity_box");
            if(cavity_normal.x() > -100 && cavity_normal.y() > -100 && cavity_normal.z() > -100)
            {
                viewer_->addArrow(pcl::PointXYZ(cavity_center.x()+0.1*cavity_normal.x(),cavity_center.y()+0.1*cavity_normal.y(),cavity_center.z()+0.1*cavity_normal.z()),
                                  pcl::PointXYZ(cavity_center.x(),cavity_center.y(),cavity_center.z()),
                                  1.0, 0.0, 0.0, false, "normal");
                viewer_->addCube(min_pt_cavity.x()-radius, max_pt_cavity.x()+radius, min_pt_cavity.y()-radius, max_pt_cavity.y()+radius, min_pt_cavity.z(), min_pt_cavity.z()+radius, 1.0, 0.0, 0.0, "radius_box");
//                viewer_->addSphere(pcl::PointXYZ(cavity_center.x(),cavity_center.y(),cavity_center.z()), radius, 1.0, 0.0, 0.0, "radius");
//                viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
//                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"radius");
            }
        }
//        if(cavity_center.x() != 0 && cavity_center.y() != 0 && cavity_center.z() != 0)
//        {
//            viewer_->addLine(pcl::PointXYZ(cavity_center.x()+0.2,cavity_center.y(),cavity_center.z()+0.01), pcl::PointXYZ(cavity_center.x()-0.2,cavity_center.y(),cavity_center.z()+0.01), "line_front");
//            viewer_->addLine(pcl::PointXYZ(cavity_center.x()+0.2,cavity_center.y(),cavity_center.z()-0.01), pcl::PointXYZ(cavity_center.x()-0.2,cavity_center.y(),cavity_center.z()-0.01), "line_back");
//        }
    }

    if(first_cloud_)
    {
        viewer_->setCameraPosition(0,0,0,0,-1,0);
        viewer_->resetCamera();
        first_cloud_ = false;
    }

    //viewer_->setRepresentationToSurfaceForAllActors();

    ui_.qvtkWidget->update();

    updateStats(processing_time, fps_sensor_pointcloud_view_, avg_fps_sensor_pointcloud_view_, n_frames_sensor_pointcloud_view_);
    ui_.label_display_avgFPS_pointcloud->setText(QString::number(avg_fps_sensor_pointcloud_view_));
    ui_.label_display_NFrames_pointcloud->setText(QString::number(n_frames_sensor_pointcloud_view_));
}

void MainWindow::lostTree()
{
    QPalette palette;
    palette.setColor(QPalette::Window, Qt::red);
    palette.setColor(QPalette::WindowText, Qt::black);
    ui_.label_info->setAutoFillBackground(true);
    ui_.label_info->setPalette(palette);
    ui_.label_info->setText("Lost Tree! Please place a new seed point!");
    viewer_->removeShape("tree");
    viewer_->removeShape("cavity_box");
    lost_tree_ = true;
}

void MainWindow::foundTree()
{
    QPalette palette;
    palette.setColor(QPalette::Window, Qt::green);
    palette.setColor(QPalette::WindowText, Qt::black);

    ui_.label_info->setAutoFillBackground(true);
    ui_.label_info->setPalette(palette);
    ui_.label_info->setText("Found the Tree! Tracking it from now on. Please place a new seed point if you notice the tree position is wrong!");
    lost_tree_ = false;
}

void MainWindow::updateCavityPoses(const QVector<double> &cavity_x, const QVector<double> &cavity_y, const QVector<double> &cavity_z, const QVector<double> &time)
{
    ROS_INFO_STREAM_ONCE("Update Cavity thread: " << QThread::currentThreadId());
    int processing_time = t_cavity_pose_view_.elapsed();
    t_cavity_pose_view_.start();

    //update plot
    // add data for estiated cavity x-coord
    ui_.plot->graph(0)->addData(time, cavity_x);
    ui_.plot->graph(1)->clearData();
    ui_.plot->graph(1)->addData(time.last(), cavity_x.last());

    // add data for estiated cavity y-coord
    ui_.plot->graph(2)->addData(time, cavity_y);
    ui_.plot->graph(3)->clearData();
    ui_.plot->graph(3)->addData(time.last(), cavity_y.last());

    // add data for estiated cavity z-coord
    ui_.plot->graph(4)->addData(time, cavity_z);
    ui_.plot->graph(5)->clearData();
    ui_.plot->graph(5)->addData(time.last(), cavity_z.last());

    // remove data of lines that's outside visible range:
    ui_.plot->graph(0)->removeDataBefore(time.last()-60);
    ui_.plot->graph(2)->removeDataBefore(time.last()-60);
    ui_.plot->graph(4)->removeDataBefore(time.last()-60);

    // rescale value (vertical) axis to fit the current data:
    ui_.plot->graph(0)->rescaleValueAxis();
    ui_.plot->graph(2)->rescaleValueAxis(true);
    ui_.plot->graph(4)->rescaleValueAxis(true);
    ui_.plot->graph(6)->rescaleValueAxis(true);
    ui_.plot->graph(8)->rescaleValueAxis(true);
    ui_.plot->graph(10)->rescaleValueAxis(true);

    // make key axis range scroll with the data (at a constant range size of 30):
    ui_.plot->xAxis->setRange(time.last()+0.1, 60, Qt::AlignRight);
    ui_.plot->replot();

    //add label for cavity coords
    ui_.label_coord_x->setText(QString::number(cavity_x.last()));
    ui_.label_coord_y->setText(QString::number(cavity_y.last()));
    ui_.label_coord_z->setText(QString::number(cavity_z.last()));

    updateStats(processing_time, fps_cavity_pose_view_, avg_fps_cavity_pose_view_, n_frames_cavity_pose_view_);
    ui_.label_display_avgFPS_cavity_pose->setText(QString::number(avg_fps_cavity_pose_view_));
    ui_.label_display_NFrames_cavity_pose->setText(QString::number(n_frames_cavity_pose_view_));
}

void MainWindow::updateCavityPose(const double &cavity_x, const double &cavity_y, const double &cavity_z)
{
    cavity_coord_x_ = cavity_x;
    cavity_coord_y_ = cavity_y;
    cavity_coord_z_ = cavity_z;
}

void MainWindow::updateRobotPoses(const QVector<double> &robot_x, const QVector<double> &robot_y, const QVector<double> &robot_z, const QVector<double> &robot_time)
{
    ROS_INFO_STREAM_ONCE("Update Robot Poses thread: " << QThread::currentThreadId());
    //update plot
    // add data for robot x-coord
    ui_.plot_robot->graph(0)->addData(robot_time, robot_x);
    ui_.plot_robot->graph(1)->clearData();
    ui_.plot_robot->graph(1)->addData(robot_time.last(), robot_x.last());

    // add data for robot y-coord
    ui_.plot_robot->graph(2)->addData(robot_time, robot_y);
    ui_.plot_robot->graph(3)->clearData();
    ui_.plot_robot->graph(3)->addData(robot_time.last(), robot_y.last());

    // add data for robot z-coord
    ui_.plot_robot->graph(4)->addData(robot_time, robot_z);
    ui_.plot_robot->graph(5)->clearData();
    ui_.plot_robot->graph(5)->addData(robot_time.last(), robot_z.last());

    // remove data of lines that's outside visible range:
    ui_.plot_robot->graph(0)->removeDataBefore(robot_time.last()-60);
    ui_.plot_robot->graph(2)->removeDataBefore(robot_time.last()-60);
    ui_.plot_robot->graph(4)->removeDataBefore(robot_time.last()-60);

    // rescale value (vertical) axis to fit the current data:
    ui_.plot_robot->graph(0)->rescaleValueAxis();
    ui_.plot_robot->graph(2)->rescaleValueAxis(true);
    ui_.plot_robot->graph(4)->rescaleValueAxis(true);
    ui_.plot_robot->graph(6)->rescaleValueAxis(true);
    ui_.plot_robot->graph(8)->rescaleValueAxis(true);
    ui_.plot_robot->graph(10)->rescaleValueAxis(true);
    ui_.plot_robot->graph(12)->rescaleValueAxis(true);
    ui_.plot_robot->graph(14)->rescaleValueAxis(true);
    ui_.plot_robot->graph(16)->rescaleValueAxis(true);

    // make key axis range scroll with the data (at a constant range size of 30):
    ui_.plot_robot->xAxis->setRange(robot_time.last()+0.1, 60, Qt::AlignRight);
    ui_.plot_robot->replot();

    //add label for cavity coords
    ui_.label_coord_x_robot->setText(QString::number(robot_x.last()));
    ui_.label_coord_y_robot->setText(QString::number(robot_y.last()));
    ui_.label_coord_z_robot->setText(QString::number(robot_z.last()));
}

void MainWindow::updateDesiredRobotPoses(const QVector<double> &desired_robot_x, const QVector<double> &desired_robot_y, const QVector<double> &desired_robot_z, const QVector<double> &desired_robot_time)
{
    //update plot
    // add data for desired robot x-coord
    ui_.plot_robot->graph(6)->addData(desired_robot_time, desired_robot_x);
    ui_.plot_robot->graph(7)->clearData();
    ui_.plot_robot->graph(7)->addData(desired_robot_time.last(), desired_robot_x.last());

    // add data for desired robot y-coord
    ui_.plot_robot->graph(8)->addData(desired_robot_time, desired_robot_y);
    ui_.plot_robot->graph(9)->clearData();
    ui_.plot_robot->graph(9)->addData(desired_robot_time.last(), desired_robot_y.last());

    // add data for desired robot z-coord
    ui_.plot_robot->graph(10)->addData(desired_robot_time, desired_robot_z);
    ui_.plot_robot->graph(11)->clearData();
    ui_.plot_robot->graph(11)->addData(desired_robot_time.last(), desired_robot_z.last());

    // remove data of lines that's outside visible range:
    ui_.plot_robot->graph(6)->removeDataBefore(desired_robot_time.last()-60);
    ui_.plot_robot->graph(8)->removeDataBefore(desired_robot_time.last()-60);
    ui_.plot_robot->graph(10)->removeDataBefore(desired_robot_time.last()-60);

    // rescale value (vertical) axis to fit the current data:
    ui_.plot_robot->graph(0)->rescaleValueAxis();
    ui_.plot_robot->graph(2)->rescaleValueAxis(true);
    ui_.plot_robot->graph(4)->rescaleValueAxis(true);
    ui_.plot_robot->graph(6)->rescaleValueAxis(true);
    ui_.plot_robot->graph(8)->rescaleValueAxis(true);
    ui_.plot_robot->graph(10)->rescaleValueAxis(true);
    ui_.plot_robot->graph(12)->rescaleValueAxis(true);
    ui_.plot_robot->graph(14)->rescaleValueAxis(true);
    ui_.plot_robot->graph(16)->rescaleValueAxis(true);

    //add label for cavity coords
    ui_.label_coord_x_desired_robot->setText(QString::number(desired_robot_x.last()));
    ui_.label_coord_y_desired_robot->setText(QString::number(desired_robot_y.last()));
    ui_.label_coord_z_desired_robot->setText(QString::number(desired_robot_z.last()));

    double err_x = std::abs(desired_robot_x.last() - ui_.label_coord_x_robot->text().toDouble());
    double err_y = std::abs(desired_robot_y.last() - ui_.label_coord_y_robot->text().toDouble());
    double err_z = std::abs(desired_robot_z.last() - ui_.label_coord_z_robot->text().toDouble());
    ui_.label_err_x_desired_robot->setText(QString::number(err_x,'f',4));
    ui_.label_err_y_desired_robot->setText(QString::number(err_y,'f',4));
    ui_.label_err_z_desired_robot->setText(QString::number(err_z,'f',4));
}

void MainWindow::updateGtRobotPoses(const QVector<double> &gt_robot_x, const QVector<double> &gt_robot_y, const QVector<double> &gt_robot_z, const QVector<double> &gt_robot_time)
{
    //update plot
    // add data for gt robot x-coord
    ui_.plot_robot->graph(12)->addData(gt_robot_time, gt_robot_x);
    ui_.plot_robot->graph(13)->clearData();
    ui_.plot_robot->graph(13)->addData(gt_robot_time.last(), gt_robot_x.last());

    // add data for gt robot y-coord
    ui_.plot_robot->graph(14)->addData(gt_robot_time, gt_robot_y);
    ui_.plot_robot->graph(15)->clearData();
    ui_.plot_robot->graph(15)->addData(gt_robot_time.last(), gt_robot_y.last());

    // add data for gt robot z-coord
    ui_.plot_robot->graph(16)->addData(gt_robot_time, gt_robot_z);
    ui_.plot_robot->graph(17)->clearData();
    ui_.plot_robot->graph(17)->addData(gt_robot_time.last(), gt_robot_z.last());

    // remove data of lines that's outside visible range:
    ui_.plot_robot->graph(12)->removeDataBefore(gt_robot_time.last()-60);
    ui_.plot_robot->graph(14)->removeDataBefore(gt_robot_time.last()-60);
    ui_.plot_robot->graph(16)->removeDataBefore(gt_robot_time.last()-60);

    // rescale value (vertical) axis to fit the current data:
    ui_.plot_robot->graph(0)->rescaleValueAxis();
    ui_.plot_robot->graph(2)->rescaleValueAxis(true);
    ui_.plot_robot->graph(4)->rescaleValueAxis(true);
    ui_.plot_robot->graph(6)->rescaleValueAxis(true);
    ui_.plot_robot->graph(8)->rescaleValueAxis(true);
    ui_.plot_robot->graph(10)->rescaleValueAxis(true);
    ui_.plot_robot->graph(12)->rescaleValueAxis(true);
    ui_.plot_robot->graph(14)->rescaleValueAxis(true);
    ui_.plot_robot->graph(16)->rescaleValueAxis(true);

    //add label for cavity coords
    ui_.label_coord_x_gt_robot->setText(QString::number(gt_robot_x.last()));
    ui_.label_coord_y_gt_robot->setText(QString::number(gt_robot_y.last()));
    ui_.label_coord_z_gt_robot->setText(QString::number(gt_robot_z.last()));

    double err_x = std::abs(gt_robot_x.last() - ui_.label_coord_x_robot->text().toDouble());
    double err_y = std::abs(gt_robot_y.last() - ui_.label_coord_y_robot->text().toDouble());
    double err_z = std::abs(gt_robot_z.last() - ui_.label_coord_z_robot->text().toDouble());
    ui_.label_err_x_gt_robot->setText(QString::number(err_x,'f',4));
    ui_.label_err_y_gt_robot->setText(QString::number(err_y,'f',4));
    ui_.label_err_z_gt_robot->setText(QString::number(err_z,'f',4));
}

void MainWindow::updateEndeffectorPoses(const QVector<double> &endeffector_x, const QVector<double> &endeffector_y, const QVector<double> &endeffector_z, const QVector<double> &endeffector_time)
{
    // add data for end x-coord
    ui_.plot_end->graph(0)->addData(endeffector_time, endeffector_x);
    ui_.plot_end->graph(1)->clearData();
    ui_.plot_end->graph(1)->addData(endeffector_time.last(), endeffector_x.last());

    // add data for endeffector y-coord
    ui_.plot_end->graph(2)->addData(endeffector_time, endeffector_y);
    ui_.plot_end->graph(3)->clearData();
    ui_.plot_end->graph(3)->addData(endeffector_time.last(), endeffector_y.last());

    // add data for endeffector z-coord
    ui_.plot_end->graph(4)->addData(endeffector_time, endeffector_z);
    ui_.plot_end->graph(5)->clearData();
    ui_.plot_end->graph(5)->addData(endeffector_time.last(), endeffector_z.last());

    // remove data of lines that's outside visible range:
    ui_.plot_end->graph(0)->removeDataBefore(endeffector_time.last()-60);
    ui_.plot_end->graph(2)->removeDataBefore(endeffector_time.last()-60);
    ui_.plot_end->graph(4)->removeDataBefore(endeffector_time.last()-60);

    // rescale value (vertical) axis to fit the current data:
    ui_.plot_end->graph(0)->rescaleValueAxis();
    ui_.plot_end->graph(2)->rescaleValueAxis(true);
    ui_.plot_end->graph(4)->rescaleValueAxis(true);
    ui_.plot_end->graph(6)->rescaleValueAxis(true);
    ui_.plot_end->graph(8)->rescaleValueAxis(true);
    ui_.plot_end->graph(10)->rescaleValueAxis(true);
    ui_.plot_end->graph(12)->rescaleValueAxis(true);
    ui_.plot_end->graph(14)->rescaleValueAxis(true);
    ui_.plot_end->graph(16)->rescaleValueAxis(true);

    // make key axis range scroll with the data (at a constant range size of 30):
    ui_.plot_end->xAxis->setRange(endeffector_time.last()+0.1, 60, Qt::AlignRight);
    ui_.plot_end->replot();

    //add label for cavity coords
    ui_.label_coord_x_end->setText(QString::number(endeffector_x.last()));
    ui_.label_coord_y_end->setText(QString::number(endeffector_y.last()));
    ui_.label_coord_z_end->setText(QString::number(endeffector_z.last()));
}

void MainWindow::updateDesiredEndeffectorPoses(const QVector<double> &desired_endeffector_x, const QVector<double> &desired_endeffector_y, const QVector<double> &desired_endeffector_z, const QVector<double> &desired_endeffector_time)
{
    //update plot
    // add data for desired endeffector x-coord
    ui_.plot_end->graph(6)->addData(desired_endeffector_time, desired_endeffector_x);
    ui_.plot_end->graph(7)->clearData();
    ui_.plot_end->graph(7)->addData(desired_endeffector_time.last(), desired_endeffector_x.last());

    // add data for desired endeffector y-coord
    ui_.plot_end->graph(8)->addData(desired_endeffector_time, desired_endeffector_y);
    ui_.plot_end->graph(9)->clearData();
    ui_.plot_end->graph(9)->addData(desired_endeffector_time.last(), desired_endeffector_y.last());

    // add data for desired endeffector z-coord
    ui_.plot_end->graph(10)->addData(desired_endeffector_time, desired_endeffector_z);
    ui_.plot_end->graph(11)->clearData();
    ui_.plot_end->graph(11)->addData(desired_endeffector_time.last(), desired_endeffector_z.last());

    // remove data of lines that's outside visible range:
    ui_.plot_end->graph(6)->removeDataBefore(desired_endeffector_time.last()-60);
    ui_.plot_end->graph(8)->removeDataBefore(desired_endeffector_time.last()-60);
    ui_.plot_end->graph(10)->removeDataBefore(desired_endeffector_time.last()-60);

    // rescale value (vertical) axis to fit the current data:
    ui_.plot_end->graph(0)->rescaleValueAxis();
    ui_.plot_end->graph(2)->rescaleValueAxis(true);
    ui_.plot_end->graph(4)->rescaleValueAxis(true);
    ui_.plot_end->graph(6)->rescaleValueAxis(true);
    ui_.plot_end->graph(8)->rescaleValueAxis(true);
    ui_.plot_end->graph(10)->rescaleValueAxis(true);
    ui_.plot_end->graph(12)->rescaleValueAxis(true);
    ui_.plot_end->graph(14)->rescaleValueAxis(true);
    ui_.plot_end->graph(16)->rescaleValueAxis(true);

    //add label for cavity coords
    ui_.label_coord_x_desired_end->setText(QString::number(desired_endeffector_x.last()));
    ui_.label_coord_y_desired_end->setText(QString::number(desired_endeffector_y.last()));
    ui_.label_coord_z_desired_end->setText(QString::number(desired_endeffector_z.last()));

    double err_x = std::abs(desired_endeffector_x.last() - ui_.label_coord_x_end->text().toDouble());
    double err_y = std::abs(desired_endeffector_y.last() - ui_.label_coord_y_end->text().toDouble());
    double err_z = std::abs(desired_endeffector_z.last() - ui_.label_coord_z_end->text().toDouble());
    ui_.label_err_x_desired_end->setText(QString::number(err_x,'f',4));
    ui_.label_err_y_desired_end->setText(QString::number(err_y,'f',4));
    ui_.label_err_z_desired_end->setText(QString::number(err_z,'f',4));
}

void MainWindow::updateGtCavityPoses(const QVector<double> &gt_cavity_x, const QVector<double> &gt_cavity_y, const QVector<double> &gt_cavity_z, const QVector<double> &gt_cavity_time)
{
    //update plot
    // add data for gt cavity x-coord
    ui_.plot_end->graph(12)->addData(gt_cavity_time, gt_cavity_x);
    ui_.plot_end->graph(13)->clearData();
    ui_.plot_end->graph(13)->addData(gt_cavity_time.last(), gt_cavity_x.last());

    // add data for gt cavity y-coord
    ui_.plot_end->graph(14)->addData(gt_cavity_time, gt_cavity_y);
    ui_.plot_end->graph(15)->clearData();
    ui_.plot_end->graph(15)->addData(gt_cavity_time.last(), gt_cavity_y.last());

    // add data for gt cavity z-coord
    ui_.plot_end->graph(16)->addData(gt_cavity_time, gt_cavity_z);
    ui_.plot_end->graph(17)->clearData();
    ui_.plot_end->graph(17)->addData(gt_cavity_time.last(), gt_cavity_z.last());

    // remove data of lines that's outside visible range:
    ui_.plot_end->graph(12)->removeDataBefore(gt_cavity_time.last()-60);
    ui_.plot_end->graph(14)->removeDataBefore(gt_cavity_time.last()-60);
    ui_.plot_end->graph(16)->removeDataBefore(gt_cavity_time.last()-60);

    // rescale value (vertical) axis to fit the current data:
    ui_.plot_end->graph(0)->rescaleValueAxis();
    ui_.plot_end->graph(2)->rescaleValueAxis(true);
    ui_.plot_end->graph(4)->rescaleValueAxis(true);
    ui_.plot_end->graph(6)->rescaleValueAxis(true);
    ui_.plot_end->graph(8)->rescaleValueAxis(true);
    ui_.plot_end->graph(10)->rescaleValueAxis(true);
    ui_.plot_end->graph(12)->rescaleValueAxis(true);
    ui_.plot_end->graph(14)->rescaleValueAxis(true);
    ui_.plot_end->graph(16)->rescaleValueAxis(true);

    //add label for cavity coords
    ui_.label_coord_x_gt_cavity->setText(QString::number(gt_cavity_x.last()));
    ui_.label_coord_y_gt_cavity->setText(QString::number(gt_cavity_y.last()));
    ui_.label_coord_z_gt_cavity->setText(QString::number(gt_cavity_z.last()));

    double err_x = std::abs(gt_cavity_x.last() - ui_.label_coord_x_end->text().toDouble());
    double err_y = std::abs(gt_cavity_y.last() - ui_.label_coord_y_end->text().toDouble());
    double err_z = std::abs(gt_cavity_z.last() - ui_.label_coord_z_end->text().toDouble());
    ui_.label_err_x_gt_cavity->setText(QString::number(err_x,'f',4));
    ui_.label_err_y_gt_cavity->setText(QString::number(err_y,'f',4));
    ui_.label_err_z_gt_cavity->setText(QString::number(err_z,'f',4));

    //update plot
    // add data for gt_cavity robot x-coord
    ui_.plot->graph(6)->addData(gt_cavity_time, gt_cavity_x);
    ui_.plot->graph(7)->clearData();
    ui_.plot->graph(7)->addData(gt_cavity_time.last(), gt_cavity_x.last());

    // add data for gt_cavity robot y-coord
    ui_.plot->graph(8)->addData(gt_cavity_time, gt_cavity_y);
    ui_.plot->graph(9)->clearData();
    ui_.plot->graph(9)->addData(gt_cavity_time.last(), gt_cavity_y.last());

    // add data for gt_cavity robot z-coord
    ui_.plot->graph(10)->addData(gt_cavity_time, gt_cavity_z);
    ui_.plot->graph(11)->clearData();
    ui_.plot->graph(11)->addData(gt_cavity_time.last(), gt_cavity_z.last());

    // remove data of lines that's outside visible range:
    ui_.plot->graph(6)->removeDataBefore(gt_cavity_time.last()-60);
    ui_.plot->graph(8)->removeDataBefore(gt_cavity_time.last()-60);
    ui_.plot->graph(10)->removeDataBefore(gt_cavity_time.last()-60);

    // rescale value (vertical) axis to fit the current data:
    ui_.plot->graph(0)->rescaleValueAxis();
    ui_.plot->graph(2)->rescaleValueAxis(true);
    ui_.plot->graph(4)->rescaleValueAxis(true);
    ui_.plot->graph(6)->rescaleValueAxis(true);
    ui_.plot->graph(8)->rescaleValueAxis(true);
    ui_.plot->graph(10)->rescaleValueAxis(true);

    err_x = std::abs(gt_cavity_x.last() - ui_.label_coord_x->text().toDouble());
    err_y = std::abs(gt_cavity_y.last() - ui_.label_coord_y->text().toDouble());
    err_z = std::abs(gt_cavity_z.last() - ui_.label_coord_z->text().toDouble());
    ui_.label_err_x->setText(QString::number(err_x,'f',4));
    ui_.label_err_y->setText(QString::number(err_y,'f',4));
    ui_.label_err_z->setText(QString::number(err_z,'f',4));
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About ..."),tr("<h2>Remote Cavity Inspection</h2><p>Kelly Steich - steichk@ethz.ch</p><p>Tree cavity inspection using a multirotor system</p><p>DRZ, WSL, ASL</p>"));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    sensor_image_view_.abortThread();
    sensor_pointcloud_view_.abortThread();
    cavity_pose_view_.abortThread();

    QMainWindow::closeEvent(event);
}


void MainWindow::on_button_refresh_tree_clicked(bool check)
{
    std::cout << "Requesting service get_tree_detection_params..." << std::endl;
    rci_comm::GetTreeDetectionParams srv;
    if (ros::service::call("gui/get_tree_detection_params", srv))
    {
        std::cout << "Request get_tree_detection_params was successfull!" << std::endl;
        ui_.lineEdit_min_depth->setEnabled(true);
        ui_.lineEdit_thresh_seed->setEnabled(true);
        ui_.lineEdit_thresh_depth->setEnabled(true);
        ui_.lineEdit_max_scaling->setEnabled(true);
        ui_.lineEdit_min_cavity_width->setEnabled(true);
        ui_.lineEdit_min_cavity_height->setEnabled(true);
        ui_.lineEdit_kmean_k->setEnabled(true);
        ui_.lineEdit_kmeans_tol->setEnabled(true);
        ui_.lineEdit_kmeans_max_trys->setEnabled(true);

        ui_.lineEdit_min_depth->setText(QString::number(srv.response.min_depth));
        ui_.lineEdit_thresh_seed->setText(QString::number(srv.response.thresh_seed));
        ui_.lineEdit_thresh_depth->setText(QString::number(srv.response.thresh_depth));
        ui_.lineEdit_max_scaling->setText(QString::number(srv.response.max_scaling));
        ui_.lineEdit_min_cavity_width->setText(QString::number(srv.response.min_cavity_width));
        ui_.lineEdit_min_cavity_height->setText(QString::number(srv.response.min_cavity_height));
        ui_.lineEdit_kmean_k->setText(QString::number(srv.response.k));
        ui_.lineEdit_kmeans_tol->setText(QString::number(srv.response.tol));
        ui_.lineEdit_kmeans_max_trys->setText(QString::number(srv.response.max_trys));
    }
    else
    {
        std::cout << "[Error] Failed to call service get_tree_detection_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_min_depth_editingFinished()
{
    std::cout << "Requesting service set_tree_detection_params for min_depth=" << ui_.lineEdit_min_depth->text().toStdString() << std::endl;
    rci_comm::SetTreeDetectionParams srv;
    srv.request.min_depth = ui_.lineEdit_min_depth->text().toDouble();
    srv.request.thresh_seed = 1000;
    srv.request.thresh_depth = 1000;
    srv.request.max_scaling = 1000;
    srv.request.min_cavity_width = 1000;
    srv.request.min_cavity_height = 1000;
    srv.request.k = 1000;
    srv.request.tol = 1000;
    srv.request.max_trys = 1000;
    if (ros::service::call("gui/set_tree_detection_params", srv))
    {
        std::cout << "Request set_tree_detection_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_tree_detection_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_thresh_seed_editingFinished()
{
    std::cout << "Requesting service set_tree_detection_params for thresh_seed=" << ui_.lineEdit_thresh_seed->text().toStdString() << std::endl;
    rci_comm::SetTreeDetectionParams srv;
    srv.request.thresh_seed = ui_.lineEdit_thresh_seed->text().toDouble();
    srv.request.thresh_depth = 1000;
    srv.request.max_scaling = 1000;
    srv.request.min_cavity_width = 1000;
    srv.request.min_cavity_height = 1000;
    srv.request.k = 1000;
    srv.request.tol = 1000;
    srv.request.max_trys = 1000;
    if (ros::service::call("gui/set_tree_detection_params", srv))
    {
        std::cout << "Request set_tree_detection_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_tree_detection_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_thresh_depth_editingFinished()
{
    std::cout << "Requesting service set_tree_detection_params for thresh_depth=" << ui_.lineEdit_thresh_depth->text().toStdString() << std::endl;
    rci_comm::SetTreeDetectionParams srv;
    srv.request.thresh_seed = 1000;
    srv.request.thresh_depth = ui_.lineEdit_thresh_depth->text().toDouble();
    srv.request.max_scaling = 1000;
    srv.request.min_cavity_width = 1000;
    srv.request.min_cavity_height = 1000;
    srv.request.k = 1000;
    srv.request.tol = 1000;
    srv.request.max_trys = 1000;
    if (ros::service::call("gui/set_tree_detection_params", srv))
    {
        std::cout << "Request set_tree_detection_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_tree_detection_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_max_scaling_editingFinished()
{
    std::cout << "Requesting service set_tree_detection_params for max_scaling=" << ui_.lineEdit_max_scaling->text().toStdString() << std::endl;
    rci_comm::SetTreeDetectionParams srv;
    srv.request.thresh_seed = 1000;
    srv.request.thresh_depth = 1000;
    srv.request.max_scaling = ui_.lineEdit_max_scaling->text().toDouble();
    srv.request.min_cavity_width = 1000;
    srv.request.min_cavity_height = 1000;
    srv.request.k = 1000;
    srv.request.tol = 1000;
    srv.request.max_trys = 1000;
    if (ros::service::call("gui/set_tree_detection_params", srv))
    {
        std::cout << "Request set_tree_detection_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_tree_detection_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_min_cavity_width_editingFinished()
{
    std::cout << "Requesting service set_tree_detection_params for thresh_depth=" << ui_.lineEdit_min_cavity_width->text().toStdString() << std::endl;
    rci_comm::SetTreeDetectionParams srv;
    srv.request.thresh_seed = 1000;
    srv.request.thresh_depth = 1000;
    srv.request.max_scaling = 1000;
    srv.request.min_cavity_width = ui_.lineEdit_min_cavity_width->text().toDouble();
    srv.request.min_cavity_height = 1000;
    srv.request.k = 1000;
    srv.request.tol = 1000;
    srv.request.max_trys = 1000;
    if (ros::service::call("gui/set_tree_detection_params", srv))
    {
        std::cout << "Request set_tree_detection_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_tree_detection_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_min_cavity_height_editingFinished()
{
    std::cout << "Requesting service set_tree_detection_params for thresh_depth=" << ui_.lineEdit_min_cavity_height->text().toStdString() << std::endl;
    rci_comm::SetTreeDetectionParams srv;
    srv.request.thresh_seed = 1000;
    srv.request.thresh_depth = 1000;
    srv.request.max_scaling = 1000;
    srv.request.min_cavity_width = 1000;
    srv.request.min_cavity_height = ui_.lineEdit_min_cavity_height->text().toDouble();
    srv.request.k = 1000;
    srv.request.tol = 1000;
    srv.request.max_trys = 1000;
    if (ros::service::call("gui/set_tree_detection_params", srv))
    {
        std::cout << "Request set_tree_detection_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_tree_detection_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_kmean_k_editingFinished()
{
    std::cout << "Requesting service set_tree_detection_params for thresh_depth=" << ui_.lineEdit_kmean_k->text().toStdString() << std::endl;
    rci_comm::SetTreeDetectionParams srv;
    srv.request.thresh_seed = 1000;
    srv.request.thresh_depth = 1000;
    srv.request.max_scaling = 1000;
    srv.request.min_cavity_width = 1000;
    srv.request.min_cavity_height = 1000;
    srv.request.k = ui_.lineEdit_kmean_k->text().toDouble();;
    srv.request.tol = 1000;
    srv.request.max_trys = 1000;
    if (ros::service::call("gui/set_tree_detection_params", srv))
    {
        std::cout << "Request set_tree_detection_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_tree_detection_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_kmeans_tol_editingFinished()
{
    std::cout << "Requesting service set_tree_detection_params for thresh_depth=" << ui_.lineEdit_kmeans_tol->text().toStdString() << std::endl;
    rci_comm::SetTreeDetectionParams srv;
    srv.request.thresh_seed = 1000;
    srv.request.thresh_depth = 1000;
    srv.request.max_scaling = 1000;
    srv.request.min_cavity_width = 1000;
    srv.request.min_cavity_height = 1000;
    srv.request.k = 1000;
    srv.request.tol = ui_.lineEdit_kmeans_tol->text().toDouble();
    srv.request.max_trys = 1000;
    if (ros::service::call("gui/set_tree_detection_params", srv))
    {
        std::cout << "Request set_tree_detection_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_tree_detection_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_kmeans_max_trys_editingFinished()
{
    std::cout << "Requesting service set_tree_detection_params for thresh_depth=" << ui_.lineEdit_kmeans_max_trys->text().toStdString() << std::endl;
    rci_comm::SetTreeDetectionParams srv;
    srv.request.thresh_seed = 1000;
    srv.request.thresh_depth = 1000;
    srv.request.max_scaling = 1000;
    srv.request.min_cavity_width = 1000;
    srv.request.min_cavity_height = 1000;
    srv.request.k = 1000;
    srv.request.tol = 1000;
    srv.request.max_trys = ui_.lineEdit_kmeans_max_trys->text().toDouble();
    if (ros::service::call("gui/set_tree_detection_params", srv))
    {
        std::cout << "Request set_tree_detection_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_tree_detection_params" << std::endl;
    }
}


void MainWindow::on_button_refresh_filters_clicked(bool checked)
{
    std::cout << "Requesting service get_filter_pipeline_params..." << std::endl;
    rci_comm::GetFilterPipelineParams srv;
    if (ros::service::call("gui/get_filter_pipeline_params", srv))
    {
        std::cout << "Request get_filter_pipeline_params was successfull!" << std::endl;
        ui_.lineEdit_passthrough_min->setEnabled(true);
        ui_.lineEdit_passthrough_max->setEnabled(true);
        ui_.lineEdit_voxelgrid_leafsize->setEnabled(true);
        ui_.lineEdit_radial_min_neighbors->setEnabled(true);
        ui_.lineEdit_radial_radius_search->setEnabled(true);
        ui_.radioButton_passthrough->setEnabled(true);
        ui_.radioButton_voxelgrid->setEnabled(true);
        ui_.radioButton_radial->setEnabled(true);

        ui_.radioButton_passthrough->setChecked(srv.response.use_passthrough);
        ui_.radioButton_voxelgrid->setChecked(srv.response.use_voxelgrid);
        ui_.radioButton_radial->setChecked(srv.response.use_radial);
        ui_.lineEdit_passthrough_min->setText(QString::number(srv.response.passthrough_min));
        ui_.lineEdit_passthrough_max->setText(QString::number(srv.response.passthrough_max));
        ui_.lineEdit_voxelgrid_leafsize->setText(QString::number(srv.response.voxelgrid_leafsize));
        ui_.lineEdit_radial_min_neighbors->setText(QString::number(srv.response.radial_min_neighbors));
        ui_.lineEdit_radial_radius_search->setText(QString::number(srv.response.radial_radius_search));
    }
    else
    {
        std::cout << "[Error] Failed to call service get_filter_pipeline_params" << std::endl;
    }
}

void MainWindow::on_radioButton_passthrough_toggled(bool checked)
{
    std::cout << "Requesting service set_filter_pipeline_params for use_passthrough=" << ui_.radioButton_passthrough->isChecked() << std::endl;
    rci_comm::SetFilterPipelineParams srv;
    srv.request.use_passthrough = ui_.radioButton_passthrough->isChecked();
    srv.request.use_voxelgrid = ui_.radioButton_voxelgrid->isChecked();
    srv.request.use_radial = ui_.radioButton_radial->isChecked();
    srv.request.passthrough_min = 1000;
    srv.request.passthrough_max = 1000;
    srv.request.voxelgrid_leafsize = 1000;
    srv.request.radial_min_neighbors = 1000;
    srv.request.radial_radius_search = 1000;
    if (ros::service::call("gui/set_filter_pipeline_params", srv))
    {
        std::cout << "Request set_filter_pipeline_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_filter_pipeline_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_passthrough_min_editingFinished()
{
    std::cout << "Requesting service set_filter_pipeline_params for passthrough_min=" << ui_.lineEdit_passthrough_min->text().toStdString() << std::endl;
    rci_comm::SetFilterPipelineParams srv;
    srv.request.use_passthrough = ui_.radioButton_passthrough->isChecked();
    srv.request.use_voxelgrid = ui_.radioButton_voxelgrid->isChecked();
    srv.request.use_radial = ui_.radioButton_radial->isChecked();
    srv.request.passthrough_min = ui_.lineEdit_passthrough_min->text().toDouble();
    srv.request.passthrough_max = 1000;
    srv.request.voxelgrid_leafsize = 1000;
    srv.request.radial_min_neighbors = 1000;
    srv.request.radial_radius_search = 1000;
    if (ros::service::call("gui/set_filter_pipeline_params", srv))
    {
        std::cout << "Request set_filter_pipeline_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_filter_pipeline_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_passthrough_max_editingFinished()
{
    std::cout << "Requesting service set_filter_pipeline_params for passthrough_ax=" << ui_.lineEdit_passthrough_max->text().toStdString() << std::endl;
    rci_comm::SetFilterPipelineParams srv;
    srv.request.use_passthrough = ui_.radioButton_passthrough->isChecked();
    srv.request.use_voxelgrid = ui_.radioButton_voxelgrid->isChecked();
    srv.request.use_radial = ui_.radioButton_radial->isChecked();
    srv.request.passthrough_min = 1000;
    srv.request.passthrough_max = ui_.lineEdit_passthrough_max->text().toDouble();
    srv.request.voxelgrid_leafsize = 1000;
    srv.request.radial_min_neighbors = 1000;
    srv.request.radial_radius_search = 1000;
    if (ros::service::call("gui/set_filter_pipeline_params", srv))
    {
        std::cout << "Request set_filter_pipeline_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_filter_pipeline_params" << std::endl;
    }
}

void MainWindow::on_radioButton_voxelgrid_toggled(bool checked)
{
    std::cout << "Requesting service set_filter_pipeline_params for use_voxelgrid=" << ui_.radioButton_voxelgrid->isChecked() << std::endl;
    rci_comm::SetFilterPipelineParams srv;
    srv.request.use_passthrough = ui_.radioButton_passthrough->isChecked();
    srv.request.use_voxelgrid = ui_.radioButton_voxelgrid->isChecked();
    srv.request.use_radial = ui_.radioButton_radial->isChecked();
    srv.request.passthrough_min = 1000;
    srv.request.passthrough_max = 1000;
    srv.request.voxelgrid_leafsize = 1000;
    srv.request.radial_min_neighbors = 1000;
    srv.request.radial_radius_search = 1000;
    if (ros::service::call("gui/set_filter_pipeline_params", srv))
    {
        std::cout << "Request set_filter_pipeline_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_filter_pipeline_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_voxelgrid_leafsize_editingFinished()
{
    std::cout << "Requesting service set_filter_pipeline_params for leafsize=" << ui_.lineEdit_voxelgrid_leafsize->text().toStdString() << std::endl;
    rci_comm::SetFilterPipelineParams srv;
    srv.request.use_passthrough = ui_.radioButton_passthrough->isChecked();
    srv.request.use_voxelgrid = ui_.radioButton_voxelgrid->isChecked();
    srv.request.use_radial = ui_.radioButton_radial->isChecked();
    srv.request.passthrough_min = 1000;
    srv.request.passthrough_max = 1000;
    srv.request.voxelgrid_leafsize = ui_.lineEdit_voxelgrid_leafsize->text().toDouble();
    srv.request.radial_min_neighbors = 1000;
    srv.request.radial_radius_search = 1000;
    if (ros::service::call("gui/set_filter_pipeline_params", srv))
    {
        std::cout << "Request set_filter_pipeline_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_filter_pipeline_params" << std::endl;
    }
}

void MainWindow::on_radioButton_radial_toggled(bool checked)
{
    std::cout << "Requesting service set_filter_pipeline_params for use_radial=" << ui_.radioButton_radial->isChecked() << std::endl;
    rci_comm::SetFilterPipelineParams srv;
    srv.request.use_passthrough = ui_.radioButton_passthrough->isChecked();
    srv.request.use_voxelgrid = ui_.radioButton_voxelgrid->isChecked();
    srv.request.use_radial = ui_.radioButton_radial->isChecked();
    srv.request.passthrough_min = 1000;
    srv.request.passthrough_max = 1000;
    srv.request.voxelgrid_leafsize = 1000;
    srv.request.radial_min_neighbors = 1000;
    srv.request.radial_radius_search = 1000;
    if (ros::service::call("gui/set_filter_pipeline_params", srv))
    {
        std::cout << "Request set_filter_pipeline_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_filter_pipeline_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_radial_min_neighbors_editingFinished()
{
    std::cout << "Requesting service set_filter_pipeline_params for min_neighbors=" << ui_.lineEdit_radial_min_neighbors->text().toStdString() << std::endl;
    rci_comm::SetFilterPipelineParams srv;
    srv.request.use_passthrough = ui_.radioButton_passthrough->isChecked();
    srv.request.use_voxelgrid = ui_.radioButton_voxelgrid->isChecked();
    srv.request.use_radial = ui_.radioButton_radial->isChecked();
    srv.request.passthrough_min = 1000;
    srv.request.passthrough_max = 1000;
    srv.request.voxelgrid_leafsize = 1000;
    srv.request.radial_min_neighbors = ui_.lineEdit_radial_min_neighbors->text().toDouble();
    srv.request.radial_radius_search = 1000;
    if (ros::service::call("gui/set_filter_pipeline_params", srv))
    {
        std::cout << "Request set_filter_pipeline_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_filter_pipeline_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_radial_radius_search_editingFinished()
{
    std::cout << "Requesting service set_filter_pipeline_params for radius_search=" << ui_.lineEdit_radial_radius_search->text().toStdString() << std::endl;
    rci_comm::SetFilterPipelineParams srv;
    srv.request.use_passthrough = ui_.radioButton_passthrough->isChecked();
    srv.request.use_voxelgrid = ui_.radioButton_voxelgrid->isChecked();
    srv.request.use_radial = ui_.radioButton_radial->isChecked();
    srv.request.passthrough_min = 1000;
    srv.request.passthrough_max = 1000;
    srv.request.voxelgrid_leafsize = 1000;
    srv.request.radial_min_neighbors = 1000;
    srv.request.radial_radius_search = ui_.lineEdit_radial_radius_search->text().toDouble();
    if (ros::service::call("gui/set_filter_pipeline_params", srv))
    {
        std::cout << "Request set_filter_pipeline_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_filter_pipeline_params" << std::endl;
    }
}


void MainWindow::on_button_refresh_box_normal_clicked(bool checked)
{
    std::cout << "Requesting service get_box_check_params and get_cavity_normal_params..." << std::endl;
    rci_comm::GetCavityNormalParams srv_normal;
    if (ros::service::call("gui/get_cavity_normal_params", srv_normal))
    {
        std::cout << "Request get_cavity_normal_params was successfull!" << std::endl;
        ui_.lineEdit_band_width->setEnabled(true);

        ui_.lineEdit_band_width->setText(QString::number(srv_normal.response.band_width));
    }
    else
    {
        std::cout << "[Error] Failed to call service get_tree_detection_params" << std::endl;
    }
    rci_comm::GetBoxCheckParams srv_box;
    if (ros::service::call("gui/get_box_check_params", srv_box))
    {
        std::cout << "Request get_box_check_params was successfull!" << std::endl;
        ui_.lineEdit_octree_resolution->setEnabled(true);
        ui_.lineEdit_initial_step->setEnabled(true);
        ui_.lineEdit_max_inliers->setEnabled(true);
        ui_.lineEdit_start_box_width->setEnabled(true);
        ui_.lineEdit_start_box_height->setEnabled(true);
        ui_.lineEdit_start_box_length->setEnabled(true);

        ui_.lineEdit_octree_resolution->setText(QString::number(srv_box.response.octree_resolution));
        ui_.lineEdit_initial_step->setText(QString::number(srv_box.response.initial_step));
        ui_.lineEdit_max_inliers->setText(QString::number(srv_box.response.max_n_inliers));
        ui_.lineEdit_start_box_width->setText(QString::number(srv_box.response.start_box_width));
        ui_.lineEdit_start_box_height->setText(QString::number(srv_box.response.start_box_height));
        ui_.lineEdit_start_box_length->setText(QString::number(srv_box.response.start_box_length));
    }
    else
    {
        std::cout << "[Error] Failed to call service get_tree_detection_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_octree_resolution_editingFinished()
{
    std::cout << "Requesting service set_box_check_params for octree_resolution=" << ui_.lineEdit_octree_resolution->text().toStdString() << std::endl;
    rci_comm::SetBoxCheckParams srv;
    srv.request.octree_resolution = ui_.lineEdit_octree_resolution->text().toDouble();
    srv.request.initial_step = 1000;
    srv.request.max_n_inliers = 1000;
    srv.request.start_box_width = 1000;
    srv.request.start_box_height = 1000;
    srv.request.start_box_length = 1000;
    if (ros::service::call("gui/set_box_check_params", srv))
    {
        std::cout << "Request set_box_check_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_box_check_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_initial_step_editingFinished()
{
    std::cout << "Requesting service set_box_check_params for initial_step=" << ui_.lineEdit_initial_step->text().toStdString() << std::endl;
    rci_comm::SetBoxCheckParams srv;
    srv.request.octree_resolution = 1000;
    srv.request.initial_step = ui_.lineEdit_initial_step->text().toDouble();
    srv.request.max_n_inliers = 1000;
    srv.request.start_box_width = 1000;
    srv.request.start_box_height = 1000;
    srv.request.start_box_length = 1000;
    if (ros::service::call("gui/set_box_check_params", srv))
    {
        std::cout << "Request set_box_check_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_box_check_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_max_inliers_editingFinished()
{
    std::cout << "Requesting service set_box_check_params for max_inliers=" << ui_.lineEdit_max_inliers->text().toStdString() << std::endl;
    rci_comm::SetBoxCheckParams srv;
    srv.request.octree_resolution = 1000;
    srv.request.initial_step = 1000;
    srv.request.max_n_inliers = ui_.lineEdit_max_inliers->text().toDouble();
    srv.request.start_box_width = 1000;
    srv.request.start_box_height = 1000;
    srv.request.start_box_length = 1000;
    if (ros::service::call("gui/set_box_check_params", srv))
    {
        std::cout << "Request set_box_check_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_box_check_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_start_box_width_editingFinished()
{
    std::cout << "Requesting service set_box_check_params for start_box_width=" << ui_.lineEdit_start_box_width->text().toStdString() << std::endl;
    rci_comm::SetBoxCheckParams srv;
    srv.request.octree_resolution = 1000;
    srv.request.initial_step = 1000;
    srv.request.max_n_inliers = 1000;
    srv.request.start_box_width = ui_.lineEdit_start_box_width->text().toDouble();
    srv.request.start_box_height = 1000;
    srv.request.start_box_length = 1000;
    if (ros::service::call("gui/set_box_check_params", srv))
    {
        std::cout << "Request set_box_check_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_box_check_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_start_box_height_editingFinished()
{
    std::cout << "Requesting service set_box_check_params for start_box_height=" << ui_.lineEdit_start_box_height->text().toStdString() << std::endl;
    rci_comm::SetBoxCheckParams srv;
    srv.request.octree_resolution = 1000;
    srv.request.initial_step = 1000;
    srv.request.max_n_inliers = 1000;
    srv.request.start_box_width = 1000;
    srv.request.start_box_height = ui_.lineEdit_start_box_height->text().toDouble();
    srv.request.start_box_length = 1000;
    if (ros::service::call("gui/set_box_check_params", srv))
    {
        std::cout << "Request set_box_check_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_box_check_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_start_box_length_editingFinished()
{
    std::cout << "Requesting service set_box_check_params for start_box_length=" << ui_.lineEdit_start_box_length->text().toStdString() << std::endl;
    rci_comm::SetBoxCheckParams srv;
    srv.request.octree_resolution = 1000;
    srv.request.initial_step = 1000;
    srv.request.max_n_inliers = 1000;
    srv.request.start_box_width = 1000;
    srv.request.start_box_height = 1000;
    srv.request.start_box_length = ui_.lineEdit_start_box_length->text().toDouble();
    if (ros::service::call("gui/set_box_check_params", srv))
    {
        std::cout << "Request set_box_check_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_box_check_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_band_width_editingFinished()
{
    std::cout << "Requesting service set_cavity_normal_params for bandwidth=" << ui_.lineEdit_band_width->text().toStdString() << std::endl;
    rci_comm::SetCavityNormalParams srv;
    srv.request.band_width = ui_.lineEdit_band_width->text().toDouble();
    if (ros::service::call("gui/set_cavity_normal_params", srv))
    {
        std::cout << "Request set_cavity_normal_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_cavity_normal_params" << std::endl;
    }
}


void MainWindow::on_button_refresh_kalman_clicked(bool checked)
{
    std::cout << "Requesting service get_kalman_params..." << std::endl;
    rci_comm::GetKalmanParams srv;
    if (ros::service::call("gui/get_kalman_params", srv))
    {
        std::cout << "Request get_kalman_params was successfull!" << std::endl;
        ui_.radioButton_fixed_q->setEnabled(true);
        ui_.radioButton_fixed_q->setChecked(srv.response.fixed_q);
        if(srv.response.fixed_q)
        {
            ui_.lineEdit_qx->setEnabled(true);
            ui_.lineEdit_qy->setEnabled(true);
            ui_.lineEdit_qz->setEnabled(true);
        }
        else
        {
            ui_.lineEdit_qx->setEnabled(false);
            ui_.lineEdit_qy->setEnabled(false);
            ui_.lineEdit_qz->setEnabled(false);
        }

        ui_.lineEdit_rx->setEnabled(true);
        ui_.lineEdit_ry->setEnabled(true);
        ui_.lineEdit_rz->setEnabled(true);

        ui_.lineEdit_qx->setText(QString::number(srv.response.qx));
        ui_.lineEdit_qy->setText(QString::number(srv.response.qy));
        ui_.lineEdit_qz->setText(QString::number(srv.response.qz));
        ui_.lineEdit_rx->setText(QString::number(srv.response.rx));
        ui_.lineEdit_ry->setText(QString::number(srv.response.ry));
        ui_.lineEdit_rz->setText(QString::number(srv.response.rz));
    }
    else
    {
        std::cout << "[Error] Failed to call service get_kalman_params" << std::endl;
    }
}

void MainWindow::on_radioButton_fixed_q_toggled(bool checked)
{
    if(ui_.radioButton_fixed_q->isChecked())
    {
        ui_.lineEdit_qx->setEnabled(true);
        ui_.lineEdit_qy->setEnabled(true);
        ui_.lineEdit_qz->setEnabled(true);
    }
    else
    {
        ui_.lineEdit_qx->setEnabled(false);
        ui_.lineEdit_qy->setEnabled(false);
        ui_.lineEdit_qz->setEnabled(false);
    }
    std::cout << "Requesting service set_kalman_params for fixed_q=" << ui_.radioButton_fixed_q->isChecked() << std::endl;
    rci_comm::SetKalmanParams srv;
    srv.request.fixed_q = ui_.radioButton_fixed_q->isChecked();
    srv.request.qx = 1000;
    srv.request.qy = 1000;
    srv.request.qz = 1000;
    srv.request.rx = 1000;
    srv.request.ry = 1000;
    srv.request.rz = 1000;
    if (ros::service::call("gui/set_kalman_params", srv))
    {
        std::cout << "Request set_kalman_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_kalman_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_qx_editingFinished()
{
    std::cout << "Requesting service set_kalman_params for qx=" << ui_.lineEdit_qx->text().toStdString() << std::endl;
    rci_comm::SetKalmanParams srv;
    srv.request.fixed_q = ui_.radioButton_fixed_q->isChecked();
    srv.request.qx = ui_.lineEdit_qx->text().toDouble();
    srv.request.qy = 1000;
    srv.request.qz = 1000;
    srv.request.rx = 1000;
    srv.request.ry = 1000;
    srv.request.rz = 1000;
    if (ros::service::call("gui/set_kalman_params", srv))
    {
        std::cout << "Request set_kalman_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_kalman_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_qy_editingFinished()
{
    std::cout << "Requesting service set_kalman_params for qy=" << ui_.lineEdit_qy->text().toStdString() << std::endl;
    rci_comm::SetKalmanParams srv;
    srv.request.fixed_q = ui_.radioButton_fixed_q->isChecked();
    srv.request.qx = 1000;
    srv.request.qy = ui_.lineEdit_qy->text().toDouble();
    srv.request.qz = 1000;
    srv.request.rx = 1000;
    srv.request.ry = 1000;
    srv.request.rz = 1000;
    if (ros::service::call("gui/set_kalman_params", srv))
    {
        std::cout << "Request set_kalman_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_kalman_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_qz_editingFinished()
{
    std::cout << "Requesting service set_kalman_params for qz=" << ui_.lineEdit_qz->text().toStdString() << std::endl;
    rci_comm::SetKalmanParams srv;
    srv.request.fixed_q = ui_.radioButton_fixed_q->isChecked();
    srv.request.qx = 1000;
    srv.request.qy = 1000;
    srv.request.qz = ui_.lineEdit_qz->text().toDouble();
    srv.request.rx = 1000;
    srv.request.ry = 1000;
    srv.request.rz = 1000;
    if (ros::service::call("gui/set_kalman_params", srv))
    {
        std::cout << "Request set_kalman_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_kalman_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_rx_editingFinished()
{
    std::cout << "Requesting service set_kalman_params for rx=" << ui_.lineEdit_rx->text().toStdString() << std::endl;
    rci_comm::SetKalmanParams srv;
    srv.request.fixed_q = ui_.radioButton_fixed_q->isChecked();
    srv.request.qx = 1000;
    srv.request.qy = 1000;
    srv.request.qz = 1000;
    srv.request.rx = ui_.lineEdit_rx->text().toDouble();
    srv.request.ry = 1000;
    srv.request.rz = 1000;
    if (ros::service::call("gui/set_kalman_params", srv))
    {
        std::cout << "Request set_kalman_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_kalman_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_ry_editingFinished()
{
    std::cout << "Requesting service set_kalman_params for ry=" << ui_.lineEdit_ry->text().toStdString() << std::endl;
    rci_comm::SetKalmanParams srv;
    srv.request.fixed_q = ui_.radioButton_fixed_q->isChecked();
    srv.request.qx = 1000;
    srv.request.qy = 1000;
    srv.request.qz = 1000;
    srv.request.rx = 1000;
    srv.request.ry = ui_.lineEdit_ry->text().toDouble();
    srv.request.rz = 1000;
    if (ros::service::call("gui/set_kalman_params", srv))
    {
        std::cout << "Request set_kalman_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_kalman_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_rz_editingFinished()
{
    std::cout << "Requesting service set_kalman_params for rz=" << ui_.lineEdit_rz->text().toStdString() << std::endl;
    rci_comm::SetKalmanParams srv;
    srv.request.fixed_q = ui_.radioButton_fixed_q->isChecked();
    srv.request.qx = 1000;
    srv.request.qy = 1000;
    srv.request.qz = 1000;
    srv.request.rx = 1000;
    srv.request.ry = 1000;
    srv.request.rz = ui_.lineEdit_rz->text().toDouble();
    if (ros::service::call("gui/set_kalman_params", srv))
    {
        std::cout << "Request set_kalman_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_kalman_params" << std::endl;
    }
}


void MainWindow::on_button_refresh_controller_clicked(bool checked)
{
    std::cout << "Requesting service get_controller_params..." << std::endl;
    rci_comm::GetControllerParams srv;
    if (ros::service::call("gui/get_controller_params", srv))
    {
        std::cout << "Request get_controller_params was successfull!" << std::endl;
        ui_.lineEdit_desired_pitching_angle->setEnabled(true);
        ui_.lineEdit_robot_dist_to_tree->setEnabled(true);
        ui_.lineEdit_arm_pitching_tol->setEnabled(true);
        ui_.lineEdit_robot_position_tol_x->setEnabled(true);
        ui_.lineEdit_robot_position_tol_y->setEnabled(true);
        ui_.lineEdit_robot_position_tol_z->setEnabled(true);
        ui_.lineEdit_robot_orientation_tol->setEnabled(true);
        ui_.lineEdit_robot_offset_x->setEnabled(true);
        ui_.lineEdit_robot_offset_y->setEnabled(true);
        ui_.lineEdit_robot_offset_z->setEnabled(true);

        ui_.lineEdit_desired_pitching_angle->setText(QString::number(srv.response.desired_pitching_angle));
        ui_.lineEdit_robot_dist_to_tree->setText(QString::number(srv.response.robot_dist_to_tree));
        ui_.lineEdit_arm_pitching_tol->setText(QString::number(srv.response.arm_pitching_tol));
        ui_.lineEdit_robot_position_tol_x->setText(QString::number(srv.response.robot_position_tol_x));
        ui_.lineEdit_robot_position_tol_y->setText(QString::number(srv.response.robot_position_tol_y));
        ui_.lineEdit_robot_position_tol_z->setText(QString::number(srv.response.robot_position_tol_z));
        ui_.lineEdit_robot_orientation_tol->setText(QString::number(srv.response.robot_orientation_tol));
        ui_.lineEdit_robot_offset_x->setText(QString::number(srv.response.robot_offset_x));
        ui_.lineEdit_robot_offset_y->setText(QString::number(srv.response.robot_offset_y));
        ui_.lineEdit_robot_offset_z->setText(QString::number(srv.response.robot_offset_z));
    }
    else
    {
        std::cout << "[Error] Failed to call service get_controller_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_desired_pitching_angle_editingFinished()
{
    std::cout << "Requesting service set_controller_params for desired_pitching_angle=" << ui_.lineEdit_desired_pitching_angle->text().toStdString() << std::endl;
    rci_comm::SetControllerParams srv;
    srv.request.desired_pitching_angle = ui_.lineEdit_desired_pitching_angle->text().toDouble();
    srv.request.robot_dist_to_tree = 1000;
    srv.request.arm_pitching_tol = 1000;
    srv.request.robot_position_tol_x = 1000;
    srv.request.robot_position_tol_y = 1000;
    srv.request.robot_position_tol_z = 1000;
    srv.request.robot_orientation_tol = 1000;
    srv.request.robot_offset_x = 1000;
    srv.request.robot_offset_y = 1000;
    srv.request.robot_offset_z = 1000;
    if (ros::service::call("gui/set_controller_params", srv))
    {
        std::cout << "Request set_controller_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_controller_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_robot_dist_to_tree_editingFinished()
{
    std::cout << "Requesting service set_controller_params for robot_dist_to_tree=" << ui_.lineEdit_robot_dist_to_tree->text().toStdString() << std::endl;
    rci_comm::SetControllerParams srv;
    srv.request.desired_pitching_angle = 1000;
    srv.request.robot_dist_to_tree = ui_.lineEdit_robot_dist_to_tree->text().toDouble();
    srv.request.arm_pitching_tol = 1000;
    srv.request.robot_position_tol_x = 1000;
    srv.request.robot_position_tol_y = 1000;
    srv.request.robot_position_tol_z = 1000;
    srv.request.robot_orientation_tol = 1000;
    srv.request.robot_offset_x = 1000;
    srv.request.robot_offset_y = 1000;
    srv.request.robot_offset_z = 1000;
    if (ros::service::call("gui/set_controller_params", srv))
    {
        std::cout << "Request set_controller_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_controller_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_arm_pitching_tol_editingFinished()
{
    std::cout << "Requesting service set_controller_params for arm_pitching_tol=" << ui_.lineEdit_arm_pitching_tol->text().toStdString() << std::endl;
    rci_comm::SetControllerParams srv;
    srv.request.desired_pitching_angle = 1000;
    srv.request.robot_dist_to_tree = 1000;
    srv.request.arm_pitching_tol = ui_.lineEdit_arm_pitching_tol->text().toDouble();
    srv.request.robot_position_tol_x = 1000;
    srv.request.robot_position_tol_y = 1000;
    srv.request.robot_position_tol_z = 1000;
    srv.request.robot_orientation_tol = 1000;
    srv.request.robot_offset_x = 1000;
    srv.request.robot_offset_y = 1000;
    srv.request.robot_offset_z = 1000;
    if (ros::service::call("gui/set_controller_params", srv))
    {
        std::cout << "Request set_controller_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_controller_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_robot_position_tol_x_editingFinished()
{
    std::cout << "Requesting service set_controller_params for robot_position_tol=" << ui_.lineEdit_robot_position_tol_x->text().toStdString() << std::endl;
    rci_comm::SetControllerParams srv;
    srv.request.desired_pitching_angle = 1000;
    srv.request.robot_dist_to_tree = 1000;
    srv.request.arm_pitching_tol = 1000;
    srv.request.robot_position_tol_x = ui_.lineEdit_robot_position_tol_x->text().toDouble();
    srv.request.robot_position_tol_y = 1000;
    srv.request.robot_position_tol_z = 1000;
    srv.request.robot_orientation_tol = 1000;
    srv.request.robot_offset_x = 1000;
    srv.request.robot_offset_y = 1000;
    srv.request.robot_offset_z = 1000;
    if (ros::service::call("gui/set_controller_params", srv))
    {
        std::cout << "Request set_controller_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_controller_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_robot_position_tol_y_editingFinished()
{
    std::cout << "Requesting service set_controller_params for robot_position_tol=" << ui_.lineEdit_robot_position_tol_y->text().toStdString() << std::endl;
    rci_comm::SetControllerParams srv;
    srv.request.desired_pitching_angle = 1000;
    srv.request.robot_dist_to_tree = 1000;
    srv.request.arm_pitching_tol = 1000;
    srv.request.robot_position_tol_x = 1000;
    srv.request.robot_position_tol_y = ui_.lineEdit_robot_position_tol_y->text().toDouble();
    srv.request.robot_position_tol_z = 1000;
    srv.request.robot_orientation_tol = 1000;
    srv.request.robot_offset_x = 1000;
    srv.request.robot_offset_y = 1000;
    srv.request.robot_offset_z = 1000;
    if (ros::service::call("gui/set_controller_params", srv))
    {
        std::cout << "Request set_controller_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_controller_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_robot_position_tol_z_editingFinished()
{
    std::cout << "Requesting service set_controller_params for robot_position_tol=" << ui_.lineEdit_robot_position_tol_z->text().toStdString() << std::endl;
    rci_comm::SetControllerParams srv;
    srv.request.desired_pitching_angle = 1000;
    srv.request.robot_dist_to_tree = 1000;
    srv.request.arm_pitching_tol = 1000;
    srv.request.robot_position_tol_x = 1000;
    srv.request.robot_position_tol_y = 1000;
    srv.request.robot_position_tol_z = ui_.lineEdit_robot_position_tol_z->text().toDouble();
    srv.request.robot_orientation_tol = 1000;
    srv.request.robot_offset_x = 1000;
    srv.request.robot_offset_y = 1000;
    srv.request.robot_offset_z = 1000;
    if (ros::service::call("gui/set_controller_params", srv))
    {
        std::cout << "Request set_controller_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_controller_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_robot_orientation_tol_editingFinished()
{
    std::cout << "Requesting service set_controller_params for robot_orientation_tol=" << ui_.lineEdit_robot_orientation_tol->text().toStdString() << std::endl;
    rci_comm::SetControllerParams srv;
    srv.request.desired_pitching_angle = 1000;
    srv.request.robot_dist_to_tree = 1000;
    srv.request.arm_pitching_tol = 1000;
    srv.request.robot_position_tol_x = 1000;
    srv.request.robot_position_tol_y = 1000;
    srv.request.robot_position_tol_z = 1000;
    srv.request.robot_orientation_tol = ui_.lineEdit_robot_orientation_tol->text().toDouble();
    srv.request.robot_offset_x = 1000;
    srv.request.robot_offset_y = 1000;
    srv.request.robot_offset_z = 1000;
    if (ros::service::call("gui/set_controller_params", srv))
    {
        std::cout << "Request set_controller_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_controller_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_robot_offset_x_editingFinished()
{
    std::cout << "Requesting service set_controller_params for robot_offset_x=" << ui_.lineEdit_robot_offset_x->text().toStdString() << std::endl;
    rci_comm::SetControllerParams srv;
    srv.request.desired_pitching_angle = 1000;
    srv.request.robot_dist_to_tree = 1000;
    srv.request.arm_pitching_tol = 1000;
    srv.request.robot_position_tol_x = 1000;
    srv.request.robot_position_tol_y = 1000;
    srv.request.robot_position_tol_z = 1000;
    srv.request.robot_orientation_tol = 1000;
    srv.request.robot_offset_x = ui_.lineEdit_robot_offset_x->text().toDouble();
    srv.request.robot_offset_y = 1000;
    srv.request.robot_offset_z = 1000;
    if (ros::service::call("gui/set_controller_params", srv))
    {
        std::cout << "Request set_controller_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_controller_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_robot_offset_y_editingFinished()
{
    std::cout << "Requesting service set_controller_params for robot_offset_y=" << ui_.lineEdit_robot_offset_y->text().toStdString() << std::endl;
    rci_comm::SetControllerParams srv;
    srv.request.desired_pitching_angle = 1000;
    srv.request.robot_dist_to_tree = 1000;
    srv.request.arm_pitching_tol = 1000;
    srv.request.robot_position_tol_x = 1000;
    srv.request.robot_position_tol_y = 1000;
    srv.request.robot_position_tol_z = 1000;
    srv.request.robot_orientation_tol = 1000;
    srv.request.robot_offset_x = 1000;
    srv.request.robot_offset_y = ui_.lineEdit_robot_offset_y->text().toDouble();
    srv.request.robot_offset_z = 1000;
    if (ros::service::call("gui/set_controller_params", srv))
    {
        std::cout << "Request set_controller_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_controller_params" << std::endl;
    }
}

void MainWindow::on_lineEdit_robot_offset_z_editingFinished()
{
    std::cout << "Requesting service set_controller_params for robot_offset_z=" << ui_.lineEdit_robot_offset_z->text().toStdString() << std::endl;
    rci_comm::SetControllerParams srv;
    srv.request.desired_pitching_angle = 1000;
    srv.request.robot_dist_to_tree = 1000;
    srv.request.arm_pitching_tol = 1000;
    srv.request.robot_position_tol_x = 1000;
    srv.request.robot_position_tol_y = 1000;
    srv.request.robot_position_tol_z = 1000;
    srv.request.robot_orientation_tol = 1000;
    srv.request.robot_offset_x = 1000;
    srv.request.robot_offset_y = 1000;
    srv.request.robot_offset_z = ui_.lineEdit_robot_offset_z->text().toDouble();
    if (ros::service::call("gui/set_controller_params", srv))
    {
        std::cout << "Request set_controller_params was successfull!" << std::endl;
    }
    else
    {
        std::cout << "[Error] Failed to call service set_controller_params" << std::endl;
    }
}

}  // namespace rci_gui
