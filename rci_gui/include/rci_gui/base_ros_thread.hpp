//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
#ifndef BASE_ROS_THREAD_HPP
#define BASE_ROS_THREAD_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>

#include <ros/callback_queue.h>

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QTime>
#include <QStringListModel>

#include "structures.h"

#include <vector>
#include <numeric>
#include <iostream>

/*****************************************************************************
** Namespaces
*****************************************************************************/

/*! Base implementation of a ros node as Qt thread
 */
namespace rci_gui
{

/*****************************************************************************
** Class
*****************************************************************************/

class BaseRosThread : public QThread
{
    Q_OBJECT

public:

    /*! Constructor
     */
    BaseRosThread(int argc, char** argv, std::string node_name);

    /*! Virtual destructor
     */
    virtual ~BaseRosThread();

    /*! Initializes the node, starts ros
     */
    virtual bool init();

    /*! Outputs a message in the GUI
     */
    void log( const LogLevel &level, const std::string &msg);


Q_SIGNALS:

    /*! Signals the GUI that there is a new log message to display
     */
    void loggingUpdated(const struct Log &log);

public Q_SLOTS:

    /*! (Re)Starts the tread
     */
    void startThread();

    /*! Stops the tread
     */
    void finishThread();

    /*! Aborts the tread
     */
    void abortThread();

protected:

    /*! The main loop of the thread, starts a ros loop where callbacks are processed
     */
    virtual void run();

    int init_argc_;
    char** init_argv_;

    /*! Initializes ros communictaions (subscribers and publishers)
     */
    virtual void initRosComm(ros::NodeHandle n){};

    ros::CallbackQueue node_queue_; //individual callback queue for the node, ensures that each nodes callbacks are processed in it's own thread

    int nFrames_; //the number of processed frames

    std::string node_name_; //the node name
    double frequency_; //the frequency at which the ros node runs (in HZ)

    QMutex mutex_;
    QWaitCondition condition_;
    bool restart_, abort_, finished_;

    int rateGui_; //the rate at which the gui is updated
};

}

#endif // BASE_ROS_THREAD_HPP
