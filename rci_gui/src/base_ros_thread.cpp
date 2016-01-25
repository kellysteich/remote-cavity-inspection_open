//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/rci_gui/base_ros_thread.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rci_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

BaseRosThread::BaseRosThread(int argc, char** argv, std::string node_name) :
    node_name_(node_name),
    init_argc_(argc),
    init_argv_(argv)
{
    restart_ = false;
    abort_ = false;
    finished_ = false;

    nFrames_ = 0;

    frequency_ = 30;
    rateGui_ = 1; //msg frequency is divided by rateGui to get gui update rate
}

BaseRosThread::~BaseRosThread()
{
    mutex_.lock();
    abort_ = true;
    condition_.wakeOne();
    mutex_.unlock();

    wait();

    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    std::cout << node_name_ + " stopped" << std::endl;
}

bool BaseRosThread::init()
{
    ros::init(init_argc_,init_argv_,node_name_);
    if ( ! ros::master::check() )
        return false;

    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    n.setCallbackQueue(&node_queue_);

    initRosComm(n);

    log(Info, std::string(node_name_ + " initialized"));
    return true;
}

void BaseRosThread::startThread()
{
    if(!isRunning())
    {
        mutex_.lock();
        finished_ = false;
        mutex_.unlock();
        start();
    }
    else
    {
        mutex_.lock();
        restart_ = true;
        finished_ = false;
        condition_.wakeOne();
        mutex_.unlock();
    }
}

void BaseRosThread::finishThread()
{
    log(Info, std::string(node_name_ + " finished"));
    mutex_.lock();
    finished_ = true;
    mutex_.unlock();
}

void BaseRosThread::abortThread()
{
    log(Info, std::string(node_name_ + " aborted"));
    mutex_.lock();
    abort_ = true;
    mutex_.unlock();
}

void BaseRosThread::log( const LogLevel &level, const std::string &msg)
{
    struct Log log;
    log.level = level;
    log.msg = msg;
    Q_EMIT loggingUpdated(log);
}

void BaseRosThread::run()
{
    Q_FOREVER
    {
        log(Info, std::string(node_name_ + " is running"));
        ROS_INFO_STREAM(node_name_ << " thread: " << QThread::currentThreadId());

        ros::Rate loop_rate(frequency_);
        while ( ros::ok() )
        {
            if (restart_)
                break;
            if (finished_)
                break;
            if (abort_)
                return;

            //use this instead of spinOnce() to ensure that each thread's callbacks are processed only in this thread (i.e. each thread(/node) has its own callback queue
            node_queue_.callAvailable(ros::WallDuration());
            loop_rate.sleep();
        }
        log(Info, std::string(node_name_ + " stopped"));
        if (abort_)
            return;
        mutex_.lock();
        if (!restart_)
            condition_.wait(&mutex_);
        restart_ = false;
        mutex_.unlock();
    }
}

}
