//=============================================================================
//  Author: Kelly Steich (steichk@student.ethz.ch)
//=============================================================================
#ifndef STRUCTURES_H
#define STRUCTURES_H

/*********************
** Logging
**********************/
enum LogLevel {
         Debug,
         Info,
         Warn,
         Error,
         Message
 };

struct Log{
    LogLevel level;
    std::string msg;

    Log() {}
    Log(LogLevel level_, std::string msg_) : level(level_) , msg(msg_) {}
};

#endif // STRUCTURES_H
