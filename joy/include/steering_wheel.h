#ifndef STEERING_WHEEL_H
#define STEERING_WHEEL_H

#include <memory>
#include <string>

#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <math.h>
#include <sys/stat.h>
#include <unistd.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>

class SteeringWheel{
    public:
        SteeringWheel(ros::NodeHandle node, ros::NodeHandle private_nh);
        void Run();

    // Private Member Functions
    private:
        void GetParameter();

        void InitializeDevice();

        bool UpdateJoy(sensor_msgs::Joy& joy_msg);



    // Private Member Variables
    private:
        ros::NodeHandle nh_; 
        ros::NodeHandle pvt_nh_;
        ros::Publisher pub_;

        //parameters
        bool open_;
        bool sticky_buttons_;
        bool default_trig_val_;
        double deadzone_;
        double autorepeat_rate_;    // in Hz.  0 for no repeat.
        double coalesce_interval_;  // Defaults to 100 Hz rate limit.
        double lastDiagTime_;

        double autorepeat_interval;
        double scale;
        double unscaled_deadzone;

        //run time parameters
        bool tv_set;
        bool publication_pending = false;

        // device address
        std::string joy_dev_;
        std::string joy_dev_name_;
        std::string joy_dev_ff_;
        
        int joy_fd;
        int ff_fd_;

        struct ff_effect joy_effect_;
        js_event event;
        struct timeval tv;
        fd_set set;
        
       
        bool device_open_;
        int event_count_;
        int pub_count_;
        bool update_feedback_;
        
        

        
        



};

#endif