#include <steering_wheel.h>

::SteeringWheel(ros::NodeHandle node, ros::NodeHandle private_nh):
            nh_(node), pvt_nh_(private_nh)
{
    GetParameter();
}

void SteeringWheel::GetParameter()
{
    pvt_nh_.param<std::string>("dev", joy_dev_, "/dev/input/js0");
    pvt_nh_.param<std::string>("dev_ff", joy_dev_ff_, "/dev/input/event0");
    pvt_nh_.param<std::string>("dev_name", joy_dev_name_, "");
    pvt_nh_.param<double>("deadzone", deadzone_, 0.05);
    pvt_nh_.param<double>("autorepeat_rate", autorepeat_rate_, 0);
    pvt_nh_.param<double>("coalesce_interval", coalesce_interval_, 0.001);
    pvt_nh_.param<bool>("default_trig_val", default_trig_val_, false);
    pvt_nh_.param<bool>("sticky_buttons", sticky_buttons_, false);

    if (autorepeat_rate_ > 1 / coalesce_interval_)
    {
      ROS_WARN("joy_node: autorepeat_rate (%f Hz) > 1/coalesce_interval (%f Hz) "
        "does not make sense. Timing behavior is not well defined.", autorepeat_rate_, 1/coalesce_interval_);
    }

    if (deadzone_ >= 1)
    {
      ROS_WARN("joy_node: deadzone greater than 1 was requested. The semantics of deadzone have changed. "
        "It is now related to the range [-1:1] instead of [-32767:32767]. For now I am dividing your deadzone "
        "by 32767, but this behavior is deprecated so you need to update your launch file.");
      deadzone_ /= 32767;
    }

    if (deadzone_ > 0.9)
    {
      ROS_WARN("joy_node: deadzone (%f) greater than 0.9, setting it to 0.9", deadzone_);
      deadzone_ = 0.9;
    }

    if (deadzone_ < 0)
    {
      ROS_WARN("joy_node: deadzone_ (%f) less than 0, setting to 0.", deadzone_);
      deadzone_ = 0;
    }

    if (autorepeat_rate_ < 0)
    {
      ROS_WARN("joy_node: autorepeat_rate (%f) less than 0, setting to 0.", autorepeat_rate_);
      autorepeat_rate_ = 0;
    }

    if (coalesce_interval_ < 0)
    {
      ROS_WARN("joy_node: coalesce_interval (%f) less than 0, setting to 0.", coalesce_interval_);
      coalesce_interval_ = 0;
    }


    // Parameter conversions
    autorepeat_interval = 1 / autorepeat_rate_;
    scale = -1. / (1. - deadzone_) / 32767.;
    unscaled_deadzone = 32767. * deadzone_;

    event_count_ = 0;
    pub_count_ = 0;
    lastDiagTime_ = ros::Time::now().toSec();
    device_open_ = false;
}

void SteeringWheel::InitializeDevice()
{
    // first check if the joystick is available
    while (true)
    {
        ros::spinOnce();
        if (!nh_.ok())
        {
            return;
        }
        joy_fd = open(joy_dev_.c_str(), O_RDONLY);
        if (joy_fd != -1)
        {
            // There seems to be a bug in the driver or something where the
            // initial events that are to define the initial state of the
            // joystick are not the values of the joystick when it was opened
            // but rather the values of the joystick when it was last closed.
            // Opening then closing and opening again is a hack to get more
            // accurate initial state data.
            close(joy_fd);
            joy_fd = open(joy_dev_.c_str(), O_RDONLY);
        }
        if (joy_fd != -1)
        {
            break;
        }
        ROS_ERROR_ONCE("Couldn't open joystick %s. Will retry every second.", joy_dev_.c_str());
        sleep(1.0);
    }

    // then check if the force feedback device is available
    ff_fd_ = open(joy_dev_ff_.c_str(), O_RDWR);
    /* Set the gain of the device*/
    int gain = 100;           /* between 0 and 100 */
    struct input_event ie;      /* structure used to communicate with the driver */

    ie.type = EV_FF;
    ie.code = FF_GAIN;
    ie.value = 0xFFFFUL * gain / 100;

    if (write(ff_fd_, &ie, sizeof(ie)) == -1)
    {
        ROS_ERROR("Couldn't set gain on joystick force feedback: %s", strerror(errno));
        return;
    }

    // find the device name
    char current_joy_name[128];
    if (ioctl(joy_fd, JSIOCGNAME(sizeof(current_joy_name)), current_joy_name) < 0)
    {
        strncpy(current_joy_name, "Unknown", sizeof(current_joy_name));
    }
    ROS_INFO("Opened joystick: %s (%s). deadzone_: %f.", joy_dev_.c_str(), current_joy_name, deadzone_);
    device_open_ = true;   
}

void SteeringWheel::Run()
{
    bool tv_set = false;
    bool publication_pending = false;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    sensor_msgs::Joy joy_msg;  // Here because we want to reset it on device close.
    

    while(device_open_ && nh_.ok())
    {
        




        // finally ros spin
        ros::spinOnce();
    }
}

bool SteeringWheel::UpdateJoy(sensor_msgs::Joy& joy_msg)
{
    bool publish_now = false;
    bool publish_soon = false;
    FD_SET(joy_fd, &set);
    FD_ZERO(&set);
    double val;  // Temporary variable to hold event values

    int select_out = select(joy_fd+1, &set, nullptr, nullptr, &tv);
    if (select_out == -1)
    {
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        return true;
    }

    if (FD_ISSET(joy_fd, &set))
    {
        if (read(joy_fd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
        {
            return false;  // Joystick is probably closed. Definitely occurs.
        }

        joy_msg.header.stamp = ros::Time().now();
        event_count_++;
        switch (event.type)
        {
            case JS_EVENT_BUTTON:
            case JS_EVENT_BUTTON | JS_EVENT_INIT:
                if (event.number >= joy_msg.buttons.size())
                {
                    size_t old_size = joy_msg.buttons.size();
                    joy_msg.buttons.resize(event.number+1);
                    for (size_t i = old_size; i < joy_msg.buttons.size(); i++)
                    {
                    joy_msg.buttons[i] = 0.0;
                    }
                }
                if (sticky_buttons_)
                {
                    if (event.value == 1)
                    {
                    joy_msg.buttons[event.number] = 1 - joy_msg.buttons[event.number];
                    }
                }
                else
                {
                    joy_msg.buttons[event.number] = (event.value ? 1 : 0);
                }
                // For initial events, wait a bit before sending to try to catch
                // all the initial events.
                if (!(event.type & JS_EVENT_INIT))
                {
                    publish_now = true;
                }
                else
                {
                    publish_soon = true;
                }
                break;
            case JS_EVENT_AXIS:
            case JS_EVENT_AXIS | JS_EVENT_INIT:
                val = event.value;
                if (event.number >= joy_msg.axes.size()){
                    size_t old_size = joy_msg.axes.size();
                    joy_msg.axes.resize(event.number+1);
                    for (size_t i = old_size; i < joy_msg.axes.size(); i++)
                    {
                    joy_msg.axes[i] = 0.0;
                    }
                }
                if (default_trig_val_){
                    // Allows deadzone to be "smooth"
                    if (val > unscaled_deadzone)
                    {
                    val -= unscaled_deadzone;
                    }
                    else if (val < -unscaled_deadzone)
                    {
                    val += unscaled_deadzone;
                    }
                    else
                    {
                    val = 0;
                    }
                    joy_msg.axes[event.number] = val * scale;
                    // Will wait a bit before sending to try to combine events.
                    publish_soon = true;
                    break;
                }else{
                    if (!(event.type & JS_EVENT_INIT))
                    {
                    val = event.value;
                    if (val > unscaled_deadzone)
                    {
                        val -= unscaled_deadzone;
                    }
                    else if (val < -unscaled_deadzone)
                    {
                        val += unscaled_deadzone;
                    }
                    else
                    {
                        val = 0;
                    }
                    joy_msg.axes[event.number] = val * scale;
                    }

                    publish_soon = true;
                    break;
                }
            default:
                ROS_WARN("joy_node: Unknown event type. Please file a ticket. "
                "time=%u, value=%d, type=%Xh, number=%d", event.time, event.value, event.type, event.number);
                break;
        }
    }
    else if (tv_set)  // Assume that the timer has expired.
    {
        joy_msg.header.stamp = ros::Time().now();
        publish_now = true;
    }
    
}