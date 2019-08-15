#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <service_server_pkg/MyCustomServiceMessage.h>



class HIBB8
{
    // private variables
    bool is_running;
    int which_state;
    int rate_hertz;
    float duration_;
    int times_;
    float radius_;
    int repetitions_;

    ros::NodeHandle nh_;
    ros::Rate *rate_;

    ros::ServiceServer srv_perform_square_;
    ros::Publisher pub_cmd_vel_;

public:

    HIBB8()
    {
        // Other variables
        is_running = false;
        which_state = 0;
        rate_hertz = 20;
        duration_ = 0;
        times_ = 0;
        radius_ = 0;
        repetitions_ = 0;

        rate_ = new ros::Rate(rate_hertz);

        srv_perform_square_ = nh_.advertiseService("/perform_square", &HIBB8::my_callback, this);

        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    ~HIBB8(void)
    {

    }

    void rateSleep(void)
    {
        rate_->sleep();
    }

    geometry_msgs::Twist getStateVelocity() {
        geometry_msgs::Twist vel;
        switch (which_state) {
            case 0: // go ahead
                vel.linear.x = 0.4;
                vel.angular.z = 0;
                break;
            case 1:  // stop
                vel.linear.x = 0;
                vel.angular.z = 0;
                break;
            case 2: // turn right
                vel.linear.x = 0;
                vel.angular.z = 0.4;
                break;
            case 3: // stop, but why again?
                vel.linear.x = 0;
                vel.angular.z = 0;
                break;
        }
        return vel;
    }

    void runTimeStateMachine(void)
    {
        geometry_msgs::Twist vel;

        if (!is_running)
        {
            vel.linear.x = 0;
            vel.angular.z = 0;
            pub_cmd_vel_.publish(vel);
            return;
        }

        vel = this->getStateVelocity();

        pub_cmd_vel_.publish(vel);

        duration_ -= 1/(float)rate_hertz;

        ROS_INFO("State [%d], Vel[%.2f, %.2f], Duration [%.2f]", which_state, vel.linear.x, vel.angular.z, duration_);

        if (duration_ <= 0) {
            float state_duration[4] = {2.0, 3.8, 4.0, 0.1};
            int next_state = which_state + 1;
            if (which_state == 3)
            {
                next_state = 0;
                times_ -= 1;
            }
            int next_state_duration = state_duration[next_state];
            this->changeState(next_state, next_state_duration);
        }

        if (times_ == 0) {
            is_running = false;
            vel.linear.x = 0;
            vel.angular.z = 0;
            pub_cmd_vel_.publish(vel);
        }
    }

    void changeState(int state, float duration)
    {
        which_state = state;
        duration_ = duration;
        ROS_INFO("Changing state to [%d]", which_state);
    }

    bool my_callback(service_server_pkg::MyCustomServiceMessage::Request &req,
                     service_server_pkg::MyCustomServiceMessage::Response &res)
    {
        is_running = ! is_running;
        radius_ = req.radius;
        repetitions_ = req.repetitions;
        times_ = 4 * repetitions_;
        return is_running;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MoveBB8Server");

    HIBB8 hiBB8;

    while (ros::ok())
    {
        hiBB8.runTimeStateMachine();

        hiBB8.rateSleep();

        ros::spinOnce();
    }

    ros::spin();

    return 0;
}