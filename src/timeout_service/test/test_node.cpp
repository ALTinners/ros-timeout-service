#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_srvs/SetBool.h>

#include "timeout_service/timeout_service.h"
#include "timeout_service/test/test_headers.h"

namespace timeout_service
{

namespace testing
{


class TestNode
{
    public:
        TestNode(const ros::NodeHandle& nh)
            : nh_(nh)
        {
            signaller_pub_ = nh_.advertise<std_msgs::Time>(testing::signaller_topic_name, 5, true);
            signaller_timer_ = nh_.createTimer(
                ros::Duration(1),
                [&](const ros::TimerEvent&) {
                    std_msgs::Time time;
                    time.data  = ros::Time::now();
                    signaller_pub_.publish(time);
                }
            );

            reference_service_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
                testing::reference_service_name,
                boost::bind(&TestNode::handle_reference, this, _1, _2)
            );

            timeout_service_ = TimeoutServiceServer::advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
                nh_,
                testing::reference_service_name,
                boost::bind(&TestNode::handle_reference, this, _1, _2)
            );
        }

        bool handle_reference(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
        {
            std::this_thread::sleep_for(testing::sleep_time);
            resp.success = true;
            resp.message = "Great success!";
            return true;
        }

    private:
        ros::NodeHandle nh_;

        ros::Publisher signaller_pub_;
        ros::Timer signaller_timer_;
        ros::ServiceServer reference_service_;

        std::shared_ptr<TimeoutServiceServer> timeout_service_;
};

}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");

    ros::NodeHandle nh;
    timeout_service::testing::TestNode test_node(nh);

    ros::spin();

    return 0;
}