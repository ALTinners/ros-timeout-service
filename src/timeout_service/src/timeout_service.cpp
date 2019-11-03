#include "timeout_service/timeout_service.h"

#include <chrono>

#include <ros/topic.h>

void TimeoutServiceClient::handle_response(const ros::SerializedMessage& msg)
{
    // std::lock_guard<std::mutex> lock(sessions_lock_);
    // auto itr = std::find_if( sessions_.begin(), sessions_.end(), predicate );

    // if (itr != sessions_.end())
    // {
    //     ser_resp = itr->response;
    //     sessions_.erase( std::remove_if( sessions_.begin(), sessions_.end(), predicate ) );
    //     has_response = true;
    //     break;
    // }
}

void TimeoutServiceClient::shutdown()
{
    ROS_ASSERT_MSG(false, "This method is not yet implemented");
}

bool TimeoutServiceClient::waitForExistence(ros::Duration timeout)
{
    // // TODO - validate that the TopicManager instance can update asynchronously
    // auto now = ros::Time::now();
    // while(ros::Time::now() - now < timeout && !exists())
    // {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
    // return exists();

    ROS_ERROR("waitForExistence not implemented");
    assert(false);
    return false;
}

bool TimeoutServiceClient::exists()
{
    return true; //(publisher_.getNumSubscribers() > 0) && (subscriber_.getNumPublishers() > 0);
}

std::string TimeoutServiceClient::getService()
{
    return service_name_;
}