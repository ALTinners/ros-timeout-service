#include <thread>

#include <ros/init.h>
#include <ros/topic.h>
#include <ros/node_handle.h>
#include <gtest/gtest.h>

#include <std_msgs/Time.h>
#include <std_srvs/SetBool.h>

#include "timeout_service/timeout_service.h"
#include "timeout_service/test/test_headers.h"


TEST(TimeoutService, ValidateExpectedBehaviour)
{
    // A service client invoked on a thread will block that thread
    // However if that thread is not the same thread as that used for the

    using namespace timeout_service;
    using namespace timeout_service::testing;

    ros::NodeHandle nh;

    auto client = nh.serviceClient<std_srvs::SetBool>(reference_service_name);
    auto subscriber = nh.subscribe<std_msgs::Time>(
        signaller_topic_name,
        0,
        [&](const std_msgs::Time::ConstPtr&) {
            FAIL() << "Subscriber was called";
        }
    );

    std_srvs::SetBool call;
    EXPECT_TRUE(client.call(call)) << "TestNode did not respond correctly";

    bool thread_returned = false;
    bool main_returned = false;

    using namespace std::chrono;
    auto now = system_clock::now();
    std::chrono::milliseconds drift(100);

    std::thread worker([&] {
        std_srvs::SetBool thread_call;
        thread_returned = client.call(thread_call);
        EXPECT_TRUE(thread_returned) << "TestNode did not respond correctly";
        std::chrono::milliseconds difference;
        if (main_returned)
        {
            difference = std::chrono::duration_cast<std::chrono::milliseconds>((system_clock::now() - now)) - (sleep_time * 2);
        }
        else
        {
            difference = std::chrono::duration_cast<std::chrono::milliseconds>((system_clock::now() - now)) - sleep_time;
        }
        EXPECT_LT( std::abs(difference.count()),  drift.count()) << "Not close to expected error in thread context";
    });

    main_returned = client.call(call);
    EXPECT_TRUE(main_returned) << "TestNode did not respond correctly";
    std::chrono::milliseconds difference;
    if (thread_returned)
    {
        difference = std::chrono::duration_cast<std::chrono::milliseconds>((system_clock::now() - now)) - (sleep_time * 2);
    }
    else
    {
        difference = std::chrono::duration_cast<std::chrono::milliseconds>((system_clock::now() - now)) - sleep_time;
    }
    EXPECT_LT( std::abs(difference.count()),  drift.count()) << "Not close to expected error in main context";

    worker.join();

    auto msg = ros::topic::waitForMessage<std_msgs::Time>(signaller_topic_name, ros::Duration(1.0));
    EXPECT_TRUE(msg != nullptr);

    bool got_messages = false;
    subscriber = nh.subscribe<std_msgs::Time>(
        signaller_topic_name,
        0,
        [&](const std_msgs::Time::ConstPtr&) {
            got_messages = true;
        }
    );

    std::atomic_bool thread_done(false);
    worker = std::thread([&] {
        std_srvs::SetBool thread_call;
        client.call(thread_call);
        thread_done = true;
    });

    while(thread_done.load() == false)
    {
        ros::spinOnce();
    }

    worker.join();
    EXPECT_TRUE(got_messages) << "Did not get any messages on main thread while srv was in progress";

}

TEST(TimeoutService, ValidateBehaviour)
{
    // A service client invoked on a thread will block that thread
    // However if that thread is not the same thread as that used for the

    using namespace timeout_service;
    using namespace timeout_service::testing;

    ros::NodeHandle nh;

    auto client = TimeoutServiceClient::serviceClient<std_srvs::SetBool>(nh, reference_service_name);
    auto subscriber = nh.subscribe<std_msgs::Time>(
        signaller_topic_name,
        0,
        [&](const std_msgs::Time::ConstPtr&) {
            FAIL() << "Subscriber was called";
        }
    );

    std_srvs::SetBool call;
    EXPECT_TRUE(client.call(call, ros::Duration(20.0))) << "TestNode did not respond correctly";

    bool thread_returned = false;
    bool main_returned = false;

    using namespace std::chrono;
    auto now = system_clock::now();
    std::chrono::milliseconds drift(100);

    std::thread worker([&] {
        // std_srvs::SetBool thread_call;
        // thread_returned = client.call(thread_call);
        // EXPECT_TRUE(thread_returned) << "TestNode did not respond correctly";
        // std::chrono::milliseconds difference;
        // if (main_returned)
        // {
        //     difference = std::chrono::duration_cast<std::chrono::milliseconds>((system_clock::now() - now)) - (sleep_time * 2);
        // }
        // else
        // {
        //     difference = std::chrono::duration_cast<std::chrono::milliseconds>((system_clock::now() - now)) - sleep_time;
        // }
        // EXPECT_LT( std::abs(difference.count()),  drift.count()) << "Not close to expected error in thread context";
    });

    main_returned = client.call(call, ros::Duration(3.0));
    EXPECT_TRUE(main_returned) << "TestNode did not respond correctly";
    std::chrono::milliseconds difference;
    if (thread_returned)
    {
        difference = std::chrono::duration_cast<std::chrono::milliseconds>((system_clock::now() - now)) - (sleep_time * 2);
    }
    else
    {
        difference = std::chrono::duration_cast<std::chrono::milliseconds>((system_clock::now() - now)) - sleep_time;
    }
    EXPECT_LT( std::abs(difference.count()),  drift.count()) << "Not close to expected error in main context";

    worker.join();

    auto msg = ros::topic::waitForMessage<std_msgs::Time>(signaller_topic_name, ros::Duration(1.0));
    EXPECT_TRUE(msg != nullptr);

    bool got_messages = false;
    subscriber = nh.subscribe<std_msgs::Time>(
        signaller_topic_name,
        0,
        [&](const std_msgs::Time::ConstPtr&) {
            got_messages = true;
        }
    );

    std::atomic_bool thread_done(false);
    worker = std::thread([&] {
        std_srvs::SetBool thread_call;
        client.call(thread_call);
        thread_done = true;
    });

    while(thread_done.load() == false)
    {
        ros::spinOnce();
    }

    worker.join();
    EXPECT_TRUE(got_messages) << "Did not get any messages on main thread while srv was in progress";
}


int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "timeout_service_test");
    return RUN_ALL_TESTS();
}