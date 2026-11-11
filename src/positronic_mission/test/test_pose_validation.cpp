// Pure gtest, no ROS. Just call validate_pose and check the result.

#include <gtest/gtest.h>
#include "mission_fixture.hpp"
#include "positronic_mission/pose_validation.hpp"

using positronic_mission::State;
using positronic_mission::GoToPose;
using positronic_mission::MotionOutcome;
using rclcpp_action::ResultCode;
using namespace positronic_mission::test;

// ======================= Pose Check =======================

TEST_F(MissionNodeTest, PoseValidationCheck)
{
    auto p = valid_pose();
    auto q = valid_pose();
    p.header.frame_id = "invalid";
    q.header.frame_id = "";

    EXPECT_EQ(positronic_mission::validate_pose(p), "invalid frame_id");
    EXPECT_EQ(positronic_mission::validate_pose(q), "invalid frame_id");

    p = valid_pose();
    q = valid_pose();

    p.pose.orientation.w = 2.0;
    p.pose.orientation.x = 1.0;

    q.pose.orientation.x = -1.0;
    q.pose.orientation.z = -1.0;
    q.pose.orientation.y = -1.0;

    EXPECT_EQ(positronic_mission::validate_pose(p), "invalid quaternion");
    EXPECT_EQ(positronic_mission::validate_pose(q), "invalid quaternion");

    p = valid_pose();
    q = valid_pose();

    p.pose.position.x = NAN;
    q.pose.position.z = INFINITY;
    EXPECT_EQ(positronic_mission::validate_pose(p), "non numeric pose value");
    EXPECT_EQ(positronic_mission::validate_pose(q), "non numeric pose value");
}
