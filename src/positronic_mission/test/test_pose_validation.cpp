// Pure gtest, no ROS. Just call validate_pose and check the result.

#include "mission_fixture.hpp"

using positronic_mission::GoToPose;
using namespace positronic_mission::test;

// ======================= Pose Check =======================

namespace {
    geometry_msgs::msg::PoseStamped valid_pose()
    {
        PoseStamped p;
        p.header.frame_id = positronic_mission::BASE_FRAME;
        p.pose.position.x = 0.15;
        p.pose.position.z = 0.20;
        p.pose.orientation.w = 1.0;
        return p;
    }
}

TEST(PoseValidation, FrameValidation)
{
    auto p = valid_pose();
    auto q = valid_pose();
    auto r = valid_pose();

    p.header.frame_id = "invalid";
    q.header.frame_id = "";

    EXPECT_EQ(positronic_mission::validate_pose(p), "invalid frame_id");
    EXPECT_EQ(positronic_mission::validate_pose(q), "invalid frame_id");
    EXPECT_EQ(positronic_mission::validate_pose(r), "");
}


TEST(PoseValidation, QuaternionValidation)
{
    auto p = valid_pose();
    auto q = valid_pose();
    auto r = valid_pose();
    auto s = valid_pose();
    auto t = valid_pose();

    p.pose.orientation.w = 0.0;
    q.pose.orientation.x = 0.7071;
    q.pose.orientation.w = 0.7071;
    r.pose.orientation.w = 1.002;
    s.pose.orientation.w = -1.0;
    t.pose.orientation.w = 1.0005;

    EXPECT_EQ(positronic_mission::validate_pose(p), "invalid quaternion");
    EXPECT_EQ(positronic_mission::validate_pose(q), "");
    EXPECT_EQ(positronic_mission::validate_pose(r), "invalid quaternion");
    EXPECT_EQ(positronic_mission::validate_pose(s), "");
    EXPECT_EQ(positronic_mission::validate_pose(t), "");
}

TEST(PoseValidation, NonFiniteValidation)
{
    auto p = valid_pose();
    auto q = valid_pose();
    auto r = valid_pose();

    p.pose.orientation.x = NAN;
    q.pose.position.x = NAN;
    r.pose.position.z = INFINITY;

    EXPECT_EQ(positronic_mission::validate_pose(p), "non numeric pose value");
    EXPECT_EQ(positronic_mission::validate_pose(q), "non numeric pose value");
    EXPECT_EQ(positronic_mission::validate_pose(r), "non numeric pose value");
}
