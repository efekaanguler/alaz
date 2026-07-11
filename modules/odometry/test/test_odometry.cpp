#include <gtest/gtest.h>
#include "odometry_node.hpp"
#include <cmath>

TEST(OdometryTest, InitialState) {
    rclcpp::Time t(0, 0);
    Odometry odom(t);
    EXPECT_DOUBLE_EQ(odom.x, 0.0);
    EXPECT_DOUBLE_EQ(odom.y, 0.0);
    EXPECT_DOUBLE_EQ(odom.th, 0.0);
}

TEST(OdometryTest, MoveForward) {
    rclcpp::Time t1(1, 0);
    Odometry odom(t1);

    rclcpp::Time t2(2, 0); // 1 second later
    double vx = 5.0; // 5 m/s
    double vth = 0.0;
    
    odom.update_odometry(vx, vth, t2);

    EXPECT_DOUBLE_EQ(odom.x, 5.0);
    EXPECT_DOUBLE_EQ(odom.y, 0.0);
    EXPECT_DOUBLE_EQ(odom.th, 0.0);
}

TEST(OdometryTest, MoveWithYaw) {
    rclcpp::Time t1(1, 0);
    Odometry odom(t1);
    
    // Set initial yaw to 90 degrees (pi/2)
    odom.th = M_PI / 2.0;

    rclcpp::Time t2(2, 0); // 1 second later
    double vx = 5.0; // 5 m/s
    double vth = 0.0;
    
    odom.update_odometry(vx, vth, t2);

    // Should move along Y axis because of 90 deg yaw
    EXPECT_NEAR(odom.x, 0.0, 1e-6);
    EXPECT_DOUBLE_EQ(odom.y, 5.0);
    EXPECT_DOUBLE_EQ(odom.th, M_PI / 2.0);
}

TEST(OdometryTest, RotationOnly) {
    rclcpp::Time t1(1, 0);
    Odometry odom(t1);

    rclcpp::Time t2(2, 0); // 1 second later
    double vx = 0.0;
    double vth = 0.5; // 0.5 rad/s
    
    odom.update_odometry(vx, vth, t2);

    EXPECT_DOUBLE_EQ(odom.x, 0.0);
    EXPECT_DOUBLE_EQ(odom.y, 0.0);
    EXPECT_DOUBLE_EQ(odom.th, 0.5);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
