#include <exception>
#include <string>
#include <vector>
#include <utility>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Int32MultiArray.h>


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadControlClient;
typedef boost::shared_ptr<HeadControlClient> HeadControlClientPtr;

constexpr int NUM_MARKERS = 3;
constexpr int CENTER_X_MIN = 270;
constexpr int CENTER_X_MAX = 370;
constexpr int CENTER_Y_MIN = 0;
constexpr int CENTER_Y_MAX = 480;

std::vector<std::pair<int, int>> marker_pixels(NUM_MARKERS, {-1, -1});
std::vector<bool> marker_valid(NUM_MARKERS, false);

void markerCallback0(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 2) {
        marker_pixels[0] = {msg->data[0], msg->data[1]};
        marker_valid[0] = true;
    }
}
void markerCallback1(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 2) {
        marker_pixels[1] = {msg->data[0], msg->data[1]};
        marker_valid[1] = true;
    }
}
void markerCallback2(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 2) {
        marker_pixels[2] = {msg->data[0], msg->data[1]};
        marker_valid[2] = true;
    }
}

bool allMarkersVisible() {
    for (int i = 0; i < NUM_MARKERS; ++i) {
        if (!marker_valid[i])
            return false;
    }
    return true;
}
bool isMarker1Centered() {
    int x = marker_pixels[1].first;
    int y = marker_pixels[1].second;
    return (x >= CENTER_X_MIN && x <= CENTER_X_MAX &&
            y >= CENTER_Y_MIN && y <= CENTER_Y_MAX);
}
void createHeadClient(HeadControlClientPtr& actionClient) {
    ROS_INFO("Creating action client to head controller...");
    actionClient.reset(new HeadControlClient("/head_controller/follow_joint_trajectory"));

    int attempts = 0, max_attempts = 5;
    while (!actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && attempts < max_attempts) {
        ROS_WARN("Waiting for the head_controller action server to come up");
        ++attempts;
    }

    if (attempts == max_attempts)
        throw std::runtime_error("Head controller action server not available");
}

void sendHeadGoal(HeadControlClientPtr& client, double yaw, double pitch) {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = {yaw, pitch};
    point.velocities = {0.3, 0.3};
    point.time_from_start = ros::Duration(1.0);
    goal.trajectory.points.push_back(point);
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);

    client->sendGoal(goal);
    client->waitForResult(ros::Duration(3.0));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "run_head_traj_control");
    ros::NodeHandle nh;

    ROS_INFO("Starting 2D head search for red markers...");

    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) {
        ROS_FATAL("Timed out waiting for valid time.");
        return EXIT_FAILURE;
    }

    // Marker subscribers
    ros::Subscriber sub0 = nh.subscribe("/marker_pixel_position_0", 1, markerCallback0);
    ros::Subscriber sub1 = nh.subscribe("/marker_pixel_position_1", 1, markerCallback1);
    ros::Subscriber sub2 = nh.subscribe("/marker_pixel_position_2", 1, markerCallback2);

    // Create head action client
    HeadControlClientPtr headClient;
    createHeadClient(headClient);

    // Generate yaw/pitch combinations with finer granularity
    std::vector<double> yaw_angles;
    std::vector<double> pitch_angles;
    for (double y = -0.5; y <= 0.5; y += 0.1) yaw_angles.push_back(y);
    for (double p = -0.8; p <= -0.2; p += 0.1) pitch_angles.push_back(p);

    std::vector<std::pair<double, double>> head_positions;
    for (double pitch : pitch_angles) {
        for (double yaw : yaw_angles) {
            head_positions.emplace_back(yaw, pitch);
        }
    }

    ros::Rate rate(1.0);  // 1 Hz
    size_t index = 0;
    int stable_count = 0;
    const int stable_frame_threshold = 3;

    while (ros::ok()) {
        double yaw = head_positions[index].first;
        double pitch = head_positions[index].second;
        ROS_INFO("Moving head to yaw = %.2f, pitch = %.2f", yaw, pitch);

        sendHeadGoal(headClient, yaw, pitch);
        ros::Duration(1.5).sleep();  // allow vision stabilization

        // Reset marker validity before checking again
        std::fill(marker_valid.begin(), marker_valid.end(), false);

        ros::spinOnce();
        ros::Duration(0.5).sleep();  // give some time for callbacks
        ros::spinOnce();

        if (allMarkersVisible()) {
            stable_count++;
            ROS_INFO("All markers visible for %d consecutive steps", stable_count);
            if (stable_count >= stable_frame_threshold) {
                if (isMarker1Centered()) {
                    ROS_INFO("Marker #1 is centered. Stopping head scan.");
                    break;
                } else {
                    ROS_INFO("Markers are stable, but Marker #1 not centered yet.");
                }
            }
        } else {
            stable_count = 0;
        }
        

        index = (index + 1) % head_positions.size();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
