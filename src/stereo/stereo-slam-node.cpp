#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    if (doRectify){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "camera/right");

    pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    tf_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::publishTfFromPose(const geometry_msgs::msg::PoseStamped& msg)
{
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = msg.header.stamp;
    t.header.frame_id = "world";
    t.child_frame_id = "pose";

    t.transform.translation.x = msg.pose.position.x;
    t.transform.translation.y = msg.pose.position.y;
    t.transform.translation.z = msg.pose.position.z;

    t.transform.rotation.x = msg.pose.orientation.x;
    t.transform.rotation.y = msg.pose.orientation.y;
    t.transform.rotation.z = msg.pose.orientation.z;
    t.transform.rotation.w = msg.pose.orientation.w;

    // Send the transformation
    tf_broadcaster->sendTransform(t);
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f estimated_pose{};

    if (doRectify){
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        estimated_pose = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
    }
    else
    {
        estimated_pose = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
    }

    // Publish pose data
    auto inv = estimated_pose.inverse();

    // Eigen::Quaternionf quat = inv.unit_quaternion();
    // Eigen::Vector3f t = inv.translation();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msgLeft->header.stamp;
    pose_msg.header.frame_id = "/world";

    pose_msg.pose.position.x = inv.translation().x();
    pose_msg.pose.position.y = inv.translation().y();
    pose_msg.pose.position.z = inv.translation().z();

    pose_msg.pose.orientation.w = inv.unit_quaternion().w();
    pose_msg.pose.orientation.x = inv.unit_quaternion().x();
    pose_msg.pose.orientation.y = inv.unit_quaternion().y();
    pose_msg.pose.orientation.z = inv.unit_quaternion().z();

    pose_pub->publish(pose_msg);
    publishTfFromPose(pose_msg);
}
