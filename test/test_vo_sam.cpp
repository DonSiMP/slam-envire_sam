#include <envire_sam/ESAM.hpp>
#include <boost/test/unit_test.hpp>

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

using namespace envire::sam;

BOOST_AUTO_TEST_CASE(envire_sam_vo_example)
{
    BOOST_TEST_MESSAGE( "\n**********************************************************\n" );
    BOOST_TEST_MESSAGE( "ENVIRE_SAM_SIMPLE_POSE_SLAM" );

    // 1. Create a factor graph container and add factors to it
    // 2a. Add a prior on the first pose, setting it to the origin
    // A prior factor consists of a mean and a noise model (covariance matrix)
    base::Pose pose_0;
    base::Vector6d var_pose_0;
    var_pose_0 << 0.3*0.3, 0.3*0.3, 0.3*0.3, 0.1*0.1, 0.1*0.1, 0.1*0.1;
    envire::sam::ESAM esam(pose_0, var_pose_0, 'x', 'l');

    // For simplicity, we will use the same noise model for odometry and loop closures
    base::Vector6d var_model;
    var_model << 0.2*0.2, 0.2*0.2, 0.2*0.2, 0.1*0.1, 0.1*0.1, 0.1*0.1;

    std::cout<<"CURRENT POSE ID: "<<esam.currentPoseId()<<"\n";
    std::cout<<"CURRENT LANDMARK ID: "<<esam.currentLandmarkId()<<"\n";

    // 2b. Add odometry factors
    // Create odometry (Between) factors between consecutive poses
    base::Pose delta_pose;
    delta_pose.position << 2.0, 0.0, 0.0;
    esam.addDeltaPoseFactor(base::Time::now(), delta_pose, var_model);
}

