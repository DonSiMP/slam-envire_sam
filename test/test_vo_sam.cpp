#include <envire_sam/ESAM.hpp>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/StereoFactor.h>
#include <boost/test/unit_test.hpp>

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

using namespace envire::sam;

BOOST_AUTO_TEST_CASE(gtsam_vo_example)
{
    BOOST_TEST_MESSAGE( "\n**********************************************************\n" );
    BOOST_TEST_MESSAGE( "GTSAM_VO_EXAMPLE" );

    //create graph object, add first pose at origin with key '1'
    gtsam::NonlinearFactorGraph graph;
    gtsam::Pose3 first_pose;
    graph.push_back(gtsam::NonlinearEquality<gtsam::Pose3>(1, gtsam::Pose3()));

    //create factor noise model with 3 sigmas of value 1
    const gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(3,1);
    //create stereo camera calibration object with .2m between cameras
    const gtsam::Cal3_S2Stereo::shared_ptr K(new gtsam::Cal3_S2Stereo(1000, 1000, 0, 320, 240, 0.2));

    //create and add stereo factors between first pose (key value 1) and the three landmarks
    graph.push_back(gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3>(gtsam::StereoPoint2(520, 480, 440), model, 1, 3, K));
    graph.push_back(gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3>(gtsam::StereoPoint2(120, 80, 440), model, 1, 4, K));
    graph.push_back(gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3>(gtsam::StereoPoint2(320, 280, 140), model, 1, 5, K));

    //create and add stereo factors between second pose and the three landmarks
    graph.push_back(gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3>(gtsam::StereoPoint2(570, 520, 490), model, 2, 3, K));
    graph.push_back(gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3>(gtsam::StereoPoint2(70, 20, 490), model, 2, 4, K));
    graph.push_back(gtsam::GenericStereoFactor<gtsam::Pose3,gtsam::Point3>(gtsam::StereoPoint2(320, 270, 115), model, 2, 5, K));

    //create Values object to contain initial estimates of camera poses and landmark locations
    gtsam::Values initial_estimate;

    //create and add iniital estimates
    initial_estimate.insert(1, first_pose);
    initial_estimate.insert(2, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, -0.1, 1.1)));
    initial_estimate.insert(3, gtsam::Point3(1, 1, 5));
    initial_estimate.insert(4, gtsam::Point3(-1, 1, 5));
    initial_estimate.insert(5, gtsam::Point3(0, -0.5, 5));

    //create Levenberg-Marquardt optimizer for resulting factor graph, optimize
    gtsam::LevenbergMarquardtOptimizer optimizer = gtsam::LevenbergMarquardtOptimizer(graph, initial_estimate);
    gtsam::Values result = optimizer.optimize();

    result.print("Final result:\n");

  }

