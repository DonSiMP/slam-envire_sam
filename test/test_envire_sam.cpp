#include <boost/test/unit_test.hpp>
#include <envire_sam/ESAM.hpp>

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

using namespace envire::sam;

BOOST_AUTO_TEST_CASE(gtsam_simple_visual_slam)
{
    BOOST_TEST_MESSAGE( "\n**********************************************************\n" );
    BOOST_TEST_MESSAGE( "GTSAM_SIMPLE_VISUAL_SLAM" );

    // Define the camera calibration parameters
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

    // Define the camera observation noise model
    gtsam::noiseModel::Isotropic::shared_ptr measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

    // Create the set of ground-truth landmarks
    std::vector<gtsam::Point3> points;
    points.push_back(gtsam::Point3(10.0,10.0,10.0));
    points.push_back(gtsam::Point3(-10.0,10.0,10.0));
    points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
    points.push_back(gtsam::Point3(10.0,-10.0,10.0));
    points.push_back(gtsam::Point3(10.0,10.0,-10.0));
    points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
    points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
    points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

    // Create the set of ground-truth poses
    std::vector<gtsam::Pose3> poses;
    double radius = 30.0;
    int i = 0;
    double theta = 0.0;
    gtsam::Point3 up(0,0,1);
    gtsam::Point3 target(0,0,0);
    for(; i < 8; ++i, theta += 2*M_PI/8)
    {
        gtsam::Point3 position = gtsam::Point3(radius*cos(theta), radius*sin(theta), 0.0);
        gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up);
        poses.push_back(camera.pose());
    }

    // Create a factor graph
    gtsam::NonlinearFactorGraph graph;

    // Add a prior on pose x1. This indirectly specifies where the origin is.
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(6, 0.3, 0.3, 0.3, 0.1, 0.1, 0.1)); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', 0), poses[0], poseNoise)); // add directly to graph

    // Simulated measurements from each camera pose, adding them to the factor graph
    for (size_t i = 0; i < poses.size(); ++i)
    {
        for (size_t j = 0; j < points.size(); ++j)
        {
          gtsam::SimpleCamera camera(poses[i], *K);
          gtsam::Point2 measurement = camera.project(points[j]);
          graph.add(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(measurement, measurementNoise, gtsam::Symbol('x', i), gtsam::Symbol('l', j), K));
        }
    }

    // Because the structure-from-motion problem has a scale ambiguity, the problem is still under-constrained
    // Here we add a prior on the position of the first landmark. This fixes the scale by indicating the distance
    // between the first camera and the first landmark. All other landmark positions are interpreted using this scale.
    gtsam::noiseModel::Isotropic::shared_ptr pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    graph.add(gtsam::PriorFactor<gtsam::Point3>(gtsam::Symbol('l', 0), points[0], pointNoise)); // add directly to graph
    graph.print("Factor Graph:\n");

    // Create the data structure to hold the initialEstimate estimate to the solution
    // Intentionally initialize the variables off from the ground truth
    gtsam::Values initialEstimate;
    for (size_t i = 0; i < poses.size(); ++i)
        initialEstimate.insert(gtsam::Symbol('x', i), poses[i].compose(gtsam::Pose3(gtsam::Rot3::rodriguez(-0.1, 0.2, 0.25), gtsam::Point3(0.05, -0.10, 0.20))));
    for (size_t j = 0; j < points.size(); ++j)
        initialEstimate.insert(gtsam::Symbol('l', j), points[j].compose(gtsam::Point3(-0.25, 0.20, 0.15)));

    initialEstimate.print("Initial Estimates:\n");

    /* Optimize the graph and print results */
    gtsam::Values result = gtsam::DoglegOptimizer(graph, initialEstimate).optimize();
    result.print("Final results:\n");

}

BOOST_AUTO_TEST_CASE(envire_sam_simple_pose_slam)
{
    BOOST_TEST_MESSAGE( "\n**********************************************************\n" );
    BOOST_TEST_MESSAGE( "ENVIRE_SAM_SIMPLE_POSE_SLAM" );

    // 1. Create a factor graph container and add factors to it
    // 2a. Add a prior on the first pose, setting it to the origin
    // A prior factor consists of a mean and a noise model (covariance matrix)
    base::Pose pose_0;
    base::Vector6d var_pose_0;
    var_pose_0 << 0.3*0.3, 0.3*0.3, 0.3*0.3, 0.1*0.1, 0.1*0.1, 0.1*0.1;
    envire::sam::ESAM esam2(pose_0, var_pose_0, 'x', 'l');

    // For simplicity, we will use the same noise model for odometry and loop closures
    base::Vector6d var_model;
    var_model << 0.2*0.2, 0.2*0.2, 0.2*0.2, 0.1*0.1, 0.1*0.1, 0.1*0.1;

    // 2b. Add odometry factors
    // Create odometry (Between) factors between consecutive poses
    base::Pose delta_pose;
    delta_pose.position << 2.0, 0.0, 0.0;
    esam2.addDeltaPoseFactor(base::Time::now(), delta_pose, var_model);
//    delta_pose.orientation = Eigen::Quaternion <double> (Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())*
//                                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
//                                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
//    for (register int i=0; i<3; ++i)
//    {
//        esam.addDeltaPoseFactor(base::Time::now(), delta_pose, var_model);
//    }
//
//    // 2c. Add the loop closure constraint
//    // This factor encodes the fact that we have returned to the same pose. In real systems,
//    // these constraints may be identified in many ways, such as appearance-based techniques
//    // with camera images. We will use another Between Factor to enforce this constraint:
//    esam.insertPoseFactor('x', 4,'x', 1, base::Time::now(), delta_pose, var_model);
//    esam.printFactorGraph("\nFactor Graph:\n"); // print
//
//    // 3. Create the data structure to hold the initialEstimate estimate to the solution
//    // For illustrative purposes, these have been deliberately set to incorrect values
//    base::Pose pose;
//    pose.position << 0.5, 0.0, 0.0;
//    pose.orientation = Eigen::Quaternion <double> (Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
//    esam.insertPoseValue('x', 0, pose);
//    pose.position << 2.3, 0.1, 0.0;
//    pose.orientation = Eigen::Quaternion <double> (Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitZ()));
//    esam.insertPoseValue('x', 1, pose);
//    pose.position << 4.1, 0.1, 0.0;
//    pose.orientation = Eigen::Quaternion <double> (Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
//    esam.insertPoseValue('x', 2, pose);
//    pose.position << 4.0, 2.0, 0.0;
//    pose.orientation = Eigen::Quaternion <double> (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
//    esam.insertPoseValue('x', 3, pose);
//    pose.position << 2.1, 2.1, 0.0;
//    pose.orientation = Eigen::Quaternion <double> (Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()));
//    esam.insertPoseValue('x', 4, pose);
//
//    // GraphViz
//    esam.graphViz("esam_pose_slam_graph.dot");
//
//    // 4. Optimize
//    esam.optimize();
//
//    esam.printMarginals(); // print
//
//    std::string frame_id;
//    ::base::TransformWithCovariance last_pose = esam.getLastPoseValueAndId(frame_id);
//    std::cout<<frame_id<<" IS THE LAST POSE:\n"<<last_pose.translation<<"\n";
//    Eigen::Vector3d euler; /** In Euler angles **/
//    euler[2] = last_pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
//    euler[1] = last_pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
//    euler[0] = last_pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
//    std::cout<<frame_id<<" IS THE LAST POSE IN ESAM ROLL: "<<euler[0]*R2D<<" PITCH: "<<euler[1]*R2D<<" YAW: "<<euler[2]*R2D<<std::endl;
//    std::cout<<frame_id<<" IS THE LAST POSE IN ESAM COV:\n"<<last_pose.cov<<"\n";
//

//   // 5. Calculate and print marginal covariances for all variables
//   cout.precision(3);
//   Marginals marginals(graph, result);
//   cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
//   cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
//   cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
//   cout << "x4 covariance:\n" << marginals.marginalCovariance(4) << endl;
//   cout << "x5 covariance:\n" << marginals.marginalCovariance(5) << endl;

}

//BOOST_AUTO_TEST_CASE(envire_sam_simple_landmark_2dslam)
//{
//    BOOST_TEST_MESSAGE( "\n**********************************************************\n" );
//    BOOST_TEST_MESSAGE( "ENVIRE_SAM_SIMPLE_LANDMARK_2DSLAM" );
//
//    // Create a factor graph
//    base::Pose pose_0;
//    base::Vector6d var_pose_0;
//    var_pose_0 << 0.3*0.3, 0.3*0.3, 0.3*0.3, 0.1*0.1, 0.1*0.1, 0.1*0.1; // 30cm std on x,y, 0.1 rad on orientation
//    envire::sam::ESAM esam(pose_0, var_pose_0, 'x', 'l');
//
//    // Add two odometry factors
//    base::Pose delta_pose;
//    delta_pose.position << 2.0, 0.0, 0.0;
//    base::Vector6d var_model;
//    var_model << 0.2*0.2, 0.2*0.2, 0.2*0.2, 0.1*0.1, 0.1*0.1, 0.1*0.1; // 20cm std on x,y, 0.1 rad on orientation
//    esam.addDeltaPoseFactor(base::Time::now(), delta_pose, var_model);
//    esam.addDeltaPoseFactor(base::Time::now(), delta_pose, var_model);
//
    // Add Range-Bearing measurements to two different landmarks
//    // create a noise model for the landmark measurements
//    noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.2)); // 0.1 rad std on bearing, 20cm on range
//    // create the measurement values - indices are (pose id, landmark id)
//    Rot2 bearing11 = Rot2::fromDegrees(45),
//       bearing21 = Rot2::fromDegrees(90),
//       bearing32 = Rot2::fromDegrees(90);
//    double range11 = std::sqrt(4.0+4.0),
//         range21 = 2.0,
//         range32 = 2.0;
//
//    // Add Bearing-Range factors
//    graph.add(BearingRangeFactor<Pose2, Point2>(x1, l1, bearing11, range11, measurementNoise));
//    graph.add(BearingRangeFactor<Pose2, Point2>(x2, l1, bearing21, range21, measurementNoise));
//    graph.add(BearingRangeFactor<Pose2, Point2>(x3, l2, bearing32, range32, measurementNoise));
//
//    // Print
//    graph.print("Factor Graph:\n");
//
//    // Create (deliberately inaccurate) initial estimate
//    Values initialEstimate;
//    initialEstimate.insert(x1, Pose2(0.5, 0.0, 0.2));
//    initialEstimate.insert(x2, Pose2(2.3, 0.1,-0.2));
//    initialEstimate.insert(x3, Pose2(4.1, 0.1, 0.1));
//    initialEstimate.insert(l1, Point2(1.8, 2.1));
//    initialEstimate.insert(l2, Point2(4.1, 1.8));
//
//    // Print
//    initialEstimate.print("Initial Estimate:\n");

        // GraphViz
//        esam.graphViz("esam_landmark_2dslam_graph.dot");

//
//    // Optimize using Levenberg-Marquardt optimization. The optimizer
//    // accepts an optional set of configuration parameters, controlling
//    // things like convergence criteria, the type of linear system solver
//    // to use, and the amount of information displayed during optimization.
//    // Here we will use the default set of parameters.  See the
//    // documentation for the full set of parameters.
//    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
//    Values result = optimizer.optimize();
//    result.print("Final Result:\n");
//
//    // Calculate and print marginal covariances for all variables
//    Marginals marginals(graph, result);
//    print(marginals.marginalCovariance(x1), "x1 covariance");
//    print(marginals.marginalCovariance(x2), "x2 covariance");
//    print(marginals.marginalCovariance(x3), "x3 covariance");
//    print(marginals.marginalCovariance(l1), "l1 covariance");
//    print(marginals.marginalCovariance(l2), "l2 covariance");

//}


