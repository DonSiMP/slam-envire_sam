#include <boost/test/unit_test.hpp>
#include <envire_sam/ESAM.hpp>

using namespace envire_sam;

BOOST_AUTO_TEST_CASE(gtsam_simple_visual_slam)
{

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

BOOST_AUTO_TEST_CASE(envire_sam_simple_visual_slam)
{
    // Define the camera calibration parameters
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

    // Define the camera observation noise model
    gtsam::noiseModel::Isotropic::shared_ptr measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

    // Create the set of ground-truth landmarks
    std::vector<base::Position> points;
    points.push_back(base::Position(10.0,10.0,10.0));
    points.push_back(base::Position(-10.0,10.0,10.0));
    points.push_back(base::Position(-10.0,-10.0,10.0));
    points.push_back(base::Position(10.0,-10.0,10.0));
    points.push_back(base::Position(10.0,10.0,-10.0));
    points.push_back(base::Position(-10.0,10.0,-10.0));
    points.push_back(base::Position(-10.0,-10.0,-10.0));
    points.push_back(base::Position(10.0,-10.0,-10.0));

    // Create the set of ground-truth poses
    std::vector<base::Pose> poses;
    double radius = 30.0;
    int i = 0;
    double theta = 0.0;
    gtsam::Point3 up(0,0,1);
    gtsam::Point3 target(0,0,0);
    for(; i < 8; ++i, theta += 2*M_PI/8)
    {
        gtsam::Point3 position = gtsam::Point3(radius*cos(theta), radius*sin(theta), 0.0);
        gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up);
        base::Pose camera_pose(base::Position(camera.pose().translation().x(), camera.pose().translation().y(), camera.pose().translation().z()),
                base::Orientation(camera.pose().rotation().toQuaternion()));
        poses.push_back(camera_pose);
    }

    // Create a factor graph
    base::Matrix6d cov_pose_0;
    cov_pose_0.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * 0.3; // 30cm std on x,y,z
    cov_pose_0.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * 0.1; // 0.1 rad on roll,pitch,yaw
    envire_sam::ESAM esam(poses[0], cov_pose_0, 'x');

    // Simulated measurements from each camera pose, adding them to the envire sam
    for (size_t i = 0; i < poses.size(); ++i)
    {
        for (size_t j = 0; j < points.size(); ++j)
        {
          gtsam::SimpleCamera camera(gtsam::Pose3(gtsam::Rot3(poses[i].orientation), gtsam::Point3(poses[i].position)), *K);
          gtsam::Point2 measurement = camera.project(gtsam::Point3(points[j]));
          esam.factor_graph().add(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(measurement, measurementNoise, gtsam::Symbol('x', i), gtsam::Symbol('l', j), K));
        }
    }

    // Because the structure-from-motion problem has a scale ambiguity, the problem is still under-constrained
    // Here we add a prior on the position of the first landmark. This fixes the scale by indicating the distance
    // between the first camera and the first landmark. All other landmark positions are interpreted using this scale.
    gtsam::noiseModel::Isotropic::shared_ptr pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    esam.factor_graph().add(gtsam::PriorFactor<gtsam::Point3>(gtsam::Symbol('l', 0), gtsam::Point3(points[0]), pointNoise)); // add directly to graph
    esam.factor_graph().print("Factor Graph:\n");

    // Create the data structure to hold the initialEstimate estimate to the solution
    // Intentionally initialize the variables off from the ground truth
    gtsam::Values initialEstimate;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        initialEstimate.insert(gtsam::Symbol('x', i), gtsam::Pose3(gtsam::Rot3(poses[i].orientation), gtsam::Point3(poses[i].position)).
                compose(gtsam::Pose3(gtsam::Rot3::rodriguez(-0.1, 0.2, 0.25), gtsam::Point3(0.05, -0.10, 0.20))));
    }

    for (size_t j = 0; j < points.size(); ++j)
    {
        initialEstimate.insert(gtsam::Symbol('l', j), gtsam::Point3(points[j]).compose(gtsam::Point3(-0.25, 0.20, 0.15)));
    }

    initialEstimate.print("Initial Estimates:\n");

    /* Optimize the graph and print results */
    gtsam::Values result = gtsam::DoglegOptimizer(esam.factor_graph(), initialEstimate).optimize();
    result.print("Final results:\n");

}
