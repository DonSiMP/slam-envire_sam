/**\file ESAM.cpp
 *
 * Wrapper class to deal with iSAM2, Factor Graphs and envire graph
 *
 * @author Javier Hidalgo Carrio et. al
 * See LICENSE for the license information
 *
 */

#include "ESAM.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

#define DEBUG_PRINTS 1

using namespace envire::sam;

ESAM::ESAM()
{
    base::Pose pose;
    base::Matrix6d cov_pose;
    ESAM(pose, cov_pose, 'x', 'l');
}

ESAM::ESAM(const ::base::Pose &pose, const ::base::Vector6d &var_pose,
        const char pose_key, const char landmark_key)
{
    BilateralFilterParams bfilter_default;
    OutlierRemovalParams outlier_default;
    SIFTKeypointParams keypoint_default;
    keypoint_default.min_scale = 0.06;
    keypoint_default.nr_octaves = 3;
    keypoint_default.nr_octaves_per_scale = 3;
    keypoint_default.min_contrast = 10.0;

    PFHFeatureParams feature_default;
    feature_default.normal_radius = 0.1;
    feature_default.feature_radius = 1.0;

    float default_downsample = 0.01;

    ESAM(pose, var_pose, pose_key, landmark_key, default_downsample,
            bfilter_default, outlier_default, keypoint_default, feature_default);
}

ESAM::ESAM(const ::base::Pose &pose, const ::base::Matrix6d &cov_pose,
        const char pose_key, const char landmark_key)
{
    BilateralFilterParams bfilter_default;
    OutlierRemovalParams outlier_default;
    SIFTKeypointParams keypoint_default;
    keypoint_default.min_scale = 0.06;
    keypoint_default.nr_octaves = 3;
    keypoint_default.nr_octaves_per_scale = 3;
    keypoint_default.min_contrast = 10.0;

    PFHFeatureParams feature_default;
    feature_default.normal_radius = 0.1;
    feature_default.feature_radius = 1.0;

    float default_downsample = 0.01;

    ESAM(pose, cov_pose, pose_key, landmark_key, default_downsample, bfilter_default,
            outlier_default, keypoint_default, feature_default);
}

ESAM::ESAM(const ::base::TransformWithCovariance &pose_with_cov,
        const char pose_key, const char landmark_key,
        const float downsample_size,
        const BilateralFilterParams &bfilter,
        const OutlierRemovalParams &outliers,
        const SIFTKeypointParams &keypoint,
        const PFHFeatureParams &feature)
{
    gtsam::Pose3 pose_0(gtsam::Rot3(pose_with_cov.orientation), gtsam::Point3(pose_with_cov.translation));
    gtsam::Matrix cov_matrix = pose_with_cov.cov;
    gtsam::noiseModel::Gaussian::shared_ptr cov_pose_0 = gtsam::noiseModel::Gaussian::Covariance(cov_matrix);

    /** Optimization parameters **/

    // Stop iterating once the change in error between steps is less than this value
    this->optimization_parameters.relativeErrorTol = 1e-5;

    // Do not perform more than N iteration steps
    this->optimization_parameters.maxIterations = 100;

    this->pose_key = pose_key;
    this->landmark_key = landmark_key;
    this->pose_idx = 0;
    this->landmark_idx = 0;

    /** Filter and outlier parameters **/
    this->bfilter_paramaters = bfilter;
    this->outlier_paramaters = outliers;
    this->keypoint_parameters = keypoint;

    /** Feature parameters **/
    this->feature_parameters = feature;

    /** Downsample size **/
    this->downsample_size = downsample_size;

    // Add a prior on pose x0. This indirectly specifies where the origin is.
    this->_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(this->pose_key, this->pose_idx), pose_0, cov_pose_0)); // add directly to graph

}
ESAM::ESAM(const ::base::Pose &pose, const ::base::Matrix6d &cov_pose,
        const char pose_key, const char landmark_key,
        const float downsample_size,
        const BilateralFilterParams &bfilter,
        const OutlierRemovalParams &outliers,
        const SIFTKeypointParams &keypoint,
        const PFHFeatureParams &feature)
{

    gtsam::Pose3 pose_0(gtsam::Rot3(pose.orientation), gtsam::Point3(pose.position));
    gtsam::Matrix cov_matrix = cov_pose;
    gtsam::noiseModel::Gaussian::shared_ptr cov_pose_0 = gtsam::noiseModel::Gaussian::Covariance(cov_matrix);

    /** Optimization parameters **/

    // Stop iterating once the change in error between steps is less than this value
    this->optimization_parameters.relativeErrorTol = 1e-5;

    // Do not perform more than N iteration steps
    this->optimization_parameters.maxIterations = 100;

    this->pose_key = pose_key;
    this->landmark_key = landmark_key;
    this->pose_idx = 0;
    this->landmark_idx = 0;

    /** Filter and outlier parameters **/
    this->bfilter_paramaters = bfilter;
    this->outlier_paramaters = outliers;
    this->keypoint_parameters = keypoint;

    /** Feature parameters **/
    this->feature_parameters = feature;

    /** Downsample size **/
    this->downsample_size = downsample_size;

    // Add a prior on pose x0. This indirectly specifies where the origin is.
    this->_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(this->pose_key, this->pose_idx), pose_0, cov_pose_0)); // add directly to graph
}

ESAM::ESAM(const ::base::Pose &pose, const ::base::Vector6d &var_pose,
        const char pose_key, const char landmark_key,
        const float downsample_size,
        const BilateralFilterParams &bfilter,
        const OutlierRemovalParams &outliers,
        const SIFTKeypointParams &keypoint,
        const PFHFeatureParams &feature)
{
    gtsam::Pose3 pose_0(gtsam::Rot3(pose.orientation), gtsam::Point3(pose.position));
    gtsam::Vector variances = var_pose;
    gtsam::noiseModel::Diagonal::shared_ptr cov_pose_0 = gtsam::noiseModel::Diagonal::Variances(variances);

    /** Optimzation parameters **/

    // Stop iterating once the change in error between steps is less than this value
    this->optimization_parameters.relativeErrorTol = 1e-5;
    // Do not perform more than N iteration steps
    this->optimization_parameters.maxIterations = 100;

    this->pose_key = pose_key;
    this->landmark_key = landmark_key;
    this->pose_idx = 0;
    this->landmark_idx = 0;

    /** Filter and outlier parameters **/
    this->bfilter_paramaters = bfilter;
    this->outlier_paramaters = outliers;
    this->keypoint_parameters = keypoint;

    /** Feature parameters **/
    this->feature_parameters = feature;

    /** Downsample size **/
    this->downsample_size = downsample_size;

    // Add a prior on pose x0. This indirectly specifies where the origin is.
    this->_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(this->pose_key, this->pose_idx), pose_0, cov_pose_0)); // add directly to graph
}

ESAM::~ESAM()
{
    this->marginals.reset();
}

void ESAM::insertFactor(const char key1, const unsigned long int &idx1,
                const char key2, const unsigned long int &idx2,
                const base::Time &time, const ::base::Pose &delta_pose,
                const ::base::Vector6d &var_delta_pose)
{
    /** Symbols **/
    gtsam::Symbol symbol1 = gtsam::Symbol(key1, idx1);
    gtsam::Symbol symbol2 = gtsam::Symbol(key2, idx2);

    /** Add the delta pose to the factor graph **/
    this->_factor_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(symbol1, symbol2,
                gtsam::Pose3(gtsam::Rot3(delta_pose.orientation), gtsam::Point3(delta_pose.position)),
                gtsam::noiseModel::Diagonal::Variances(var_delta_pose)));

    /** Add the delta pose to envire **/
    ::base::Matrix6d cov(base::Matrix6d::Identity());
    cov.diagonal() = var_delta_pose;
    envire::core::Transform tf(time, delta_pose.position, delta_pose.orientation, cov);
    this->_transform_graph.addTransform(symbol1, symbol2, tf);
}

void ESAM::insertFactor(const char key1, const unsigned long int &idx1,
                const char key2, const unsigned long int &idx2,
                const base::Time &time, const ::base::Pose &delta_pose,
                const ::base::Matrix6d &cov_delta_pose)
{

    /** Symbols **/
    gtsam::Symbol symbol1 = gtsam::Symbol(key1, idx1);
    gtsam::Symbol symbol2 = gtsam::Symbol(key2, idx2);

    /** Add the delta pose to the factor graph **/
    this->_factor_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(symbol1, symbol2,
                gtsam::Pose3(gtsam::Rot3(delta_pose.orientation), gtsam::Point3(delta_pose.position)),
                gtsam::noiseModel::Gaussian::Covariance(cov_delta_pose)));

    /** Add the delta pose to envire **/
    envire::core::Transform tf(time, delta_pose.position, delta_pose.orientation, cov_delta_pose);
    this->_transform_graph.addTransform(symbol1, symbol2, tf);

}

void ESAM::addDeltaPoseFactor(const base::Time &time, const ::Eigen::Affine3d &delta_tf, const ::base::Vector6d &var_delta_tf)
{
    ::base::Pose delta_pose(delta_tf);
    this->pose_idx++;
    return this->insertFactor(this->pose_key, this->pose_idx-1, this->pose_key, this->pose_idx, time, delta_pose, var_delta_tf);
}

void ESAM::addDeltaPoseFactor(const base::Time &time, const ::base::TransformWithCovariance &delta_pose_with_cov)
{
    ::base::Pose delta_pose(delta_pose_with_cov.translation, delta_pose_with_cov.orientation);
    this->pose_idx++;
    return this->insertFactor(this->pose_key, this->pose_idx-1, this->pose_key, this->pose_idx, time, delta_pose, delta_pose_with_cov.cov);
}

void ESAM::addDeltaPoseFactor(const base::Time &time, const ::base::Pose &delta_pose, const ::base::Vector6d &var_delta_pose)
{
    this->pose_idx++;
    return this->insertFactor(this->pose_key, this->pose_idx-1, this->pose_key, this->pose_idx, time, delta_pose, var_delta_pose);
}

void ESAM::addDeltaPoseFactor(const base::Time &time, const ::base::Pose &delta_pose, const ::base::Matrix6d &cov_delta_pose)
{
    this->pose_idx++;
    return this->insertFactor(this->pose_key, this->pose_idx-1, this->pose_key, this->pose_idx, time, delta_pose, cov_delta_pose);
}

void ESAM::insertValue(const std::string &frame_id, const ::base::TransformWithCovariance &pose_with_cov)
{
    try
    {
        envire::sam::PoseItem::Ptr pose_item(new envire::sam::PoseItem());
        pose_item->setData(pose_with_cov);
        this->_transform_graph.addItemToFrame(frame_id, pose_item);

    }catch(envire::core::UnknownFrameException &ufex)
    {
        std::cerr << ufex.what() << std::endl;
    }
}


void ESAM::insertValue(const char key, const unsigned long int &idx,
        const ::base::TransformWithCovariance &pose_with_cov)
{
    gtsam::Symbol symbol = gtsam::Symbol(key, idx);
    try
    {
        envire::sam::PoseItem::Ptr pose_item(new envire::sam::PoseItem());
        pose_item->setData(pose_with_cov);
        this->_transform_graph.addItemToFrame(symbol, pose_item);

    }catch(envire::core::UnknownFrameException &ufex)
    {
        std::cerr << ufex.what() << std::endl;
    }

}

void ESAM::insertValue(const char key, const unsigned long int &idx,
        const ::base::Pose &pose, const ::base::Matrix6d &cov_pose)
{
    gtsam::Symbol symbol = gtsam::Symbol(key, idx);
    try
    {
        envire::sam::PoseItem::Ptr pose_item(new envire::sam::PoseItem());
        base::TransformWithCovariance pose_with_cov(pose.position, pose.orientation, cov_pose);
        pose_item->setData(pose_with_cov);
        this->_transform_graph.addItemToFrame(symbol, pose_item);

    }catch(envire::core::UnknownFrameException &ufex)
    {
        std::cerr << ufex.what() << std::endl;
    }
}

void ESAM::addPoseValue(const ::base::TransformWithCovariance &pose_with_cov)
{
    /** Add the frame to the transform graph **/
    gtsam::Symbol frame_id = gtsam::Symbol(this->pose_key, this->pose_idx);
    this->_transform_graph.addFrame(frame_id);

    /** Insert the item **/
    return this->insertValue(this->pose_key, this->pose_idx, pose_with_cov);
}

base::TransformWithCovariance& ESAM::getLastPoseValueAndId(std::string &frame_id_string)
{
    gtsam::Symbol frame_id = gtsam::Symbol(this->pose_key, this->pose_idx);
    envire::sam::PoseItem::Ptr pose_item;
    try
    {
        std::vector<envire::core::ItemBase::Ptr> items = this->_transform_graph.getItems(frame_id);
        pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
        frame_id_string = static_cast<std::string>(frame_id);
    }catch(envire::core::UnknownFrameException &ufex)
    {
        std::cerr << ufex.what() << std::endl;
    }

    return pose_item->getData();
}

std::string ESAM::currentPoseId()
{
    return gtsam::Symbol(this->pose_key, this->pose_idx);
}

std::string ESAM::currentLandmarkId()
{
    return gtsam::Symbol(this->landmark_key, this->landmark_idx);
}

void ESAM::optimize()
{
    gtsam::Values initialEstimate;

    std::cout<<"GETTING THE ESTIMATES\n";

    /** Initial estimates for poses **/
    for(register unsigned int i=0; i<this->pose_idx+1; ++i)
    {
        gtsam::Symbol frame_id(this->pose_key, i);
        //frame_id.print();
        try
        {
            std::vector<envire::core::ItemBase::Ptr> items = this->_transform_graph.getItems(frame_id);
            envire::sam::PoseItem::Ptr pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
            gtsam::Pose3 pose(gtsam::Rot3(pose_item->getData().orientation), gtsam::Point3(pose_item->getData().translation));
            initialEstimate.insert(frame_id, pose);
        }catch(envire::core::UnknownFrameException &ufex)
        {
            std::cerr << ufex.what() << std::endl;
            return;
        }
    }

    std::cout<<"FINISHED GETTING ESTIMATES\n";

    initialEstimate.print("\nInitial Estimate:\n"); // print

    /** Create the optimizer ... **/
    gtsam::GaussNewtonOptimizer optimizer(this->_factor_graph, initialEstimate, this->optimization_parameters);

    /** Optimize **/
    gtsam::Values result = optimizer.optimize();
    result.print("Final Result:\n");

    std::cout<<"OPTIMIZE\n";

    /** Save the marginals **/
    this->marginals.reset(new gtsam::Marginals(this->_factor_graph, result));

    /** Store the result back in the transform graph **/
    gtsam::Values::iterator key_value = result.begin();
    for(; key_value != result.end(); ++key_value)
    {
        try
        {
            gtsam::Symbol const &frame_id(key_value->key);
            std::vector<envire::core::ItemBase::Ptr> items = this->_transform_graph.getItems(frame_id);
            envire::sam::PoseItem::Ptr pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
            base::TransformWithCovariance result_pose_with_cov;
            boost::shared_ptr<gtsam::Pose3> pose = boost::reinterpret_pointer_cast<gtsam::Pose3>(key_value->value.clone());
            result_pose_with_cov.translation = pose->translation().vector();
            result_pose_with_cov.orientation = pose->rotation().toQuaternion();
            result_pose_with_cov.cov = this->marginals->marginalCovariance(key_value->key);
            pose_item->setData(result_pose_with_cov);
        }catch(envire::core::UnknownFrameException &ufex)
        {
            std::cerr << ufex.what() << std::endl;
            return;
        }
    }
}

::base::TransformWithCovariance ESAM::getTransformPose(const std::string &frame_id)
{
    ::base::TransformWithCovariance tf_cov;
    try
    {
        std::vector<envire::core::ItemBase::Ptr> items = this->_transform_graph.getItems(frame_id);
        envire::sam::PoseItem::Ptr pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
        return pose_item->getData();
    }catch(envire::core::UnknownFrameException &ufex)
    {
        std::cerr << ufex.what() << std::endl;
    }

    return tf_cov;
}

::base::samples::RigidBodyState ESAM::getRbsPose(const std::string &frame_id)
{
    ::base::samples::RigidBodyState rbs_pose;
    try
    {
        ::base::TransformWithCovariance tf_pose;
        std::vector<envire::core::ItemBase::Ptr> items = this->_transform_graph.getItems(frame_id);
        envire::sam::PoseItem::Ptr pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
        tf_pose = pose_item->getData();
        rbs_pose.position = tf_pose.translation;
        rbs_pose.orientation = tf_pose.orientation;
        rbs_pose.cov_position = tf_pose.cov.block<3,3>(0,0);
        rbs_pose.cov_orientation = tf_pose.cov.block<3,3>(3,3);
    }catch(envire::core::UnknownFrameException &ufex)
    {
        std::cerr << ufex.what() << std::endl;
    }

    return rbs_pose;
}

std::vector< ::base::samples::RigidBodyState > ESAM::getRbsPoses()
{
    std::vector< ::base::samples::RigidBodyState > rbs_poses;

    for(register unsigned int i=0; i<this->pose_idx+1; ++i)
    {
        gtsam::Symbol frame_id(this->pose_key, i);
        rbs_poses.push_back(this->getRbsPose(frame_id));
    }

    return rbs_poses;
}

PCLPointCloud &ESAM::getPointCloud(const std::string &frame_id)
{
    try
    {
        std::vector<envire::core::ItemBase::Ptr> items = this->_transform_graph.getItems(frame_id);
        envire::sam::PointCloudItem::Ptr point_cloud_item = boost::static_pointer_cast<envire::sam::PointCloudItem>(items[1]);
        return point_cloud_item->getData();
    }catch(envire::core::UnknownFrameException &ufex)
    {
        std::cerr << ufex.what() << std::endl;
    }
}

void ESAM::mergePointClouds(PCLPointCloud &merged_point_cloud, bool downsample)
{
    merged_point_cloud.clear();
    for(register unsigned int i=0; i<this->pose_idx+1; ++i)
    {
        gtsam::Symbol frame_id(this->pose_key, i);
        //std::cout<<"MERGING POINT CLOUDS: ";
        //frame_id.print();
        size_t number_items = this->_transform_graph.getItems(frame_id).size();

        if (number_items > 1)
        {
            PCLPointCloud local_points = this->getPointCloud(frame_id);
            base::TransformWithCovariance tf_cov = this->getTransformPose(frame_id);
            this->transformPointCloud(local_points, tf_cov.getTransform());
            merged_point_cloud += local_points;
            //std::cout<<"local_points.size(); "<<local_points.size()<<"\n";
        }
    }

    /** Downsample **/
    if (downsample)
    {
        PCLPointCloudPtr merged_point_cloud_ptr = boost::make_shared<PCLPointCloud>(merged_point_cloud);
        PCLPointCloudPtr downsample_point_cloud (new PCLPointCloud);
        this->downsample (merged_point_cloud_ptr, this->downsample_size, downsample_point_cloud);

        merged_point_cloud = *downsample_point_cloud;
    }
}

void ESAM::mergePointClouds(base::samples::Pointcloud &base_point_cloud, bool downsample)
{
    PCLPointCloud pcl_point_cloud;
    this->mergePointClouds(pcl_point_cloud, downsample);

    //std::cout<<"merged_points.size(); "<<pcl_point_cloud.size()<<"\n";
    base_point_cloud.points.clear();
    base_point_cloud.colors.clear();
    envire::sam::fromPCLPointCloud<PointType>(base_point_cloud, pcl_point_cloud);
    //std::cout<<"base merged point cloud.size(); "<<base_point_cloud.points.size()<<"\n";
}

void ESAM::currentPointCloud(base::samples::Pointcloud &base_point_cloud, bool downsample)
{
    /** Get the current point cloud **/
    gtsam::Symbol frame_id = gtsam::Symbol(this->pose_key, this->pose_idx-1);

    /* Clear point cloud **/
    base_point_cloud.points.clear();
    base_point_cloud.colors.clear();

    /** Get the number of items **/
    size_t number_items = this->_transform_graph.getItems(frame_id).size();

    if (number_items > 1)
    {
        PCLPointCloud current_point_cloud = this->getPointCloud(frame_id);

        /** Downsample **/
        if (downsample)
        {
            PCLPointCloudPtr current_point_cloud_ptr = boost::make_shared<PCLPointCloud>(current_point_cloud);
            PCLPointCloudPtr downsample_point_cloud (new PCLPointCloud);
            this->downsample (current_point_cloud_ptr, this->downsample_size, downsample_point_cloud);
            current_point_cloud = *downsample_point_cloud;
        }

        /** Convert to base point cloud **/
        envire::sam::fromPCLPointCloud<PointType>(base_point_cloud, current_point_cloud);
    }
}

void ESAM::currentPointCloudtoPLY(const std::string &prefixname, bool downsample)
{
    base::samples::Pointcloud base_point_cloud;

    /** Get the current point cloud **/
    gtsam::Symbol frame_id = gtsam::Symbol(this->pose_key, this->pose_idx-1);

    /** Get the number of items **/
    size_t number_items = this->_transform_graph.getItems(frame_id).size();

    if (number_items > 1)
    {

        PCLPointCloud current_point_cloud = this->getPointCloud(frame_id);

        /** Downsample **/
        if (downsample)
        {
            PCLPointCloudPtr current_point_cloud_ptr = boost::make_shared<PCLPointCloud>(current_point_cloud);
            PCLPointCloudPtr downsample_point_cloud (new PCLPointCloud);
            this->downsample (current_point_cloud_ptr, this->downsample_size, downsample_point_cloud);
            current_point_cloud = *downsample_point_cloud;
        }

        /** Convert to base point cloud **/
        envire::sam::fromPCLPointCloud<PointType>(base_point_cloud, current_point_cloud);
    }

    /** Write to PLY **/
    std::string filename; filename = prefixname + static_cast<std::string>(frame_id) + ".ply";
    this->writePlyFile(base_point_cloud, filename);
}


void ESAM::printMarginals()
{
    std::cout.precision(3);
    for(register unsigned int i=0; i<this->pose_idx+1; ++i)
    {
        gtsam::Symbol frame_id(this->pose_key, i);
        std::cout <<this->pose_key<<i<<" covariance:\n" << this->marginals->marginalCovariance(frame_id) << std::endl;
    }
}


void ESAM::pushPointCloud(const ::base::samples::Pointcloud &base_point_cloud, const int height, const int width)
{
    #ifdef DEBUG_PRINTS
    std::cout<<"Transform point cloud\n";
    std::cout<<"Number points: "<<base_point_cloud.points.size()<<"\n";
    std::cout<<"Number colors: "<<base_point_cloud.colors.size()<<"\n";
    #endif

    /** Convert to pcl point cloud **/
    PCLPointCloudPtr pcl_point_cloud(new PCLPointCloud);
    envire::sam::toPCLPointCloud<PointType>(base_point_cloud, *pcl_point_cloud);
    pcl_point_cloud->height = height;
    pcl_point_cloud->width = width;

    #ifdef DEBUG_PRINTS
    std::cout<<"Convert point cloud\n";
    std::cout<<"pcl_point_cloud.size(): "<<pcl_point_cloud->size()<<"\n";
    std::cout<<"pcl_point_cloud.heigh: "<<pcl_point_cloud->height<<"\n";
    std::cout<<"pcl_point_cloud.width: "<<pcl_point_cloud->width<<"\n";
    #endif

    /** Bilateral filter **/
    PCLPointCloudPtr filter_point_cloud (new PCLPointCloud);
    this->bilateralFilter(pcl_point_cloud, bfilter_paramaters.spatial_width,
                        bfilter_paramaters.range_sigma, filter_point_cloud);
    #ifdef DEBUG_PRINTS
    std::cout<<"Filter point cloud\n";
    std::cout<<"filter_point_cloud.size(): "<<filter_point_cloud->size()<<"\n";
    std::cout<<"filter_point_cloud.heigh: "<<filter_point_cloud->height<<"\n";
    std::cout<<"filter_point_cloud.width: "<<filter_point_cloud->width<<"\n";
    #endif

    pcl_point_cloud.reset();

    /** Remove Outliers **/
    PCLPointCloudPtr radius_point_cloud(new PCLPointCloud);
    if (outlier_paramaters.type == RADIUS)
    {
        /** Radius need organized point clouds **/
        this->radiusOutlierRemoval(filter_point_cloud, outlier_paramaters.parameter_one,
                outlier_paramaters.parameter_two, radius_point_cloud);
    }
    else
    {
        radius_point_cloud = filter_point_cloud;
    }

    filter_point_cloud.reset();
    #ifdef DEBUG_PRINTS
    std::cout<<"Radius point cloud\n";
    std::cout<<"radius_point_cloud.size(): "<<radius_point_cloud->size()<<"\n";
    std::cout<<"radius_point_cloud.heigh: "<<radius_point_cloud->height<<"\n";
    std::cout<<"radius_point_cloud.width: "<<radius_point_cloud->width<<"\n";
    #endif

    /** Downsample, lost the organized point cloud **/
    PCLPointCloudPtr downsample_point_cloud (new PCLPointCloud);
    this->downsample (radius_point_cloud, this->downsample_size, downsample_point_cloud);

    radius_point_cloud.reset();

    #ifdef DEBUG_PRINTS
    std::cout<<"Downsample point cloud\n";
    std::cout<<"downsample_points.size(): "<<downsample_point_cloud->size()<<"\n";
    std::cout<<"Point width: " << downsample_point_cloud->width<<" Height : "<<downsample_point_cloud->height << std::endl;
    std::cout<<"Point cloud downsampled size: " << downsample_point_cloud->width * downsample_point_cloud->height << " data points." << std::endl;
    #endif

    /** Statistical outlier removal **/
    PCLPointCloudPtr statistical_point_cloud(new PCLPointCloud);
    if (outlier_paramaters.type == STATISTICAL)
    {
        this->statisticalOutlierRemoval(downsample_point_cloud, outlier_paramaters.parameter_one,
                outlier_paramaters.parameter_two, statistical_point_cloud);
    }
    else
    {
        statistical_point_cloud = downsample_point_cloud;
    }

    downsample_point_cloud.reset();

    #ifdef DEBUG_PRINTS
    std::cout<<"Statistical outlier point cloud\n";
    std::cout<<"statistical_points.size(): "<<statistical_point_cloud->size()<<"\n";
    #endif

    /** Remove point without color **/
    PCLPointCloudPtr final_point_cloud(new PCLPointCloud);
    this->removePointsWithoutColor (statistical_point_cloud, final_point_cloud);
    statistical_point_cloud.reset();

    #ifdef DEBUG_PRINTS
    std::cout<<"Final outlier point cloud\n";
    std::cout<<"final_points.size(): "<<final_point_cloud->size()<<"\n";
    #endif

    /** Get current point cloud in the node **/
    gtsam::Symbol frame_id = gtsam::Symbol(this->pose_key, this->pose_idx);
    std::vector<envire::core::ItemBase::Ptr> items = this->_transform_graph.getItems(frame_id);

    std::cout<<"FRAME ID: ";
    frame_id.print();
    std::cout<<" with "<<items.size()<<" items\n";

    /** Merge with the existing point cloud **/
    if (items.size() > 1)
    {
        /** Get the current point cloud **/
        envire::sam::PointCloudItem::Ptr point_cloud_item = boost::static_pointer_cast<envire::sam::PointCloudItem>(items[1]);

        /** Concatenate fields **/
        point_cloud_item->getData() += *final_point_cloud;

        /** Downsample the union **/
        PCLPointCloudPtr point_cloud_in_node = boost::make_shared<PCLPointCloud>(point_cloud_item->getData());
        PCLPointCloudPtr downsample_point_cloud (new PCLPointCloud);
        this->uniformsample(point_cloud_in_node, 2.0 * this->downsample_size, downsample_point_cloud);
        point_cloud_item->setData(*downsample_point_cloud.get());

        #ifdef DEBUG_PRINTS
        std::cout<<"Merging Point cloud with the existing one\n";
        std::cout<<"Number points: "<<point_cloud_item->getData().size()<<"\n";
        #endif

    }
    else
    {
        envire::sam::PointCloudItem::Ptr point_cloud_item(new PointCloudItem);
        point_cloud_item->setData(*final_point_cloud);
        this->_transform_graph.addItemToFrame(frame_id, point_cloud_item);

        #ifdef DEBUG_PRINTS
        std::cout<<"First time to push Point cloud\n";
        std::cout<<"Number points: "<<point_cloud_item->getData().size()<<"\n";
        #endif
    }

    final_point_cloud.reset();

    #ifdef DEBUG_PRINTS
    std::cout<<"END!!\n";
    #endif

    return;
}

void ESAM::keypointsPointCloud(const gtsam::Symbol &frame_id, const float normal_radius, const float feature_radius)
{
    /** Get the point cloud in the node **/
    std::vector<envire::core::ItemBase::Ptr> items = this->_transform_graph.getItems(frame_id);
    envire::sam::PointCloudItem::Ptr point_cloud_item = boost::static_pointer_cast<envire::sam::PointCloudItem>(items[1]);
    PCLPointCloudPtr point_cloud_ptr = boost::make_shared<PCLPointCloud>(point_cloud_item->getData());

    std::cout<<"FRAME ID: ";
    frame_id.print();
    std::cout<<" with "<<items.size()<<" items\n";

    /** Downsample **/
    PCLPointCloudPtr downsample_point_cloud (new PCLPointCloud);
    this->downsample (point_cloud_ptr, 5.0 * this->downsample_size, downsample_point_cloud);

    #ifdef DEBUG_PRINTS
    std::cout<<"DOWNSAMPLE SIZE: "<< 5.0 * this->downsample_size <<"\n";
    std::cout<<"NORMAL RADIUS: "<< normal_radius <<"\n";
    #endif

    /**  Compute surface normals **/
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    this->computeNormals (downsample_point_cloud, normal_radius, normals);

    /** Compute keypoints **/
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints (new pcl::PointCloud<pcl::PointWithScale>);
    this->detectKeypoints (point_cloud_ptr, keypoint_parameters.min_scale,
            keypoint_parameters.nr_octaves, keypoint_parameters.nr_octaves_per_scale,
            keypoint_parameters.min_contrast, keypoints);

    #ifdef DEBUG_PRINTS
    std::cout<<"DETECTED "<<keypoints->size()<<"KEYPOINTS\n";
    this->printKeypoints(keypoints);
    #endif

    /**  Compute PFH features **/
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors (new pcl::PointCloud<pcl::FPFHSignature33>);
    this->computeFPFHFeaturesAtKeypoints (downsample_point_cloud, normals, keypoints, feature_radius, descriptors);

    #ifdef DEBUG_PRINTS
    std::cout<<"DETECTED "<<descriptors->size()<<"FEATURE DESCRIPTORS\n";
    #endif

    /** Store the features descriptors in the envire node **/
    envire::sam::FPFHDescriptorItem::Ptr descriptors_item (new FPFHDescriptorItem);
    descriptors_item->setData(*descriptors);
    this->_transform_graph.addItemToFrame(frame_id, descriptors_item);

    return;
}

gtsam::Symbol ESAM::computeAlignedBoundingBox()
{
    /** Check that there is more than one frame **/
    if (this->pose_idx ==0)
        return this->invalid_symbol;

    /** Get the previous frame pose **/
    gtsam::Symbol prev_frame_id = gtsam::Symbol(this->pose_key, this->pose_idx-1);
    std::vector<envire::core::ItemBase::Ptr> prev_items = this->_transform_graph.getItems(prev_frame_id);
    envire::sam::PoseItem::Ptr prev_pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(prev_items[0]);
    boost::shared_ptr<base::TransformWithCovariance> prev_pose = boost::make_shared<base::TransformWithCovariance>(prev_pose_item->getData());

    /** Get the current frame pose **/
    gtsam::Symbol current_frame_id = gtsam::Symbol(this->pose_key, this->pose_idx);
    std::vector<envire::core::ItemBase::Ptr> current_items = this->_transform_graph.getItems(current_frame_id);
    envire::sam::PoseItem::Ptr current_pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(current_items[0]);
    boost::shared_ptr<base::TransformWithCovariance> current_pose = boost::make_shared<base::TransformWithCovariance>(current_pose_item->getData());

    /** Computer standard deviation **/
    Eigen::Vector3d std_prev_pose = prev_pose->cov.block<3,3>(0,0).diagonal().array().sqrt();
    Eigen::Vector3d std_current_pose = current_pose->cov.block<3,3>(0,0).diagonal().array().sqrt();
    std_prev_pose[2] = 1.5; std_current_pose[2] = 1.5;

    /** Compute Bounding box limits in the global frame **/
    Eigen::Vector3d front_limit(current_pose->translation);
    Eigen::Vector3d rear_limit(prev_pose->translation);

    /** Increase the limits using the standard deviation **/
    for (register int i=0; i<3; ++i)
    {
        (front_limit[i] > 0) ? (front_limit[i] + std_current_pose[i]) : (front_limit[i] - std_current_pose[i]);
        (rear_limit[i] > 0) ? (rear_limit[i] + std_prev_pose[i]) : (rear_limit[i] - std_prev_pose[i]);
    }

    /** Set the Bounding box **/
    envire::core::AlignedBoundingBox::Ptr bounding_box(new AlignedBoundingBox);
    bounding_box->extend(front_limit);
    bounding_box->extend(rear_limit);

    /** Assign the Bounding box to the item **/
    prev_pose_item->setBoundary(bounding_box);

    std::cout<<"FRAME ID: ";
    prev_frame_id.print();
    std::cout<<" with "<<prev_items.size()<<" items\n";
    std::cout<<"FRONT BOUNDING LIMITS:\n"<<front_limit<<"\n";
    std::cout<<"REAR BOUNDING LIMITS:\n"<<rear_limit<<"\n";
    std::cout<<"CENTER:\n"<<prev_pose_item->centerOfBoundary()<<"\n";

    return prev_frame_id;
}

void ESAM::detectLandmarks()
{
    /** Compute aligned bounding box from the previous to the current frame **/
    std::cout<<"COMPUTE BOUNDING BOX\n";
    gtsam::Symbol frame_id = this->computeAlignedBoundingBox();

    /** Get the number of items **/
    size_t number_items = this->_transform_graph.getItems(frame_id).size();

    /** Compute the keypoints in case of valid frame and it has point cloud **/
    if (frame_id != this->invalid_symbol && number_items > 1)
    {

        /** Compute the keypoints and features of the frame **/
        std::cout<<"KEYPOINTS AND FEATURES DESCRIPTORS\n";
        this->keypointsPointCloud(frame_id, this->feature_parameters.normal_radius, this->feature_parameters.feature_radius);

        /** Find next frame intersections **/
        std::cout<<"CONTAINER FRAME ID: "; frame_id.print();
        this->containsFrames(frame_id);

        /** Features Correspondences **/
        //this->featuresCorrespondences();

        /** Landmarks to Factor graph **/
    }

    return;
}

bool ESAM::intersects(const gtsam::Symbol &frame1, const gtsam::Symbol &frame2)
{
    /** Get Spatial item of the first frame **/
    std::vector<envire::core::ItemBase::Ptr> items1 = this->_transform_graph.getItems(frame1);
    envire::sam::PoseItem::Ptr pose_item1 = boost::static_pointer_cast<envire::sam::PoseItem>(items1[0]);

    /** Get Spatial item of the second frame **/
    std::vector<envire::core::ItemBase::Ptr> items2 = this->_transform_graph.getItems(frame2);
    envire::sam::PoseItem::Ptr pose_item2 = boost::static_pointer_cast<envire::sam::PoseItem>(items2[0]);

    /** Check intersection **/
    return pose_item1->intersects(*pose_item2);
}

bool ESAM::contains(const gtsam::Symbol &container_frame, const gtsam::Symbol &query_frame)
{
    /** Get Spatial item of the source frame **/
    std::vector<envire::core::ItemBase::Ptr> items1 = this->_transform_graph.getItems(container_frame);
    envire::sam::PoseItem::Ptr pose_item1 = boost::static_pointer_cast<envire::sam::PoseItem>(items1[0]);

    /** Get Spatial item of the query frame **/
    std::vector<envire::core::ItemBase::Ptr> items2 = this->_transform_graph.getItems(query_frame);
    envire::sam::PoseItem::Ptr pose_item2 = boost::static_pointer_cast<envire::sam::PoseItem>(items2[0]);

    /** Check intersection **/
    return (pose_item1->contains(pose_item2->getData().translation) ||
            pose_item1->contains(pose_item2->centerOfBoundary()));
}

void ESAM::containsFrames (const gtsam::Symbol &container_frame_id)
{
    for(register unsigned int i=0; i<this->pose_idx+1; ++i)
    {
        gtsam::Symbol target_frame_id = gtsam::Symbol(this->pose_key, i);
        if (target_frame_id != container_frame_id)
        {
            std::cout<<"TARGET FRAME ID: "; target_frame_id.print();

            if (this->contains(container_frame_id, target_frame_id))
            {
                std::cout<<"INTERSECTION FOUND!\n";
            }
            else
            {
                std::cout<<"NO FOUND!\n";
            }
        }
    }
}

void ESAM::printFactorGraph(const std::string &title)
{
    this->_factor_graph.print(title);
}

void ESAM::graphViz(const std::string &filename)
{
    envire::core::GraphViz viz;
    viz.write(this->_transform_graph, filename);
}

void ESAM::writePlyFile(const base::samples::Pointcloud& points, const std::string& file)
{
    std::ofstream data( file.c_str() );

    data << "ply" << "\n";
    data << "format ascii 1.0\n";

    data << "element vertex " << points.points.size() <<  "\n";
    data << "property float x\n";
    data << "property float y\n";
    data << "property float z\n";

    if( !points.colors.empty() )
    {
    data << "property uchar red\n";
    data << "property uchar green\n";
    data << "property uchar blue\n";
    data << "property uchar alpha\n";
    }
    data << "end_header\n";

    for( size_t i = 0; i < points.points.size(); i++ )
    {
    data 
        << points.points[i].x() << " "
        << points.points[i].y() << " "
        << points.points[i].z() << " ";
    if( !points.colors.empty() )
    {
        data 
        << (int)(points.colors[i].x()*255) << " "
        << (int)(points.colors[i].y()*255) << " "
        << (int)(points.colors[i].z()*255) << " "
        << (int)(points.colors[i].w()*255) << " ";
    }
    data << "\n";
    }
}


void ESAM::transformPointCloud(const ::base::samples::Pointcloud & pc, ::base::samples::Pointcloud & transformed_pc, const Eigen::Affine3d& transformation)
{
    std::cout<<"Static function transform point cloud\n";
    transformed_pc.points.clear();
    for(std::vector< ::base::Point >::const_iterator it = pc.points.begin(); it != pc.points.end(); it++)
    {
        transformed_pc.points.push_back(transformation * (*it));
    }
    transformed_pc.colors = pc.colors;
}

void ESAM::transformPointCloud(::base::samples::Pointcloud & pc, const Eigen::Affine3d& transformation)
{
    for(std::vector< ::base::Point >::iterator it = pc.points.begin(); it != pc.points.end(); it++)
    {
        *it = (transformation * (*it));
    }
}

void ESAM::transformPointCloud(pcl::PointCloud< PointType >&pcl_pc, const Eigen::Affine3d& transformation)
{
    for(std::vector< PointType, Eigen::aligned_allocator<PointType> >::iterator it = pcl_pc.begin();
            it != pcl_pc.end(); it++)
    {
        Eigen::Vector3d point (it->x, it->y, it->z);
        point = transformation * point;
        PointType pcl_point;
        pcl_point.x = point[0]; pcl_point.y = point[1]; pcl_point.z = point[2];
        pcl_point.rgb = it->rgb;
        *it = pcl_point;
    }
}

void ESAM::downsample (PCLPointCloud::Ptr &points, float leaf_size, PCLPointCloud::Ptr &downsampled_out)
{

  pcl::VoxelGrid<PointType> vox_grid;
  vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  vox_grid.setInputCloud (points);
  vox_grid.filter (*downsampled_out);

  return;
}

void ESAM::uniformsample (PCLPointCloud::Ptr &points, float radius_search, PCLPointCloud::Ptr &uniformsampled_out)
{
    pcl::PointCloud<int> sampled_indices;

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (points);
    uniform_sampling.setRadiusSearch (radius_search);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*points, sampled_indices.points, *uniformsampled_out);
    std::cout << "Original total points: " << points->size () << "; Uniform Sampling: " << uniformsampled_out->size () << std::endl;
}

void ESAM::removePointsWithoutColor (const PCLPointCloud::Ptr &points, PCLPointCloud::Ptr &points_out)
{
    points_out->clear();

    /* The output is an unorganized point cloud **/
    for(size_t i = 0; i < points->size(); ++i)
    {
        PointType const &pcl_point(points->points[i]);
        if (pcl_point.rgb > 0.00)
        {
            points_out->push_back(pcl_point);
        }
    }
    points_out->width = points_out->size();
    points_out->height = 1;
}

void ESAM::bilateralFilter(const PCLPointCloud::Ptr &points, const double &spatial_width, const double &range_sigma , PCLPointCloud::Ptr &filtered_out)
{
    pcl::FastBilateralFilter<PointType> b_filter;

    /** Configure Bilateral filter **/
    b_filter.setSigmaS(spatial_width);
    b_filter.setSigmaR(range_sigma);

    b_filter.setInputCloud(points);
    filtered_out->width = points->width;
    filtered_out->height = points->height;
    std::cout<<"width: "<<filtered_out->width<<"\n";
    std::cout<<"height: "<<filtered_out->height<<"\n";
    b_filter.filter(*filtered_out);
}

void ESAM::radiusOutlierRemoval(PCLPointCloud::Ptr &points, const double &radius, const double &min_neighbors, PCLPointCloud::Ptr &outliersampled_out)
{
    pcl::RadiusOutlierRemoval<PointType> ror;

    ror.setRadiusSearch(radius);
    ror.setMinNeighborsInRadius(min_neighbors);

    #ifdef DEBUG_PRINTS
    std::cout<<"RADIUS FILTER\n";
    std::cout<<"radius: "<< radius<<"\n";
    std::cout<<"min_neighbors: "<< min_neighbors<<"\n";
    #endif
    ror.setInputCloud(points);
    ror.filter (*outliersampled_out);
}

void ESAM::statisticalOutlierRemoval(PCLPointCloud::Ptr &points, const double &mean_k, const double &std_mul, PCLPointCloud::Ptr &outliersampled_out)
{
    pcl::StatisticalOutlierRemoval<PointType> sor;

    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(std_mul);

    #ifdef DEBUG_PRINTS
    std::cout<<"STATISTICAL FILTER\n";
    std::cout<<"mean_k: "<<mean_k<<"\n";
    std::cout<<"std_mul: "<<std_mul<<"\n";
    #endif
    sor.setInputCloud(points);
    sor.filter (*outliersampled_out);
}

void ESAM::computeNormals (PCLPointCloud::Ptr &points,
                                float normal_radius,
                                pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
  pcl::NormalEstimation<PointType, pcl::Normal> norm_est;

  // Use a FLANN-based KdTree to perform neighborhood searches
  //norm_est.setSearchMethod (pcl::KdTreeFLANN<PointType>::Ptr (new pcl::KdTreeFLANN<PointType>));
  norm_est.setSearchMethod (pcl::search::KdTree<PointType>::Ptr (new pcl::search::KdTree<PointType>));


  // Specify the size of the local neighborhood to use when computing the surface normals
  norm_est.setRadiusSearch (normal_radius);

  // Set the input points
  norm_est.setInputCloud (points);

  // Estimate the surface normals and store the result in "normals_out"
  norm_est.compute (*normals_out);
}

void ESAM::computePFHFeatures (PCLPointCloud::Ptr &points,
                      pcl::PointCloud<pcl::Normal>::Ptr &normals,
                      float feature_radius,
                      pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
{
    // Create a PFHEstimation object
    pcl::PFHEstimation<PointType, pcl::Normal, pcl::PFHSignature125> pfh_est;

    // Set it to use a FLANN-based KdTree to perform its neighborhood searches
    pfh_est.setSearchMethod (pcl::search::KdTree<PointType>::Ptr (new pcl::search::KdTree<PointType>));

    // Specify the radius of the PFH feature
    pfh_est.setRadiusSearch (feature_radius);

    // Set the input points and surface normals
    pfh_est.setInputCloud (points);
    pfh_est.setInputNormals (normals);

    // Compute the features
    pfh_est.compute (*descriptors_out);

    return;
}

void ESAM::detectKeypoints (PCLPointCloud::Ptr &points,
          float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast,
          pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_out)
{
    pcl::SIFTKeypoint<PointType, pcl::PointWithScale> sift_detect;

    // Use a FLANN-based KdTree to perform neighborhood searches
    sift_detect.setSearchMethod (pcl::search::KdTree<PointType>::Ptr (new pcl::search::KdTree<PointType>));

    // Set the detection parameters
    sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
    sift_detect.setMinimumContrast (min_contrast);

    // Set the input
    sift_detect.setInputCloud (points);

    // Detect the keypoints and store them in "keypoints_out"
    sift_detect.compute (*keypoints_out);

    return;
}

void ESAM::computePFHFeaturesAtKeypoints (PCLPointCloud::Ptr &points,
                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                           pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
                           pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
{
    // Create a PFHEstimation object
    pcl::PFHEstimation<PointType, pcl::Normal, pcl::PFHSignature125> pfh_est;

    // Set it to use a FLANN-based KdTree to perform its neighborhood searches
    pfh_est.setSearchMethod (pcl::search::KdTree<PointType>::Ptr (new pcl::search::KdTree<PointType>));

    // Specify the radius of the PFH feature
    pfh_est.setRadiusSearch (feature_radius);

    /* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
    * use them as an input to our PFH estimation, which expects clouds of PointXYZRGBA points.  To get around this,
    * we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to
    * "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGBA>).  Note that the original cloud doesn't have any RGB
    * values, so when we copy from PointWithScale to PointXYZRGBA, the new r,g,b fields will all be zero.
    */

    PCLPointCloud::Ptr keypoints_xyzrgb (new PCLPointCloud);
    pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);

    // Use all of the points for analyzing the local structure of the cloud
    pfh_est.setSearchSurface (points);
    pfh_est.setInputNormals (normals);

    // But only compute features at the keypoints
    pfh_est.setInputCloud (keypoints_xyzrgb);

    // Compute the features
    pfh_est.compute (*descriptors_out);

    return;
}

void ESAM::computeFPFHFeaturesAtKeypoints (PCLPointCloud::Ptr &points,
                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                           pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptors_out)
{
    // Create a FPFHEstimation object
    pcl::FPFHEstimation<PointType, pcl::Normal, pcl::FPFHSignature33> fpfh_est;

    // Set it to use a FLANN-based KdTree to perform its neighborhood searches
    fpfh_est.setSearchMethod (pcl::search::KdTree<PointType>::Ptr (new pcl::search::KdTree<PointType>));

    // Specify the radius of the PFH feature
    fpfh_est.setRadiusSearch (feature_radius);

    /* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
    * use them as an input to our PFH estimation, which expects clouds of PointXYZRGBA points.  To get around this,
    * we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to
    * "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGBA>).  Note that the original cloud doesn't have any RGB
    * values, so when we copy from PointWithScale to PointXYZRGBA, the new r,g,b fields will all be zero.
    */

    PCLPointCloud::Ptr keypoints_xyzrgb (new PCLPointCloud);
    pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);

    // Use all of the points for analyzing the local structure of the cloud
    fpfh_est.setSearchSurface (points);
    fpfh_est.setInputNormals (normals);

    // But only compute features at the keypoints
    fpfh_est.setInputCloud (keypoints_xyzrgb);

    // Compute the features
    fpfh_est.compute (*descriptors_out);

    return;
}

void ESAM::findFeatureCorrespondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                      pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                      std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{

    // Resize the output vector
    correspondences_out.resize (source_descriptors->size ());
    correspondence_scores_out.resize (source_descriptors->size ());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (target_descriptors);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);
    for (size_t i = 0; i < source_descriptors->size (); ++i)
    {
        descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
        correspondences_out[i] = k_indices[0];
        correspondence_scores_out[i] = k_squared_distances[0];
    }

    return;
}

void ESAM::printKeypoints(const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints)
{

    for (size_t i = 0; i < keypoints->size (); ++i)
    {
        /** Get the point data **/
        const pcl::PointWithScale & p = keypoints->points[i];

        std::cout<<"KEYPOINT: "<<p.x<<" "<<p.y<<" "<<p.z<<"\n";
    }

    return;
}
