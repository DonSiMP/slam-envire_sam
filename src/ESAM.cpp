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
    ESAM(pose, var_pose, pose_key, landmark_key, bfilter_default, outlier_default);
}

ESAM::ESAM(const ::base::Pose &pose, const ::base::Matrix6d &cov_pose,
        const char pose_key, const char landmark_key)
{
    BilateralFilterParams bfilter_default;
    OutlierRemovalParams outlier_default;
    ESAM(pose, cov_pose, pose_key, landmark_key, bfilter_default, outlier_default);
}

ESAM::ESAM(const ::base::TransformWithCovariance &pose_with_cov,
        const char pose_key, const char landmark_key,
        const BilateralFilterParams &bfilter,
        const OutlierRemovalParams &outliers)
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

    // Add a prior on pose x0. This indirectly specifies where the origin is.
    this->_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(this->pose_key, this->pose_idx), pose_0, cov_pose_0)); // add directly to graph

}
ESAM::ESAM(const ::base::Pose &pose, const ::base::Matrix6d &cov_pose,
        const char pose_key, const char landmark_key,
        const BilateralFilterParams &bfilter,
        const OutlierRemovalParams &outliers)
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

    // Add a prior on pose x0. This indirectly specifies where the origin is.
    this->_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(this->pose_key, this->pose_idx), pose_0, cov_pose_0)); // add directly to graph
}

ESAM::ESAM(const ::base::Pose &pose, const ::base::Vector6d &var_pose,
        const char pose_key, const char landmark_key,
        const BilateralFilterParams &bfilter,
        const OutlierRemovalParams &outliers)
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

void ESAM::mergePointClouds(PCLPointCloud &merged_point_cloud)
{
    merged_point_cloud.clear();
    for(register unsigned int i=0; i<this->pose_idx+1; ++i)
    {
        gtsam::Symbol frame_id(this->pose_key, i);
        frame_id.print();
        PCLPointCloud local_points = this->getPointCloud(frame_id);
        base::TransformWithCovariance tf_cov = this->getTransformPose(frame_id);
        this->transformPointCloud(local_points, tf_cov.getTransform());
        merged_point_cloud += local_points;
        std::cout<<"local_points.size(); "<<local_points.size()<<"\n";
    }
}

void ESAM::mergePointClouds(base::samples::Pointcloud &base_point_cloud)
{
    PCLPointCloud pcl_point_cloud;
    this->mergePointClouds(pcl_point_cloud);

    std::cout<<"merged_points.size(); "<<pcl_point_cloud.size()<<"\n";
    base_point_cloud.points.clear();
    base_point_cloud.colors.clear();
    envire::sam::fromPCLPointCloud<PointType>(base_point_cloud, pcl_point_cloud);
    std::cout<<"base merged point cloud.size(); "<<base_point_cloud.points.size()<<"\n";
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
    #endif

    /** Downsample **/
    const float voxel_grid_leaf_size = 0.01;
    PCLPointCloudPtr downsample_point_cloud (new PCLPointCloud);
    this->downsample (pcl_point_cloud, voxel_grid_leaf_size, downsample_point_cloud);

    #ifdef DEBUG_PRINTS
    std::cout<<"Downsample point cloud\n";
    std::cout<<"downsample_points.size(): "<<downsample_point_cloud->size()<<"\n";
    #endif

    /** Remove Outliers **/
    PCLPointCloudPtr outlier_point_cloud(new PCLPointCloud);
    if (outlier_paramaters.type == RADIUS)
    {
        this->radiusOutlierRemoval(downsample_point_cloud, outlier_paramaters.parameter_one,
                outlier_paramaters.parameter_two, outlier_point_cloud);
    }
    else if (outlier_paramaters.type == STATISTICAL)
    {
        this->statisticalOutlierRemoval(downsample_point_cloud, outlier_paramaters.parameter_one,
                outlier_paramaters.parameter_two, outlier_point_cloud);
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"Outlier point cloud\n";
    std::cout<<"outlier_points.size(): "<<outlier_point_cloud->size()<<"\n";
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

        /** Integrate fields **/
        point_cloud_item->getData() += *outlier_point_cloud;

        #ifdef DEBUG_PRINTS
        std::cout<<"Merging Point cloud with the existing one\n";
        std::cout<<"Number points: "<<point_cloud_item->getData().size()<<"\n";
        #endif

    }
    else
    {
        envire::sam::PointCloudItem::Ptr point_cloud_item(new PointCloudItem);
        point_cloud_item->setData(*outlier_point_cloud);
        this->_transform_graph.addItemToFrame(frame_id, point_cloud_item);

        #ifdef DEBUG_PRINTS
        std::cout<<"First time to push Point cloud\n";
        std::cout<<"Number points: "<<point_cloud_item->getData().size()<<"\n";
        #endif
    }

    #ifdef DEBUG_PRINTS
    std::cout<<"END!!\n";
    #endif

    return;
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

void ESAM::bilateralFilter(const PCLPointCloud::Ptr &points, const double &spatial_width, const double &range_sigma , PCLPointCloud::Ptr &filtered_out)
{
    pcl::FastBilateralFilter<PointType> b_filter;

    /** Configure Bilateral filter **/
    b_filter.setSigmaS(spatial_width);
    b_filter.setSigmaR(range_sigma);

    b_filter.setInputCloud(points);
    //filtered_out->width = points->width;
    //filtered_out->height = points->height;
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
    PCLPointCloud filtered_cloud;
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
