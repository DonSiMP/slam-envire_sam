/**\file ESAM.hpp
 *
 * Wrapper class to deal with iSAM2, Factor Graphs and envire graph
 *
 * @author Javier Hidalgo Carrio et. al
 * See LICENSE for the license information
 *
 */


#ifndef __ENVIRE_SAM_ESAM__
#define __ENVIRE_SAM_ESAM__

#include <vector>
#include <fstream>

/** Rock Base Types **/
#include <base/Eigen.hpp>
#include <base/Pose.hpp>
#include <base/TransformWithCovariance.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Pointcloud.hpp>

/** Eigen **/
#include <Eigen/Eigenvalues>

/** Envire **/
#include <envire_core/all>

/** GTSAM TYPES **/
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/Symbol.h>

/** GTSAM Factors **/
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

/** GTSAM Optimizer **/
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

/** GTSAM Marhinals **/
#include <gtsam/nonlinear/Marginals.h>

/** GTSAM Values to estimate **/
#include <gtsam/nonlinear/Values.h>

/** PCL **/
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/keypoints/sift_keypoint.h>

/** Envire SAM **/
#include <envire_sam/Configuration.hpp>
#include <envire_sam/Conversions.hpp>

namespace envire { namespace sam
{
    /** PCL TYPES **/
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud<PointType> PCLPointCloud;
    typedef PCLPointCloud::Ptr PCLPointCloudPtr;

    /** Transform Graph types **/
    typedef envire::core::SpatialItem<base::TransformWithCovariance> PoseItem;
    typedef envire::core::Item<PCLPointCloud> PointCloudItem;
    typedef envire::core::Item< pcl::PointCloud<pcl::PFHSignature125> > PFHDescriptorItem;
    typedef envire::core::Item< pcl::PointCloud<pcl::FPFHSignature33> > FPFHDescriptorItem;

    /**
     * A class to perform SAM using PCL and Envire
     */
    class ESAM
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Structures having Eigen members
        //EIGEN_DONT_VECTORIZE //That disables all 128-bit alignment code, and in particular everything vectorization-related

    private:

        /** Invalid key symbol **/
        const gtsam::Symbol invalid_symbol = gtsam::Symbol('u', -1);

        /** Keys to identify poses and landmarks **/
        char pose_key, landmark_key;

        unsigned long int pose_idx, landmark_idx;

        /** The environment in a graph structure **/
        envire::core::TransformGraph _transform_graph;

        /** Factor graph **/
        gtsam::NonlinearFactorGraph _factor_graph;

        /** Optimization **/
        gtsam::GaussNewtonParams optimization_parameters;

        /** Marginals in the estimation **/
        boost::shared_ptr<gtsam::Marginals> marginals;

        /** Values estimates **/
        gtsam::Values estimates_values;

        /** Filter parameters **/
        BilateralFilterParams bfilter_paramaters;

        /** Outlier parameters **/
        OutlierRemovalParams outlier_paramaters;

        /** Keypoint parameters **/
        SIFTKeypointParams keypoint_parameters;

        /** Feature parameters **/
        PFHFeatureParams feature_parameters;

        /** Downsampling factor **/
        float downsample_size;

    public:

        /** Constructors
         */
        ESAM();

        ESAM(const ::base::Pose &pose, const ::base::Vector6d &var_pose,
                const char pose_key, const char landmark_key);

        ESAM(const ::base::Pose &pose, const ::base::Matrix6d &cov_pose,
                const char pose_key, const char landmark_key);

        ESAM(const ::base::TransformWithCovariance &pose_with_cov,
                const char pose_key, const char landmark_key,
                const float downsample_size,
                const BilateralFilterParams &bfilter,
                const OutlierRemovalParams &outliers,
                const SIFTKeypointParams &keypoint,
                const PFHFeatureParams &feature);

        ESAM(const ::base::Pose &pose, const ::base::Matrix6d &cov_pose,
                const char pose_key, const char landmark_key,
                const float downsample_size,
                const BilateralFilterParams &bfilter,
                const OutlierRemovalParams &outliers,
                const SIFTKeypointParams &keypoint,
                const PFHFeatureParams &feature);

        ESAM(const ::base::Pose &pose, const ::base::Vector6d &var_pose, 
                const char pose_key, const char landmark_key,
                const float downsample_size,
                const BilateralFilterParams &bfilter,
                const OutlierRemovalParams &outliers,
                const SIFTKeypointParams &keypoint,
                const PFHFeatureParams &feature);

        ~ESAM();

        void insertFactor(const char key1, const unsigned long int &idx1,
                 const char key2, const unsigned long int &idx2,
                 const base::Time &time, const ::base::Pose &delta_pose,
                 const ::base::Vector6d &var_delta_pose);

        void insertFactor(const char key1, const unsigned long int &idx1,
                const char key2, const unsigned long int &idx2,
                const base::Time &time, const ::base::Pose &delta_pose,
                const ::base::Matrix6d &cov_delta_pose);

        void addDeltaPoseFactor(const base::Time &time, const ::Eigen::Affine3d &delta_tf, const ::base::Vector6d &var_delta_tf);

        void addDeltaPoseFactor(const base::Time &time, const ::base::TransformWithCovariance &delta_pose_with_cov);

        void addDeltaPoseFactor(const base::Time &time, const ::base::Pose &delta_pose, const ::base::Vector6d &var_delta_pose);

        void addDeltaPoseFactor(const base::Time &time, const ::base::Pose &delta_pose, const ::base::Matrix6d &cov_delta_pose);

        void insertValue(const std::string &frame_id, const ::base::TransformWithCovariance &pose_with_cov);

        void insertValue(const char key, const unsigned long int &idx, const ::base::TransformWithCovariance &pose_with_cov);

        void insertValue(const char key, const unsigned long int &idx, const ::base::Pose &pose, const ::base::Matrix6d &cov_pose = ::base::Matrix6d::Identity());

        void addPoseValue(const ::base::TransformWithCovariance &pose_with_cov);

        base::TransformWithCovariance& getLastPoseValueAndId(std::string &frame_id_string);

        std::string currentPoseId();

        std::string currentLandmarkId();

        void optimize();

        ::base::TransformWithCovariance getTransformPose(const std::string &frame_id);

        ::base::samples::RigidBodyState getRbsPose(const std::string &frame_id);

        std::vector< ::base::samples::RigidBodyState > getRbsPoses();

        void pushPointCloud(const ::base::samples::Pointcloud &base_point_cloud, const int height, const int width);

        void keypointsPointCloud(const gtsam::Symbol &frame_id, const float normal_radius, const float feature_radius);

        void transformPointCloud(const ::base::samples::Pointcloud & pc, ::base::samples::Pointcloud & transformed_pc, const Eigen::Affine3d& transformation);

        void transformPointCloud(::base::samples::Pointcloud & pc, const Eigen::Affine3d& transformation);

        void transformPointCloud(pcl::PointCloud< PointType >&pcl_pc, const Eigen::Affine3d& transformation);

        PCLPointCloud &getPointCloud(const std::string &frame_id);

        void mergePointClouds(PCLPointCloud &merged_point_cloud, bool downsample = false);

        void mergePointClouds(base::samples::Pointcloud &base_point_cloud, bool downsample = false);

        void currentPointCloud(base::samples::Pointcloud &base_point_cloud, bool downsample = false);

        void currentPointCloudtoPLY(const std::string &prefixname, bool downsample = false);

        gtsam::Symbol computeAlignedBoundingBox();

        void detectLandmarks();

        bool intersects(const gtsam::Symbol &frame1, const gtsam::Symbol &frame2);

        bool contains(const gtsam::Symbol &container_frame, const gtsam::Symbol &query_frame);

        void containsFrames (const gtsam::Symbol &container_frame_id);

        void printMarginals();

        inline gtsam::NonlinearFactorGraph& factor_graph() { return this->_factor_graph; };

        void printFactorGraph(const std::string &title);

        void graphViz(const std::string &filename);

        void writePlyFile(const base::samples::Pointcloud& points, const std::string& file);

    protected:

        void downsample (PCLPointCloud::Ptr &points, float leaf_size, PCLPointCloud::Ptr &downsampled_out);

        void uniformsample (PCLPointCloud::Ptr &points, float leaf_size, PCLPointCloud::Ptr &uniformsampled_out);

        void removePointsWithoutColor (const PCLPointCloud::Ptr &points, PCLPointCloud::Ptr &points_out);

        void bilateralFilter(const PCLPointCloud::Ptr &points, const double &spatial_width, const double &range_sigma , PCLPointCloud::Ptr &filtered_out);

        void radiusOutlierRemoval(PCLPointCloud::Ptr &points, const double &mean_k, const double &std_mul, PCLPointCloud::Ptr &outliersampled_out);

        void statisticalOutlierRemoval(PCLPointCloud::Ptr &points, const double &radius, const double &min_neighbors, PCLPointCloud::Ptr &outliersampled_out);

        void computeNormals (PCLPointCloud::Ptr &points,
                                float normal_radius,
                                pcl::PointCloud<pcl::Normal>::Ptr &normals_out);

        void computePFHFeatures (PCLPointCloud::Ptr &points,
                      pcl::PointCloud<pcl::Normal>::Ptr &normals,
                      float feature_radius,
                      pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out);

        void detectKeypoints (PCLPointCloud::Ptr &points,
              float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast,
              pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_out);

        void computePFHFeaturesAtKeypoints (pcl::PointCloud<PointType>::Ptr &points,
                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                           pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
                           pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out);

        void computeFPFHFeaturesAtKeypoints (pcl::PointCloud<PointType>::Ptr &points,
                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                           pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptors_out);

        void findFeatureCorrespondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                      pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                      std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out);

        void printKeypoints(const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints);

    public:

        static bool isCovBigger(const Eigen::Matrix3d &cov_position, const double &value)
        {
            /**get the rotation and scaling of the ellipsoid from the covariance matrix **/
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ev( cov_position );
            Eigen::Vector3d e_val = ev.eigenvalues();

            std::cout<<"COMPUTER NORM COV: "<<Eigen::Vector3d(e_val.array().sqrt()).norm()<<"\n";

            if(Eigen::Vector3d(e_val.array().sqrt()).norm() > value)
            {
                return true;
            }
            else
            {
                return false;
            }
        };
    };

}}
#endif
