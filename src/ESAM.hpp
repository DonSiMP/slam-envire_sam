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

#include <fstream>

/** Rock Base Types **/
#include <base/Eigen.hpp>
#include <base/Pose.hpp>
#include <base/samples/Pointcloud.hpp>

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
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

/** GTSAM Optimizer **/
#include <gtsam/nonlinear/DoglegOptimizer.h>

/** Values to estimate **/
#include <gtsam/nonlinear/Values.h>

#include <envire_sam/Conversions.hpp>
#include <envire_sam/Filters.hpp>
#include <envire_sam/Features.hpp>

namespace envire_sam
{
    /** PCL TYPES **/
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud<PointType> PCLPointCloud;
    typedef typename PCLPointCloud::Ptr PCLPointCloudPtr;

    /**
     * A calss to perform SAM using PCL and Envire
     */
    class ESAM
    {

    private:

        /** The environment in a graph structure **/
        envire::core::TransformGraph envire_graph_;

        /** Factor graph **/
        gtsam::NonlinearFactorGraph factor_graph_;

        /** Values estimates **/
        gtsam::Values estimates_values;

        /** Input point cloud in pcl **/
        PCLPointCloudPtr pcl_point_cloud_in;

    public:

        /** Constructors
         */
        ESAM();
        ESAM(const ::base::Pose &pose, const ::base::Matrix6d &cov_pose, const char key);

        ~ESAM();

        inline gtsam::NonlinearFactorGraph& factor_graph() { return this->factor_graph_; };
    };

}
#endif
