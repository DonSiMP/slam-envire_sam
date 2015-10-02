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

using namespace envire_sam;

ESAM::ESAM()
{
    base::Pose pose;
    base::Matrix6d cov_pose;
    ESAM(pose, cov_pose, 'x');
}

ESAM::ESAM(const ::base::Pose &pose, const ::base::Matrix6d &cov_pose, const char key)
{

    gtsam::Pose3 pose_0(gtsam::Rot3(pose.orientation), gtsam::Point3(pose.position));
    gtsam::Matrix cov_matrix = cov_pose;
    gtsam::noiseModel::Gaussian::shared_ptr cov_pose_0 = gtsam::noiseModel::Gaussian::Covariance(cov_matrix); 

    // Add a prior on pose x1. This indirectly specifies where the origin is.
    this->factor_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(key, 0), pose_0, cov_pose_0)); // add directly to graph

}

ESAM::~ESAM(){};
