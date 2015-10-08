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
    this->pose_key = 'x';
    this->landmark_key = 'l';
    ESAM(pose, cov_pose, this->pose_key, this->landmark_key);
}


ESAM::ESAM(const ::base::TransformWithCovariance &pose_with_cov, const char pose_key, const char landmark_key)
{
    gtsam::Pose3 pose_0(gtsam::Rot3(pose_with_cov.orientation), gtsam::Point3(pose_with_cov.translation));
    gtsam::Matrix cov_matrix = pose_with_cov.cov;
    gtsam::noiseModel::Gaussian::shared_ptr cov_pose_0 = gtsam::noiseModel::Gaussian::Covariance(cov_matrix);

    /** Optimization parameters **/

    // Stop iterating once the change in error between steps is less than this value
    this->parameters.relativeErrorTol = 1e-5;

    // Do not perform more than N iteration steps
    this->parameters.maxIterations = 100;

    this->pose_key = pose_key;
    this->landmark_key = landmark_key;
    this->pose_idx = 0;
    this->landmark_idx = 0;

    // Add a prior on pose x0. This indirectly specifies where the origin is.
    this->_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(this->pose_key, this->pose_idx), pose_0, cov_pose_0)); // add directly to graph

}
ESAM::ESAM(const ::base::Pose &pose, const ::base::Matrix6d &cov_pose, const char pose_key, const char landmark_key)
{

    gtsam::Pose3 pose_0(gtsam::Rot3(pose.orientation), gtsam::Point3(pose.position));
    gtsam::Matrix cov_matrix = cov_pose;
    gtsam::noiseModel::Gaussian::shared_ptr cov_pose_0 = gtsam::noiseModel::Gaussian::Covariance(cov_matrix);

    /** Optimization parameters **/

    // Stop iterating once the change in error between steps is less than this value
    this->parameters.relativeErrorTol = 1e-5;
    // Do not perform more than N iteration steps
    this->parameters.maxIterations = 100;

    this->pose_key = pose_key;
    this->landmark_key = landmark_key;
    this->pose_idx = 0;
    this->landmark_idx = 0;

    // Add a prior on pose x0. This indirectly specifies where the origin is.
    this->_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(this->pose_key, this->pose_idx), pose_0, cov_pose_0)); // add directly to graph
}

ESAM::ESAM(const ::base::Pose &pose, const ::base::Vector6d &var_pose, const char pose_key, const char landmark_key)
{
    gtsam::Pose3 pose_0(gtsam::Rot3(pose.orientation), gtsam::Point3(pose.position));
    gtsam::Vector variances = var_pose;
    gtsam::noiseModel::Diagonal::shared_ptr cov_pose_0 = gtsam::noiseModel::Diagonal::Variances(variances);

    /** Optimzation parameters **/

    // Stop iterating once the change in error between steps is less than this value
    this->parameters.relativeErrorTol = 1e-5;
    // Do not perform more than N iteration steps
    this->parameters.maxIterations = 100;

    this->pose_key = pose_key;
    this->landmark_key = landmark_key;
    this->pose_idx = 0;
    this->landmark_idx = 0;

    // Add a prior on pose x0. This indirectly specifies where the origin is.
    this->_factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(this->pose_key, this->pose_idx), pose_0, cov_pose_0)); // add directly to graph

}

ESAM::~ESAM(){};

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
        boost::intrusive_ptr<envire::sam::PoseItem> pose_item(new envire::sam::PoseItem());
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
        boost::intrusive_ptr<envire::sam::PoseItem> pose_item(new envire::sam::PoseItem());
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
        boost::intrusive_ptr<envire::sam::PoseItem> pose_item(new envire::sam::PoseItem());
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
    ::base::TransformWithCovariance tf_cov;

    gtsam::Symbol frame_id = gtsam::Symbol(this->pose_key, this->pose_idx);
    try
    {
        std::vector<envire::core::ItemBase::Ptr> items = this->_transform_graph.getItems(frame_id);
        boost::intrusive_ptr<envire::sam::PoseItem> pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
        frame_id_string = static_cast<std::string>(frame_id);
        return pose_item->getData();
    }catch(envire::core::UnknownFrameException &ufex)
    {
        std::cerr << ufex.what() << std::endl;
    }

    return tf_cov;
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
            boost::intrusive_ptr<envire::sam::PoseItem> pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
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
    gtsam::GaussNewtonOptimizer optimizer(this->_factor_graph, initialEstimate, this->parameters);

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
            boost::intrusive_ptr<envire::sam::PoseItem> pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
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
        boost::intrusive_ptr<envire::sam::PoseItem> pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
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
        boost::intrusive_ptr<envire::sam::PoseItem> pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
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

void ESAM::printMarginals()
{
    std::cout.precision(3);
    for(register unsigned int i=0; i<this->pose_idx+1; ++i)
    {
        gtsam::Symbol frame_id(this->pose_key, i);
        std::cout <<this->pose_key<<i<<" covariance:\n" << this->marginals->marginalCovariance(frame_id) << std::endl;
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
 
