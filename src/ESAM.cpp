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

ESAM::ESAM(const ::base::Pose &pose, const ::base::Matrix6d &cov_pose, const char pose_key, const char landmark_key)
{

    gtsam::Pose3 pose_0(gtsam::Rot3(pose.orientation), gtsam::Point3(pose.position));
    gtsam::Matrix cov_matrix = cov_pose;
    gtsam::noiseModel::Gaussian::shared_ptr cov_pose_0 = gtsam::noiseModel::Gaussian::Covariance(cov_matrix); 

    /** Optimzation parameters **/

    // Stop iterating once the change in error between steps is less than this value
    this->parameters.relativeErrorTol = 1e-5;
    // Do not perform more than N iteration steps
    this->parameters.maxIterations = 100;

    this->pose_key = pose_key;
    this->landmark_key = landmark_key;
    this->pose_idx = 0;
    this->landmark_idx = 0;

    // Add a prior on pose x1. This indirectly specifies where the origin is.
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

    // Add a prior on pose x1. This indirectly specifies where the origin is.
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

void ESAM::addFactor(const base::Time &time, const ::base::Pose &delta_pose, const ::base::Vector6d &var_delta_pose)
{
    this->pose_idx++;
    return this->insertFactor(this->pose_key, this->pose_idx-1, this->pose_key, this->pose_idx, time, delta_pose, var_delta_pose);
}

void ESAM::addFactor(const base::Time &time, const ::base::Pose &delta_pose, const ::base::Matrix6d &cov_delta_pose)
{
    this->pose_idx++;
    return this->insertFactor(this->pose_key, this->pose_idx-1, this->pose_key, this->pose_idx, time, delta_pose, cov_delta_pose);
}

void ESAM::insertValue(const char key, const unsigned long int &idx, const ::base::Pose &pose)
{
    gtsam::Symbol symbol = gtsam::Symbol(key, idx);
    try
    {
        boost::intrusive_ptr<envire::sam::PoseItem> pose_item(new envire::sam::PoseItem());
        pose_item->setData(pose);
        this->_transform_graph.addItemToFrame(symbol, pose_item);

    }catch(envire::core::UnknownFrameException &ufex)
    {
        std::cerr << ufex.what() << std::endl;
    }
}

void ESAM::optimize()
{
    gtsam::Values initialEstimate;

    /** Initial estimates for poses **/
    for(register unsigned int i=0; i<this->pose_idx+1; ++i)
    {
        gtsam::Symbol frame_id(this->pose_key, i);
        frame_id.print();
        try
        {
            std::vector<envire::core::ItemBase::Ptr> items = this->_transform_graph.getItems(frame_id);
            boost::intrusive_ptr<envire::sam::PoseItem> pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
            gtsam::Pose3 pose(gtsam::Rot3(pose_item->getData().orientation), gtsam::Point3(pose_item->getData().position));
            initialEstimate.insert(frame_id, pose);
        }catch(envire::core::UnknownFrameException &ufex)
        {
            std::cerr << ufex.what() << std::endl;
            return;
        }
    }


    //initialEstimate.print("\nInitial Estimate:\n"); // print

    /** Create the optimizer ... **/
    gtsam::GaussNewtonOptimizer optimizer(this->_factor_graph, initialEstimate, this->parameters);

    /** Optimize **/
    gtsam::Values result = optimizer.optimize();
    result.print("Final Result:\n");

    /** Save the marginals **/
    this->marginals.reset(new gtsam::Marginals(this->_factor_graph, result));

    /** Store the result in the transform graph **/
    gtsam::Values::iterator key_value = result.begin();
    for(; key_value != result.end(); ++key_value)
    {
        try
        {
            gtsam::Symbol const &frame_id(key_value->key);
            std::vector<envire::core::ItemBase::Ptr> items = this->_transform_graph.getItems(frame_id);
            boost::intrusive_ptr<envire::sam::PoseItem> pose_item = boost::static_pointer_cast<envire::sam::PoseItem>(items[0]);
            base::Pose result_pose;
            boost::shared_ptr<gtsam::Pose3> pose = boost::reinterpret_pointer_cast<gtsam::Pose3>(key_value->value.clone());
            std::cout<<"reinterpret: "<<pose->translation()<<"\n";
            pose_item->setData(result_pose);
        }catch(envire::core::UnknownFrameException &ufex)
        {
        }
    }

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
 
