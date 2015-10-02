/**\file Features.hpp
 *
 * This is a helper class to convert compute Features
 * Keypoints and Descriptors in PCL
 *
 * @author Javier Hidalgo Carrio et. al
 * See LICENSE for the license information
 *
 */


#ifndef __ENVIRE_SAM_FEATURES__
#define __ENVIRE_SAM_FEATURES__

/** PCL **/
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/pfh.h>
#include <pcl/keypoints/sift_keypoint.h>

namespace envire { namespace sam
{
    static void compute_surface_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                                float normal_radius,
                                pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
    {
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;

      // Use a FLANN-based KdTree to perform neighborhood searches
      //norm_est.setSearchMethod (pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZRGB>));
      norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));


      // Specify the size of the local neighborhood to use when computing the surface normals
      norm_est.setRadiusSearch (normal_radius);

      // Set the input points
      norm_est.setInputCloud (points);

      // Estimate the surface normals and store the result in "normals_out"
      norm_est.compute (*normals_out);
    };

    static void compute_PFH_features (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                          pcl::PointCloud<pcl::Normal>::Ptr &normals,
                          float feature_radius,
                          pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
    {
        // Create a PFHEstimation object
        pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;

        // Set it to use a FLANN-based KdTree to perform its neighborhood searches
        pfh_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));

        // Specify the radius of the PFH feature
        pfh_est.setRadiusSearch (feature_radius);

        // Set the input points and surface normals
        pfh_est.setInputCloud (points);
        pfh_est.setInputNormals (normals);

        // Compute the features
        pfh_est.compute (*descriptors_out);

        return;
    };

    static void detect_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
              float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast,
              pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_out)
    {
        pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;

        // Use a FLANN-based KdTree to perform neighborhood searches
        sift_detect.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));

        // Set the detection parameters
        sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
        sift_detect.setMinimumContrast (min_contrast);

        // Set the input
        sift_detect.setInputCloud (points);

        // Detect the keypoints and store them in "keypoints_out"
        sift_detect.compute (*keypoints_out);

        return;
    };

    static void compute_PFH_features_at_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                               pcl::PointCloud<pcl::Normal>::Ptr &normals,
                               pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
                               pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
    {
        // Create a PFHEstimation object
        pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;

        // Set it to use a FLANN-based KdTree to perform its neighborhood searches
        pfh_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));

        // Specify the radius of the PFH feature
        pfh_est.setRadiusSearch (feature_radius);

        /* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
        * use them as an input to our PFH estimation, which expects clouds of PointXYZRGBA points.  To get around this,
        * we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to
        * "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGBA>).  Note that the original cloud doesn't have any RGB
        * values, so when we copy from PointWithScale to PointXYZRGBA, the new r,g,b fields will all be zero.
        */

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);

        // Use all of the points for analyzing the local structure of the cloud
        pfh_est.setSearchSurface (points);
        pfh_est.setInputNormals (normals);

        // But only compute features at the keypoints
        pfh_est.setInputCloud (keypoints_xyzrgb);

        // Compute the features
        pfh_est.compute (*descriptors_out);

        return;
    };

    static void find_feature_correspondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
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
    };
}}
#endif

