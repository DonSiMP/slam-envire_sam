/**\file Features.hpp
 *
 * This is a helper class to convert compute Features
 * Keypoints and Descriptors in PCL
 *
 * @author Javier Hidalgo Carrio et. al
 * See LICENSE for the license information
 *
 */


#ifndef __ENVIRE_SAM_FILTERS__
#define __ENVIRE_SAM_FILTERS__


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace envire { namespace sam
{
    static void downsample (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float leaf_size, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &downsampled_out)
    {

      pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
      vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
      vox_grid.setInputCloud (points);
      vox_grid.filter (*downsampled_out);

      return;
    };

    static void bilateralFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, const double &spatial_width, const double &range_sigma , pcl::PointCloud<pcl::PointXYZRGB>::Ptr &filtered_out)
    {
        pcl::FastBilateralFilter<pcl::PointXYZRGB> b_filter;

        /** Configure Bilateral filter **/
        b_filter.setSigmaS(spatial_width);
        b_filter.setSigmaR(range_sigma);

        b_filter.setInputCloud(points);
        filtered_out->width = points->width;
        filtered_out->height = points->height;
        b_filter.filter(*filtered_out);
    };

    static void statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, const double &radius, const double &min_neighbors, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outliersampled_out)
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;

        ror.setRadiusSearch(radius);
        ror.setMinNeighborsInRadius(min_neighbors);

        #ifdef DEBUG_PRINTS
        std::cout<<"RADIUS FILTER\n";
        #endif
        pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
        outliersampled_out->width = points->width;
        outliersampled_out->height = points->height;
        ror.setInputCloud(points);
        ror.filter (*outliersampled_out);
    };

    static void radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, const double &mean_k, const double &std_mul, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outliersampled_out)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;

        sor.setMeanK(mean_k);
        sor.setStddevMulThresh(std_mul);

        #ifdef DEBUG_PRINTS
        std::cout<<"STATISTICAL FILTER\n";
        #endif
        outliersampled_out->width = points->width;
        outliersampled_out->height = points->height;
        sor.setInputCloud(points);
        sor.filter (*outliersampled_out);
    }
}}
#endif
