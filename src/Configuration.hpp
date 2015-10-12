/**\file Configuration.hpp
 *
 * @author Javier Hidalgo Carrio et. al
 * See LICENSE for the license information
 *
 */


#ifndef __ENVIRE_CONFIGURATION__
#define __ENVIRE_CONFIGURATION__

namespace envire { namespace sam
{
    enum OutlierFilterType
    {
        NONE,
        STATISTICAL,
        RADIUS
    };

    struct BilateralFilterParams
    {
        bool filterOn;
        float spatial_width; //size of the window bilateral filter
        float range_sigma; // the standard deviation of the Gaussian for the intensity difference
    };

    struct OutlierRemovalParams
    {
        OutlierFilterType type;

        //STATISTICAL: the number of nearest neighbors to use for mean distance estimation (nr_k)
        //RADIUS: Get the radius of the sphere that will determine which points are neighbors (radiu).
        float parameter_one;

        //STATISTICAL: the standard deviation multiplier for the distance threshold calculation.(stddev_null)
        //RADIUS: number of neighbors that need to be present in order to be classified as an inlier(min_pts)
        float parameter_two;
    };

    struct SIFTKeypointParams
    {
        float min_scale;
        int nr_octaves;
        int nr_octaves_per_scale;
        float min_contrast;
    };

}}

#endif
