/**\file Conversions.hpp
 *
 * This is a helper class to convert from to point clouds
 *
 * @author Javier Hidalgo Carrio et. al
 * See LICENSE for the license information
 *
 */


#ifndef __ENVIRE_SAM_CONVERSIONS__
#define __ENVIRE_SAM_CONVERSIONS__

#include <fstream>

/** PCL **/
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/** Rock Base Types **/
#include <base/Eigen.hpp>
#include <base/samples/Pointcloud.hpp>

namespace envire { namespace sam
{

    template <class PointType>
    void toPCLPointCloud(const ::base::samples::Pointcloud & pc,
            pcl::PointCloud< PointType >& pcl_pc, double density = 1.0)
    {
        pcl_pc.clear();
        std::vector<bool> mask;
        unsigned sample_count = (unsigned)(density * pc.points.size());

        if(density <= 0.0 || pc.points.size() == 0)
        {
            return;
        }
        else if(sample_count >= pc.points.size())
        {
            mask.resize(pc.points.size(), true);
        }
        else
        {
            mask.resize(pc.points.size(), false);
            unsigned samples_drawn = 0;

            while(samples_drawn < sample_count)
            {
                unsigned index = rand() % pc.points.size();
                if(mask[index] == false)
                {
                    mask[index] = true;
                    samples_drawn++;
                }
            }
        }

        for(size_t i = 0; i < pc.points.size(); ++i)
        {
            if(mask[i])
            {
                if (base::isnotnan<base::Point>(pc.points[i]))
                {
                    PointType pcl_point;
                    pcl_point.x = pc.points[i].x();
                    pcl_point.y = pc.points[i].y();
                    pcl_point.z = pc.points[i].z();
                    uint8_t r = pc.colors[i].x()*255.00;
                    uint8_t g = pc.colors[i].y()*255.00;
                    uint8_t b = pc.colors[i].z()*255.00;
                    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                    pcl_point.rgb = *reinterpret_cast<float*>(&rgb);

                    /** Point info **/
                    pcl_pc.push_back(pcl_point);
                }
            }
        }

        /** All data points are finite (no NaN or Infinite) **/
        pcl_pc.is_dense = true;
    };

    template <class PointType>
    void fromPCLPointCloud(::base::samples::Pointcloud & pc, const pcl::PointCloud< PointType >& pcl_pc, double density = 1.0)
    {
        std::vector<bool> mask;
        unsigned sample_count = (unsigned)(density * pcl_pc.size());

        if(density <= 0.0 || pcl_pc.size() == 0)
        {
            return;
        }
        else if(sample_count >= pcl_pc.size())
        {
            mask.resize(pcl_pc.size(), true);
        }
        else
        {
            mask.resize(pcl_pc.size(), false);
            unsigned samples_drawn = 0;

            while(samples_drawn < sample_count)
            {
                unsigned index = rand() % pcl_pc.size();
                if(mask[index] == false)
                {
                    mask[index] = true;
                    samples_drawn++;
                }
            }
        }

        for(size_t i = 0; i < pcl_pc.size(); ++i)
        {
            if(mask[i])
            {
                PointType const &pcl_point(pcl_pc.points[i]);

                /** Position **/
                pc.points.push_back(::base::Point(pcl_point.x, pcl_point.y, pcl_point.z));

                /** Color **/
                uint8_t r = (static_cast<int>(pcl_point.rgb) >> 16) & 0x0000ff;
                uint8_t g = (static_cast<int>(pcl_point.rgb) >> 8)  & 0x0000ff;
                uint8_t b = (static_cast<int>(pcl_point.rgb))       & 0x0000ff;
                pc.colors.push_back(::base::Vector4d(r, g, b, 255.0)/255.00);
            }
        }
    };

}}
#endif

