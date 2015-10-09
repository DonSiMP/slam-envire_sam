rock_library(envire_sam
    HEADERS Conversions.hpp
            Features.hpp
            Configuration.hpp
            Filters.hpp
            ESAM.hpp

    SOURCES ESAM.cpp
    DEPS_CMAKE Boost
    DEPS_PKGCONFIG base-types
    DEPS_PKGCONFIG pcl_registration-1.7 pcl_filters-1.7 pcl_kdtree-1.7 pcl_features-1.7 pcl_keypoints-1.7 flann
    DEPS_PKGCONFIG gtsam envire_core)

