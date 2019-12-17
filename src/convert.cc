/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"

#include <pointcloudXYZIR.h>
#include <organized_cloudXYZIR.h>

namespace velodyne_pointcloud {
    /** @brief Constructor. */
    Convert::Convert() :
            data_(new velodyne_rawdata::RawData()), first_rcfg_call(true) {


    }

    void Convert::setConfigFile(std::string file, double max_range, double min_range) {
        this->calibFile = file;
        boost::optional<velodyne_pointcloud::Calibration> calibration = data_->setup(this->calibFile);
        if (calibration) {
            ROS_DEBUG_STREAM("Calibration file loaded.");
            config_.num_lasers = static_cast<uint16_t>(calibration.get().num_lasers);
        } else {
            ROS_ERROR_STREAM("Could not load calibration file!");
        }

        if (config_.organize_cloud) {
            container_ptr_ = boost::shared_ptr<OrganizedCloudXYZIR>(
                    new OrganizedCloudXYZIR(config_.max_range, config_.min_range,
                                            config_.target_frame, config_.fixed_frame,
                                            config_.num_lasers, data_->scansPerPacket()));
        } else {
            container_ptr_ = boost::shared_ptr<PointcloudXYZIR>(
                    new PointcloudXYZIR(config_.max_range, config_.min_range,
                                        config_.target_frame, config_.fixed_frame,
                                        data_->scansPerPacket()));
        }

        container_ptr_->configure(max_range, min_range);
        data_->setParameters(min_range, max_range, 0.0, 2 * M_PI);
    }


    /** @brief Callback for raw scan messages. */
    const sensor_msgs::PointCloud2 Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg) {
//    if (output_.getNumSubscribers() == 0)         // no one listening?
//      return;                                     // avoid much work

        boost::lock_guard<boost::mutex> guard(reconfigure_mtx_);
        // allocate a point cloud with same time and frame ID as raw data
        container_ptr_->setup(scanMsg);

        // process each packet provided by the driver
        for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
            data_->unpack(scanMsg->packets[i], *container_ptr_);
        }
        return container_ptr_->finishCloud();
    }

} // namespace velodyne_pointcloud
