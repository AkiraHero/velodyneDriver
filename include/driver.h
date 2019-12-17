// Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef VELODYNE_DRIVER_DRIVER_H
#define VELODYNE_DRIVER_DRIVER_H

#include <string>

#include <input.h>
#include <convert.h>

namespace velodyne_driver
{

class VelodyneDriver
{
public:
  VelodyneDriver(std::string modelname, std::string calibfile, std::pair<double, double>range=std::make_pair(0, 200), std::string pcapfile="");
  ~VelodyneDriver() {}

  bool poll(sensor_msgs::PointCloud2 &cloud);

private:

  // configuration parameters
  struct
  {
    std::string frame_id;            // tf frame ID
    std::string model;               // device model name
    int    npackets;                 // number of packets to collect
    double rpm;                      // device rotation rate (RPMs)
    int cut_angle;                   // cutting angle in 1/100°
    double time_offset;              // time in seconds added to each velodyne time stamp
    bool enabled;                    // polling is enabled
  }
  config_;

  boost::shared_ptr<Input> input_;

  int last_azimuth_;
  velodyne_pointcloud::Convert converter;

};

}  // namespace velodyne_driver

#endif  // VELODYNE_DRIVER_DRIVER_H
