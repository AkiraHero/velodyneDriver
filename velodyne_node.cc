
#include "opencv/cv.hpp"
#include "./include/driver.h"



int main(int argc, char** argv) {

    // start the driver
    velodyne_driver::VelodyneDriver dvr("VLP16", "/home/akira/Project/velodyne_driver/VLP16db.yaml");
    sensor_msgs::PointCloud2 cloud;
    cv::Mat img;
    img.create(cvSize(1200, 1200), CV_8UC3);
    img.setTo(0);
    float pixelsize = 0.01;
    // loop until shut down or end of file
    while (dvr.poll(cloud)) {
        img.setTo(0);
        auto iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
        auto iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
        auto iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
        auto iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
        auto iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t>(cloud, "ring");
        for (int i = 0; i != cloud.width; i++) {
            double x = *iter_x;
            double y = *iter_y;
            double z = *iter_z;
            int mapx = 600 + x / pixelsize;
            int mapy = 600 + y / pixelsize;
            if (z < 0.0) {
                int r = 255;
                cv::circle(img, cvPoint(mapx, mapy), 1, cv::Scalar(r, 0, 0));
            }
                // 红色
            else {
                int b = 255;
                cv::circle(img, cvPoint(mapx, mapy), 1, cv::Scalar(0, 0, b));;
            }
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_ring;
            ++iter_intensity;
        }
        cv::imshow("LIDAR", img);
        cv::waitKey(1);
    }
    return 0;
}
