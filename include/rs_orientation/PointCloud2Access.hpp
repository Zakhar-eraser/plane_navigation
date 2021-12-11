#ifndef POINTCLOUD2_ACCESS
#define POINTCLOUD2_ACCESS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class PointCloud2Access
{
    private:
        sensor_msgs::PointCloud2 message;

        bool IsInFrame(int row, int column)
        {
            return row >= 0 && row < message.height && column >= 0 && column < message.width; 
        }
    public:
        PointCloud2Access(sensor_msgs::PointCloud2 message);
         int GetWidth();
         int GetHeight();
         sensor_msgs::PointCloud2Iterator<float> operator()(std::pair<int, int> point);
         sensor_msgs::PointCloud2Iterator<float> operator()(int row, int column);
         PointCloud2Access &operator=(PointCloud2Access &array);
};
#endif