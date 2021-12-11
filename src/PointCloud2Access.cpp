#include "PointCloud2Access.hpp"

PointCloud2Access::PointCloud2Access(sensor_msgs::PointCloud2 message)
{
    this->message = message;
}

 int PointCloud2Access::GetWidth()
{
    return this->message.width;
}
 int PointCloud2Access::GetHeight()
{
    return this->message.height;
}
 sensor_msgs::PointCloud2Iterator<float> PointCloud2Access::operator()(std::pair<int, int> point)
{
    int row = point.first;
    int column = point.second;
    sensor_msgs::PointCloud2Iterator<float> iter(message, "x");
    if(IsInFrame(row, column))
    {
        return iter + (message.width * row + column);
    } else return iter.end();
}
 sensor_msgs::PointCloud2Iterator<float> PointCloud2Access::operator()(int row, int column)
{
    sensor_msgs::PointCloud2Iterator<float> iter(message, "x");
    if(IsInFrame(row, column))
    {
        return iter + (message.width * row + column);
    } else return iter.end();
}
 PointCloud2Access &PointCloud2Access::operator=(PointCloud2Access &array)
{
    this->message = array.message;
    return *this;
}