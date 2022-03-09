#include "Filters.hpp"

float GetMedianValue(std::vector<float> &array)
{
    std::sort(array.begin(), array.end());
    return array[array.size() / 2];
}