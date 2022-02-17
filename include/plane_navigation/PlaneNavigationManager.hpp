#ifndef PLANE_NAVIGATION_PLANE_NAVIGATION_MANAGER_HPP_
#define PLANE_NAVIGATION_PLANE_NAVIGATION_MANAGER_HPP_

#include <mutex>
#include <vector>
#include <algorithm>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "plane_navigation.hpp"

class PlaneNavigationManager
{
	private:
		static PlaneNavigationManager*	instance_;
		static std::mutex				mutex_;
		Navigator* 						nav_;
		SensorScans*					scans_;
		std::vector<float>				estimatedPose_;

	protected:
		PlaneNavigationManager(/* args */);
		~PlaneNavigationManager();

	public:
		PlaneNavigationManager(PlaneNavigationManager& other) = delete;
		void operator=(const PlaneNavigationManager &) = delete;
		static PlaneNavigationManager* GetInstance();
		void updatePose();
		std::vector<float> getEstimatedPose();
		void setRange(float right, float left, float front, float back);
		void setOrientation(float yaw, float pitch, float roll);
		
};

#endif