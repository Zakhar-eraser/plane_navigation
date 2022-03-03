#include "PlaneNavigationManager.hpp"

PlaneNavigationManager* PlaneNavigationManager::instance_{nullptr};
std::mutex PlaneNavigationManager::mutex_;

PlaneNavigationManager *PlaneNavigationManager::GetInstance()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (instance_ == nullptr)
    {
        instance_ = new PlaneNavigationManager();
    }
    return instance_;
}

PlaneNavigationManager::PlaneNavigationManager()
{
	ranges = std::vector<std::vector<float>>(4, std::vector<float>(set));
}

std::vector<float> PlaneNavigationManager::getEstimatedPose()
{
	return estimatedPose_;
}

void PlaneNavigationManager::setRange(float right, float left, float front, float back)
{
	if(counter < set && calibration)
	{
		ranges[0][counter] = right;
		ranges[1][counter] = left;
		ranges[2][counter] = front;
		ranges[3][counter] = back;
		counter++;
	}
	else
	{
		scans_->frontLaser.range = front;
		scans_->leftLaser.range = left;
		scans_->backLaser.range = back;
		scans_->rightLaser.range = right;
	}
}

void PlaneNavigationManager::setOrientation(float yaw, float pitch, float roll)
{
	scans_->pitch = pitch;
	scans_->roll = roll;
	scans_->yaw = yaw;
}

void PlaneNavigationManager::updatePose()
{
	nav_->CalculatePose();
	Pose lastPose = nav_->GetPose();
	estimatedPose_[0] = lastPose.position.x;
	estimatedPose_[1] = lastPose.position.y;
	estimatedPose_[2] = lastPose.angle;
}

bool PlaneNavigationManager::calibrateRectMap()
{
	if(!calibration) calibration = true;
	if(counter == set)
	{
		counter = 0;
		std::sort(ranges[0].begin(), ranges[0].end());
		std::sort(ranges[1].begin(), ranges[1].end());
		std::sort(ranges[2].begin(), ranges[2].end());
		std::sort(ranges[3].begin(), ranges[3].end());
		scans_->rightLaser.range = ranges[0][center];
		scans_->leftLaser.range = ranges[1][center];
		scans_->frontLaser.range = ranges[2][center];
		scans_->backLaser.range = ranges[3][center];
		nav_->CalibrateWidth("line2");
		calibration = false;
		return true;
	}
	else return false;
}

PlaneNavigationManager::PlaneNavigationManager(/* args */)
{
	scans_ = new SensorScans();
	YAML::Node robotConfig = YAML::LoadFile("../plane_navigation/config/robot_config.yaml");
	YAML::Node leftConfig = robotConfig["left"];
	YAML::Node rightConfig = robotConfig["right"];
	YAML::Node frontConfig = robotConfig["front"];
	YAML::Node backConfig = robotConfig["back"];
	std::string offsets = "offsets";
	std::string isOn = "is_on";

	scans_->leftLaser.isOn = leftConfig[isOn].as<bool>();
	scans_->rightLaser.isOn = rightConfig[isOn].as<bool>();
	scans_->backLaser.isOn = backConfig[isOn].as<bool>();
	scans_->frontLaser.isOn = frontConfig[isOn].as<bool>();
	scans_->leftLaser.offsets = leftConfig[offsets].as<pair>();
	scans_->rightLaser.offsets = rightConfig[offsets].as<pair>();
	scans_->backLaser.offsets = backConfig[offsets].as<pair>();
	scans_->frontLaser.offsets = frontConfig[offsets].as<pair>();
						 
    nav_ = new Navigator("../plane_navigation/config/map.yaml", scans_);
	Position initPos(YAML::Load("../plane_navigation/config/map.yaml")["initial_pose"].as<pair>());
	nav_->SetLastPose(Pose(initPos, 0));
	estimatedPose_ = {0,0,0};
}

PlaneNavigationManager::~PlaneNavigationManager()
{
	delete scans_;
    delete nav_;
}


