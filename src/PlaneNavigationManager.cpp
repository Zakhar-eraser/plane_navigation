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

std::vector<float> PlaneNavigationManager::getEstimatedPose()
{
	return estimatedPose_;
}

void PlaneNavigationManager::setRange(float right, float left, float front, float back)
{
	scans_->frontLaser.range = front;
	scans_->leftLaser.range = left;
	scans_->backLaser.range = back;
	scans_->rightLaser.range = right;
}

void PlaneNavigationManager::setOrientation(float yaw, float pitch, float roll)
{
	scans_->pitch = pitch;
	scans_->roll = roll;
	scans_->yaw = yaw;
}

void PlaneNavigationManager::updatePose()
{
	nav_->isUpdate = true;
	nav_->CalculatePose();
	Pose lastPose = nav_->GetPose();
	estimatedPose_[0] = lastPose.position.x;
	estimatedPose_[1] = lastPose.position.y;
	estimatedPose_[2] = lastPose.angle;
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


