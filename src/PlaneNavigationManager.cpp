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
	lastPose_ = nav_->GetPose();
	estimatedPose_[0] = lastPose_.position.x;
	estimatedPose_[1] = lastPose_.position.y;
	estimatedPose_[2] = lastPose_.angle;
}


PlaneNavigationManager::PlaneNavigationManager(/* args */)
{
	scans_ = new SensorScans();
	YAML::Node sensorTfNode = YAML::LoadFile("../plane_navigation/config/sensors_tf.yaml");
	YAML::Node switcherNode = YAML::LoadFile("../plane_navigation/config/sensor_switcher.yaml");
	YAML::Node initPose = YAML::LoadFile("../plane_navigation/config/initial_pose.yaml");

	scans_->leftLaser.isOn = switcherNode["use_left"].as<bool>();
	scans_->rightLaser.isOn = switcherNode["use_right"].as<bool>();
	scans_->backLaser.isOn = switcherNode["use_back"].as<bool>();
	scans_->frontLaser.isOn = switcherNode["use_front"].as<bool>();
	scans_->leftLaser.offsets = sensorTfNode["left"].as<pair>();
	scans_->rightLaser.offsets = sensorTfNode["right"].as<pair>();
	scans_->backLaser.offsets = sensorTfNode["back"].as<pair>();
	scans_->frontLaser.offsets = sensorTfNode["front"].as<pair>();
						 
	lastPose_ = Pose(initPose["x"].as<float>(), initPose["y"].as<float>(), initPose["yaw"].as<float>());
    nav_ = new Navigator("../plane_navigation/config/map.yaml", scans_);
	nav_->SetLastPose(lastPose_);
	//nav_->CalibrateSymmetricMap();
	estimatedPose_ = {0,0,0};
}

PlaneNavigationManager::~PlaneNavigationManager()
{
	delete scans_;
    delete nav_;
}


