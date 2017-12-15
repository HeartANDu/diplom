#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "youbot_driver/youbot/YouBotBase.hpp"

using namespace youbot;

const float vel_y_const = 0.1;
float vel_x = 0.1, vel_y = 0, omega = 0, current_vel_x, current_vel_y, current_omega;
float drho = 5;
bool Obstacle = false, PrevSideObstacle = false, SideObstacle = false, direction;
double current_time = 0, previous_time = 0, start_time = 0;
bool youBotHasBase = false;
YouBotBase* myYouBotBase = 0;
float x = 0, y = 0, psi = 0, prev_x, prev_y, prev_psi;
float lost_obstacle_x = 0;

void setSpeed(float x, float y, float omega) {
    quantity<si::velocity> longitudinalVelocity, transversalVelocity;
    quantity<si::angular_velocity> angularVelocity;
    try {
      if (youBotHasBase) {
        longitudinalVelocity = x * meter_per_second;
        transversalVelocity = y * meter_per_second;
        angularVelocity = omega * radian_per_second;

        myYouBotBase->setBaseVelocity(longitudinalVelocity,transversalVelocity,angularVelocity);
      }
    } catch (std::exception& e) {
      ROS_INFO("%s", e.what());
      ROS_INFO("Unhandled exeption");
    }
}

void getSpeed(float& x, float& y, float& omega) {
    quantity<si::velocity> actualLongitudinalVelocity, actualTransversalVelocity;
    quantity<si::angular_velocity> actualAngularVelocity;
    try {
        if (youBotHasBase) {
            myYouBotBase->getBaseVelocity(actualLongitudinalVelocity,actualTransversalVelocity,actualAngularVelocity);
            x = actualLongitudinalVelocity / meter_per_second;
            y = actualTransversalVelocity / meter_per_second;
            omega = actualAngularVelocity / radian_per_second;
        } else {
            x = 0;
            y = 0;
            omega = 0;
        }
    } catch (std::exception& e) {
        ROS_INFO("%s", e.what());
        ROS_INFO("Unhandled exeption");
    }
}

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  std::vector<float> angles;
  std::vector<float> ranges;
  for (int i=0; i<scan->ranges.size(); i++) {
    if (!std::isnan(scan->ranges[i]) && !std::isinf(scan->ranges[i])) {
      angles.push_back(scan->angle_min + i * scan->angle_increment);
      ranges.push_back(scan->ranges[i]);
    }
  }

  float x_obs_start = -1, x_obs_end = -1, y_obs_start = 0, y_obs_end = 0;
  Obstacle = false;
  for (int i=0; i<angles.size(); i++) {
      if (ranges[i]*cos(angles[i]) <= 0.50 && std::abs(ranges[i]*sin(angles[i])) <= 0.25) Obstacle = true;
      if (Obstacle)  {
          float a = (ranges[i]-ranges[i-1])/std::abs(angles[i]-angles[i-1]);
          if (ranges[i]*cos(angles[i]) <= 0.50 && std::abs(ranges[i]*sin(angles[i])) <= 0.25) {
            if (std::abs(a) > drho && a < 0) {
                x_obs_start = ranges[i]*cos(angles[i]);
                y_obs_start = ranges[i]*sin(angles[i]);
            } else if (x_obs_start == -1) {
                x_obs_start = ranges[i]*cos(angles[i]);
                y_obs_start = ranges[i]*sin(angles[i]);
            }
            if (std::abs(a) > drho && a > 0) {
                x_obs_end = ranges[i-1]*cos(angles[i-1]);
                y_obs_end = ranges[i-1]*sin(angles[i-1]);
            }
          } else if (x_obs_end == -1 && x_obs_start != -1) {
              x_obs_end = ranges[i-1]*cos(angles[i-1]);
              y_obs_end = ranges[i-1]*sin(angles[i-1]);
          }
      }
  }

  getSpeed(current_vel_x,current_vel_y,current_omega);

  previous_time = current_time;
  current_time = ros::Time::now().toSec();

  prev_x = x;
  prev_y = y;
  prev_psi = psi;

  x = prev_x+(current_time-previous_time)*(current_vel_x*cos(prev_psi)-current_vel_y*sin(prev_psi));
  y = prev_y+(current_time-previous_time)*(current_vel_x*sin(prev_psi)+current_vel_y*cos(prev_psi));
  psi = prev_psi+(current_time-previous_time)*current_omega;

  ROS_INFO("%f %f %f %f %f %f %f", x, y, psi, current_vel_x, current_vel_y, current_omega, current_time - start_time);

  PrevSideObstacle = SideObstacle;
  SideObstacle = false;
  if (ranges[0] < 0.3 || ranges[ranges.size()-1] < 0.3) {
      SideObstacle = true;
  }

  if (!SideObstacle && PrevSideObstacle) {
      lost_obstacle_x = x;
      PrevSideObstacle = false;
  }

  if (Obstacle) {
      direction = std::abs(y_obs_start) < std::abs(y_obs_end);
      if (direction) vel_y = -1 * vel_y_const;	
      else vel_y = 1 * vel_y_const;
  } else if (lost_obstacle_x != 0 && x - lost_obstacle_x >= 0.6) {
      if (std::abs(y) > 0.01) {
          if (!direction) vel_y = -1 * vel_y_const;
          else vel_y = 1 * vel_y_const;
      } else {
          vel_y = 0;
          lost_obstacle_x = 0;
      }
  } else {
      vel_y = 0;
  }

  setSpeed(vel_x,vel_y,omega);

}

int main(int argc, char **argv)
{
  try {
    myYouBotBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
    myYouBotBase->doJointCommutation();
    youBotHasBase = true;
  } catch (std::exception& e) {
    ROS_INFO("%s", e.what());
    youBotHasBase = false;
  }

  ros::init(argc, argv, "navigation_by_laser");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("scan", 1, chatterCallback);

  start_time = ros::Time::now().toSec();
  current_time = start_time;

  ROS_INFO("x, y, psi, current_vel_x, current_vel_y, current_omega, t");
  ROS_INFO("%f %f %f %f %f %f %f", x, y, psi, current_vel_x, current_vel_y, current_omega, current_time - start_time);

  ros::spin();

  return 0;
}
