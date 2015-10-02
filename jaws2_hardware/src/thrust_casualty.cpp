#include "ros/ros.h"
#include "jaws2_msgs/ThrustStamped.h"
#include "jaws2_msgs/PwmStamped.h"

class ThrustCal
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber ts;
    ros::Publisher pwm;
    jaws2_msgs::PwmStamped duration;
    int aft_calibrate(double raw_force);
    int stbd_calibrate(double raw_force);
    int port_calibrate(double raw_force);

  public:
    ThrustCal();
    void callback(const jaws2_msgs::ThrustStamped::ConstPtr& force);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thrust_cal");
  ThrustCal thrust_cal;
  thrust_cal.loop();
}

ThrustCal::ThrustCal() : nh()
{
  ts = nh.subscribe<jaws2_msgs::ThrustStamped>("solver/thrust", 1, &ThrustCal::callback, this);
  pwm = nh.advertise<jaws2_msgs::PwmStamped>("thrust_cal/pwm", 1);
}

void ThrustCal::callback(const jaws2_msgs::ThrustStamped::ConstPtr& force)
{
  duration.header.stamp = force->header.stamp;

  duration.pwm.aft = aft_calibrate(force->thrust.aft);
  duration.pwm.stbd = stbd_calibrate(force->thrust.stbd);
  duration.pwm.port = port_calibrate(force->thrust.port);

  pwm.publish(duration);
}

void ThrustCal::loop()
{
  ros::spin();
}

int ThrustCal::aft_calibrate(double raw_force)
{
  int pwm = 1500;
  if(raw_force < -10.0)
  {
    pwm = 1360;
  }
  else if(raw_force > 10.0)
  {
    pwm = 1690;
  }
  return pwm;
}

int ThrustCal::stbd_calibrate(double raw_force)
{
  int pwm_offset = 0;
  if(raw_force < -5.0)
  {
    pwm_offset = int(raw_force / 25.0 * 15.0) - 110;
  }
  else if(raw_force > 5.0)
  {
    pwm_offset = int(raw_force / 25.0 * 40.0) + 50;
  }
  return 1500 + pwm_offset;
}

int ThrustCal::port_calibrate(double raw_force)
{
  int pwm_offset = 0;
  if(raw_force < -5.0)
  {
    pwm_offset = int(raw_force / 25.0 * 20.0) - 110;
  }
  else if(raw_force > 5.0)
  {
    pwm_offset = int(raw_force / 25.0 * 30.0) + 20;
  }
  return 1500 + pwm_offset;
}
