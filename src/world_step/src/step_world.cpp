#include <condition_variable>
#include <math.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <world_step/step_world.h>

gazebo::transport::PublisherPtr pub;
gazebo::transport::SubscriberPtr sub;

int stepped;

std::mutex mtx;
std::condition_variable cv;
std::unique_lock<std::mutex> lck(mtx);

void world_steppedCB(ConstIntPtr &_msg) {
  stepped = 1;
  cv.notify_one();
}

//bool step(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
bool step(world_step::step_world::Request& request, world_step::step_world::Response& response) {
  // Publish the step message for the simulation.
  gazebo::msgs::WorldControl msg;
  msg.set_step(1);
  pub->Publish(msg);

  cv.wait(lck);
  lck.unlock();

  //while(!stepped) {
  //}
  stepped = 0;

  response.stepped = true;
  
  return true;
}

int main(int argc, char** argv)
{
  // Initialize the ROS Node (Client calls this service)
  ros::init(argc, argv, "step_world_server");
  ros::NodeHandle n;
  
  // Setup the Gazebo node that will step the simulation.
  gazebo::client::setup(argc,argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  sub = node->Subscribe("~/world_stepped", world_steppedCB); 
  stepped = 0;

  // Advertise the service as ready.
  ros::ServiceServer service = n.advertiseService("step_world", step);
 
  ros::spin();

  return 0;
}

