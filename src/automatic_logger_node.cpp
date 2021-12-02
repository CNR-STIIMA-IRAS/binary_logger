
/*********************************************************************
 *
 * Authors: Manuel Beschi (manuel.beschi@itia.cnr.it)
 *          Enrico Villagrossi (enrico.villagrossi@itia.cnr.it)
 *
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, National Research Council of Italy, Institute of Industrial Technologies and Automation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the National Research Council of Italy nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <nodelet/NodeletList.h>
#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
#include <boost/filesystem.hpp>
#include <boost/concept_check.hpp>
#include <std_srvs/Empty.h>


bool start_new=false;
bool stop_new=false;

bool stop_log_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  stop_new=true;
  return true;
}



bool start_log_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  start_new=true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "automatic_logger_start");
  ros::NodeHandle nh;

  std::string manager_name;
  if (!nh.getParam("binary_logger/manager_name", manager_name))
  {
    ROS_ERROR("binary_logger/manager_name NOT FOUND");
    return -1;
  }
  start_new=false;
  stop_new=false;


  ros::ServiceClient list_logger_srv = nh.serviceClient<nodelet::NodeletList>(manager_name+"/list");
  ros::ServiceClient load_logger_srv = nh.serviceClient<nodelet::NodeletLoad>(manager_name+"/load_nodelet");
  ros::ServiceClient unload_logger_srv = nh.serviceClient<nodelet::NodeletUnload>(manager_name+"/unload_nodelet");

  if (!list_logger_srv.waitForExistence(ros::Duration(5)))
  {
    ROS_ERROR("Manager '%s' is not found", manager_name.c_str());
    return -1;
  }
  ros::ServiceServer start_log = nh.advertiseService("start_log",start_log_cb);
  ros::ServiceServer stop_log  = nh.advertiseService("stop_log", stop_log_cb);
    nodelet::NodeletList nodelet_list;
  nodelet::NodeletLoad nodelet_load;
  nodelet::NodeletUnload nodelet_unload;

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.001).sleep();

    if (stop_new)
    {
      stop_new=false;
      std::vector<std::string> unstoppable_nodelets;
      if (!nh.getParam("binary_logger/unstoppable_nodelets", unstoppable_nodelets))
      {
        ROS_ERROR("binary_logger/unstoppable_nodelets NOT FOUND");
        return false;
      }

      list_logger_srv.call(nodelet_list);
      if (nodelet_list.response.nodelets.size()>0)
        ROS_INFO("%zu logger threads will be stopped", nodelet_list.response.nodelets.size());

      for (int idx = 0;idx < nodelet_list.response.nodelets.size();idx++)
      {
        bool do_stop = true;
        for (int idx2 =0; idx2 < unstoppable_nodelets.size(); idx2++)
        {
          if ( !nodelet_list.response.nodelets.at(idx).compare(unstoppable_nodelets.at(idx2) ) )
            do_stop=false;
        }

        if (do_stop)
        {
          nodelet_unload.request.name = nodelet_list.response.nodelets.at(idx);
          unload_logger_srv.call(nodelet_unload);
          if (!nodelet_unload.response.success)
            ROS_ERROR("Unloading of '%s' unsuccessfull", nodelet_unload.request.name.c_str());
        }
      }

    }

    if (start_new)
    {

      start_new=false;
      std::string test_name;
      if (!nh.getParam("binary_logger/test_name", test_name))
      {
        ROS_ERROR("binary_logger/test_name NOT FOUND");
        return false;
      }

      bool append_time;
      if (!nh.getParam("binary_logger/append_time",append_time))
        append_time=false;


      if (append_time)
      {
        ros::Time t0=ros::Time::now();
        test_name+="_"+std::to_string(t0.toSec());
        ROS_INFO("TEST NAME = %s",test_name.c_str());
      }
      std::string test_path;
      if (!nh.getParam("binary_logger/test_path", test_path))
      {
        test_path=( boost::filesystem::current_path().string() );
        ROS_INFO("binary_logger/test_path NOT FOUND, select PWD = '%s'",test_path.c_str());
      }
      if (!test_path.compare(""))
      {
        test_path=( boost::filesystem::current_path().string() );
        ROS_INFO("binary_logger/test_path VOID, select PWD = '%s'",test_path.c_str());
      }


      std::vector<std::string> topic_type;
      if (!nh.getParam("binary_logger/topic_type", topic_type))
      {
        ROS_ERROR("binary_logger/topic_type NOT FOUND");
        return false;
      }

      for (int idx = 0;idx<topic_type.size();idx++)
      {
        std::vector<std::string> topic_names;
        std::vector<double> duration;
        std::vector<int> decimation;
        std::string nodelet_type;



        if (!nh.getParam("binary_logger/"+topic_type.at(idx)+"/topic_names", topic_names))
        {
          ROS_ERROR("binary_logger/%s/topic_names NOT FOUND", topic_type.at(idx).c_str());
          return false;
        }
        if (!nh.getParam("binary_logger/"+topic_type.at(idx)+"/duration", duration))
        {
          ROS_ERROR("binary_logger/%s/duration NOT FOUND", topic_type.at(idx).c_str());
          return false;
        }
        if (!nh.getParam("binary_logger/"+topic_type.at(idx)+"/decimation", decimation))
        {
          ROS_ERROR("binary_logger/%s/decimation NOT FOUND", topic_type.at(idx).c_str());
          return false;
        }
        nodelet_type = "itia/"+topic_type.at(idx)+"BinaryLogger";

        for (int iT = 0;iT<topic_names.size();iT++)
        {
          nodelet_load.request.name = topic_names.at(iT);
          nodelet_load.request.type = nodelet_type;
          nodelet_load.request.my_argv.resize(4);
          nodelet_load.request.my_argv.at(1) = "/"+topic_names.at(iT);
          std::replace( topic_names.at(iT).begin(), topic_names.at(iT).end(), '/', '_');
          nodelet_load.request.my_argv.at(0) = test_path+"/"+test_name+"_"+topic_type.at(idx)+"_"+topic_names.at(iT);
          nodelet_load.request.my_argv.at(2) = std::to_string(duration.at(iT));
          nodelet_load.request.my_argv.at(3) = std::to_string(decimation.at(iT));
          load_logger_srv.call(nodelet_load);
          if (!nodelet_load.response.success)
          {
            ROS_ERROR("Fail calling '%s' logger ", topic_names.at(iT).c_str());
          }
        }
      }
    }
  }
  return 0;
}
