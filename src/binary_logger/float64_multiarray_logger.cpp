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

#include <binary_logger/float64_multiarray_logger.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::logger::Float64MultiArrayBinaryLogger, nodelet::Nodelet) 


namespace itia
{
namespace logger
{

void Float64MultiArrayBinaryLogger::onInit()
{
  ROS_DEBUG("Pose Stamped Logger nodelet");
  
  std::vector<std::string> args = getMyArgv();
  
  std::string prefix = "";
  m_topic_name = "pose";
  m_file_name = "pose";

  if (args.size() > 0)
  {
    if (args.at(0).size()>0)
    {
      m_file_name = args.at(0);
      prefix = args.at(0)+"_";  
    }
  }
  if (args.size() > 1)
  {
    if (args.at(0).size()>0)
    {
      m_topic_name = args.at(1);
    }
  }
  
  
  ROS_DEBUG("Starting logging topic '%s' in file named '%s'",  m_topic_name.c_str(), m_file_name.c_str()   );
  
  m_duration = 100;
  if (args.size() > 2)
    m_duration = std::stod(args.at(2));
  
  m_decimation = 1;
  
  m_idx = 0;
  
  if (args.size()>3)
    m_decimation = std::stod(args.at(3));


  
  m_file.open(m_file_name+".bin",std::ios::out | std::ios::binary);
  
  m_stop = false;
  m_log_thread  = std::thread(&itia::logger::Float64MultiArrayBinaryLogger::log, this);
  
  const size_t bufsize = 1024 * 1024;
  m_buf.reset(new char[bufsize]);
  m_file.rdbuf()->pubsetbuf(m_buf.get(), bufsize);
  
};

Float64MultiArrayBinaryLogger::~Float64MultiArrayBinaryLogger()
{
  ROS_DEBUG("Shutting down logging topic '%s' in file named '%s'",  m_topic_name.c_str(), m_file_name.c_str()   );
  m_stop_mtx.lock();
  m_stop = true;
  m_stop_mtx.unlock();
  m_log_thread.join(); 
  ROS_DEBUG("logging topic '%s' in file named '%s' SHUT DOWN",  m_topic_name.c_str(), m_file_name.c_str()   );
  m_file.close();
};


void Float64MultiArrayBinaryLogger::log()
{
  double read_rate = 1000;
  ros::Rate lp(read_rate);
  bool stop = false;
  ROS_DEBUG("Starting THREAD: logging topic '%s' in file named '%s'",  m_topic_name.c_str(), m_file_name.c_str()   );
  ros::Subscriber m_js_sub = getNodeHandle().subscribe<std_msgs::Float64MultiArray>(m_topic_name,100, &itia::logger::Float64MultiArrayBinaryLogger::Callback, this);
  
  ros::Time t0 = ros::Time::now();
  
  while (true)
  {
    m_stop_mtx.lock();
    stop = m_stop;
    m_stop_mtx.unlock();
  
    if (( ros::Time::now()-t0).toSec() >m_duration )
    {
      ROS_INFO("Log of topic '%s' in file named '%s' COMPLETE",  m_topic_name.c_str(), m_file_name.c_str()   );
      m_stop_mtx.lock();
      stop = m_stop = true;
      m_stop_mtx.unlock();
    }
    if (stop)
      break;
    
    lp.sleep();
  }
  m_js_sub.shutdown();
  ROS_DEBUG("End of THREAD: logging topic '%s' in file named '%s'",  m_topic_name.c_str(), m_file_name.c_str()   );
};


void Float64MultiArrayBinaryLogger::Callback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  if (m_decimation)
  {
    if (m_idx >= (m_decimation-1))
    {
      std::vector<double> data(msg->data.size());
      data = msg->data;
      double time = ros::Time::now().toSec();
      
      m_file.write((char*) &time, sizeof(double));
      m_file.write((char*) &data[0], data.size() * sizeof(double));
      m_idx = 0;
    }
    else
      m_idx++;
  }
};


}
}