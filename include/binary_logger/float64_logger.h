
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

#ifndef __FLOAT64_LOGGER_NODELET__
#define __FLOAT64_LOGGER_NODELET__

#include <thread>
#include <mutex>
#include <iostream>
#include <fstream> 

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <boost/graph/graph_concepts.hpp>
#include <std_msgs/Float64.h>

namespace itia
{
namespace logger
{  

class Float64BinaryLogger : public nodelet::Nodelet
{
public:
  virtual void onInit();
  
protected:
  std::ofstream m_file;
  
  std::unique_ptr<char[]> m_buf;
  ros::Subscriber m_sub;
  
  double m_duration;
  bool m_stop;
  
  unsigned int m_decimation;
  
  unsigned int m_idx;
  
  std::string m_file_name;
  std::string m_topic_name;
  
  std::mutex m_stop_mtx;
  std::thread m_log_thread;
  
  void log();
  void Callback(const std_msgs::Float64ConstPtr& msg);
  ~Float64BinaryLogger();
  
};


}
}

# endif