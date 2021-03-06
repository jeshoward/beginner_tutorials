/*
 * 
 * @author Jessica Howard
 * @file add_two_ints_server.cpp
 * @brief Server for addition service
 *
 * 
 * @copyright Copyright (C) 2017, Jessica Howard
 * @license 3-Clause BSD License
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "ros/console.h"
#include "beginner_tutorials/AddTwoInts.h"

/*
 * @brief Addition of two integer parameters
 * @param &req request for two integers, a and b
 * @param &res response for the sum of a and b
 */
bool add(beginner_tutorials::AddTwoInts::Request &req,
         beginner_tutorials::AddTwoInts::Response &res) {
  // Stores the sum of a and b into the result
  res.sum = req.a + req.b;
  ROS_DEBUG_STREAM("req.a: " << req.a << ", req.b: " << req.b);
  // publishes messages showing what was requested and what was returned
  ROS_INFO("request: x=%ld, y=%ld", static_cast<int64_t>(req.a),
                                    static_cast<int64_t>(req.b));
  ROS_INFO("sending back response: [%ld]", static_cast<int64_t>(res.sum));
  return true;
}

int main(int argc, char **argv) {
  // Initialize the server
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  // Create and advertise the service over ROS
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
