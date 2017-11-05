/*
 * 
 * @author Jessica Howard
 * @file add_two_ints_client.cpp
 * @brief Demonstration of how messages are received on ROS
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

#include <cstdlib>
#include "beginner_tutorials/AddTwoInts.h"
#include "ros/ros.h"
#include "ros/console.h"

/*
 * @brief determines if an input string is an integer
 */
bool legal_int(char *str) {
  char *iter = str;
  std::string s(str);
  bool decimal = false;
  bool letter = false;

  for (std::string::iterator it = s.begin(), end = s.end(); it != end; ++it) {
    if (!isdigit(*it)) {
      if (*it == '.')
        decimal = true;
      else
        letter = true;
    }
  }
  if (letter) {
    ROS_ERROR_STREAM(str << " is not a number.");
    return false;
  }

  if (decimal) {
    ROS_WARN_STREAM(str << " is not an integer and will be truncated.");
  }
  return true;
}

/*
 * @brief Client service for the addition service
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "add_two_ints_client");
  // Check for proper arguments
  if (argc != 3)   {
    ROS_INFO("usage: add_two_ints_client X Y");
    ROS_DEBUG_STREAM_COND(argc != 3, "argc is " << argc);
    return 1;
  }

  ros::NodeHandle n;
  // Creates the client for the addition service
  ros::ServiceClient client =
    n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  ROS_DEBUG_STREAM("argv[1]: " << argv[1] << ", argv[2]: " <<argv[2]);

  // Stores the user's two integers into the request variable
  if (legal_int(argv[1]) && legal_int(argv[2])) {
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);

    // Call the server to add
    if (client.call(srv))   {
      ROS_INFO("Sum: %ld", static_cast<int64_t>(srv.response.sum));
    } else {
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }

    return 0;
  } else {
    ROS_FATAL("Arguments must be numeric.");
    return 1;
  }
}
