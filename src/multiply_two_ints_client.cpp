/*
 * 
 * @author Jessica Howard
 * @file multiply_two_ints_client.cpp
 * @brief Client application for multiplication of two ints service
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
#include "beginner_tutorials/MultiplyTwoInts.h"
#include "ros/ros.h"

/*
 * @brief Client service for the multiplication service
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "multiply_two_ints_client");
  /* Check for proper arguments */
  if (argc != 3)   {
    ROS_INFO("usage: multiply_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  /* Creates the client for the multiplication service */
  ros::ServiceClient client =
    n.serviceClient<beginner_tutorials::MultiplyTwoInts>("multiply_two_ints");
  beginner_tutorials::MultiplyTwoInts srv;
  /* Stores the user's two integers into the request variable */
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  /* Call the server to multiply */
  if (client.call(srv))   {
    ROS_INFO("Product: %ld", static_cast<int64_t>(srv.response.product));
  } else {
    ROS_ERROR("Failed to call service multiply_two_ints");
    return 1;
  }

  return 0;
}
