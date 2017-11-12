/*
 * 
 * @author Jessica Howard
 * @file test/talker_test.cpp
 * @brief Unit testing file for talker.cpp
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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

/**
 * @brief callback helper pointer
 */
struct Helper {
  Helper() : count(0) { }

  void cb(const std_msgs::String::ConstPtr& msg) {
    ++count;
  }
  uint32_t count;
};

/**
 * @brief tests existence of publisher and subscriber
 */
TEST(TestSuite, talkerTest) {
  ros::NodeHandle nh;
  Helper h;
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber sub = nh.subscribe("chatter", 1000, &Helper::cb, &h);
  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
