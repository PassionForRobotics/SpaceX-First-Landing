/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
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
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "atom_esp_joy/joydata.h"
// %EndTag(MSG_HEADER)%

#include <pthread.h> // exit test

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

#define SHORT_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
#define SHORT_TO_BINARY(byte)  \
  (byte & 0x8000 ? '1' : '0'), \
  (byte & 0x4000 ? '1' : '0'), \
  (byte & 0x2000 ? '1' : '0'), \
  (byte & 0x1000 ? '1' : '0'), \
  (byte & 0x0800 ? '1' : '0'), \
  (byte & 0x0400 ? '1' : '0'), \
  (byte & 0x0200 ? '1' : '0'), \
  (byte & 0x0100 ? '1' : '0'), \
  (byte & 0x0080 ? '1' : '0'), \
  (byte & 0x0040 ? '1' : '0'), \
  (byte & 0x0020 ? '1' : '0'), \
  (byte & 0x0010 ? '1' : '0'), \
  (byte & 0x0008 ? '1' : '0'), \
  (byte & 0x0004 ? '1' : '0'), \
  (byte & 0x0002 ? '1' : '0'), \
  (byte & 0x0001 ? '1' : '0') 
  
    
#include <iostream>
#include "atom_esp_joy/Extreme3DProService.hpp"
#include <chrono>

using JoystickLibrary::Extreme3DProService;
//using JoystickLibrary::Xbox360Service;
using JoystickLibrary::Extreme3DProButton;
using JoystickLibrary::POV;

Extreme3DProService& es = Extreme3DProService::GetInstance();


#include <sstream>

//#include "atom_esp_joy/JoyServiceRun.hpp"
//Extreme3DProServiceRun joyserrun(0);

atom_esp_joy::joydata joydata;


#include <signal.h>

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ROS_WARN("Joy is shuttng down ros");
  ros::shutdown();
}

void LogitechAxes(int id, int16_t *_x, int16_t *_y, int16_t *_z, int16_t *_s, int16_t *_btns)
{
//#define PRINT_JOYDATA
    int x, y, z, slider;
    std::map<Extreme3DProButton, bool> buttons;
    POV pov;

    if (!es.GetX(id, x))
        x = 0;
    if (!es.GetY(id, y))
        y = 0;
    if (!es.GetZRot(id, z))
        z = 0;
    if (!es.GetSlider(id, slider))
        slider = 0;

    *_x = (int16_t)x;
    *_y = (int16_t)y;
    *_z = (int16_t)z;
    *_s = (int16_t)slider;

    #if defined(PRINT_JOYDATA)
    	std::cout << "X: " << x << " | Y: " << y << " | Z: " << z << " | Slider: " << slider << " | Btns: [ ";
    #endif

    if( ! es.GetButtons(id, buttons))
    {

    }

    #define BASEBUTTONID (288)
    //unsigned char indx = 0;
    *_btns = 0;
    // _btns as per joydata.msg
    // get 4 LSBs for POV from _btns
    // right shift _btns
    // and get all the buttons from MSB

    //int btnarr[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 };
    //_btnarr must be of size 13 at least or an uint16

    for (auto pair : buttons)
    {
        //btnarr[((int)pair.first)-((int)BASEBUTTONID)] = pair.second;
        *_btns |= ((int)pair.second) << (((int)pair.first)-BASEBUTTONID) ;


    #if defined(PRINT_JOYDATA)
        std::cout <<"["<< static_cast<int>(pair.first)-((int)BASEBUTTONID)+1 << "]:" << pair.second ;
        //<< " (" << btnarr[((int)pair.first)-((int)BASEBUTTONID)] << "), ";
    #endif
    }


    #if defined(PRINT_JOYDATA)
    std::cout << "] | *_btns: " << *_btns << " | (";
    #endif

    *_btns <<= 4;

    if( ! es.GetPOV(id, pov))
    {

    }
    *_btns |= ((int)pov & 0x000F) ; // to check
    //std::cout << *_btns << ") | &F: "<< ((int)pov & 0x000F) << " | pov: " << (int)pov  << std::endl;

}

#include <stdlib.h>
#include <stdio.h>
void onExit(){
    ROS_WARN("EXIT");
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
   atexit(onExit);
   
// %Tag(INIT)%
  ros::init(argc, argv, "joy_node", ros::init_options::NoSigintHandler); 

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);
  
  printf("%c]0;%s%c", '\033', "JOY_ATOM_ESP", '\007');
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher joydata_pub = n.advertise<atom_esp_joy::joydata>("atom_joydata", 1000);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(1000);
// %EndTag(LOOP_RATE)%

    bool s = es.Initialize();
    ROS_INFO("Waiting for js plugin ... ");
    while (es.GetNumberConnected() < 1)
    {
	ROS_INFO_THROTTLE(5,"joy waiting ...");
    }
    ROS_INFO("RUNNING NOW...");

   //joyserrun.Start();
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int32_t count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    atom_esp_joy::joydata msg;
    int16_t x, y, z, s, btns; // please check the type 16/32 bits
    //joyserrun.GetAbsoluteAxesAndBtns(&x, &y, &z, &s, &btns);
    auto& a = es.GetIDs();
    if (a.size() <= 0)
    	continue;

    LogitechAxes(a[0], &x, &y, &z, &s, &btns); // ctrl+c is not working here ???
    msg.header.stamp = ros::Time::now(); // this was not working
    // msg.header.frame_id = "?"
    //msg.header.seq = count;
    msg.X = x;
    msg.Y = y;
    msg.Z = z;
    msg.S = s;
    msg.buttons = btns & (0x0000FFFF); // check diffrerent datatypes
    ROS_INFO_THROTTLE(10,"Pub Joy: x:%d, y:%d, z:%d, s:%d, b:" SHORT_TO_BINARY_PATTERN, msg.X, msg.Y, msg.Z, msg.S, SHORT_TO_BINARY(msg.buttons) );
    //std::stringstream ss;
    //ss << "hello world " << count;
    msg = msg;
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    //ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    joydata_pub.publish(msg);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
   loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }
  
  ROS_WARN("EXIT");
  pthread_exit(NULL);

  return 0;
}
// %EndTag(FULLTEXT)%
