/*
 * MiabotPro Robot Driver
 * Copyright (c) 2011, Merlin Systems Corp. Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANISATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

/**

@mainpage

@htmlinclude manifest.html

@b miabot is a driver for the Merlin Systems Corp. Ltd Miabot Pro mobile robot, available from
<a href="http://www.merlinsystemscorp.co.uk">Merlin Systems Corp. Ltd</a>.


<hr>

@section usage Usage
@verbatim
$ miabot [standard ROS args]
@endverbatim

@par Example

@verbatim
$ miabot
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "cmd_vel"/Twist : velocity commands to differentially drive the robot.

Publishes to (name / type):
- @b "odom"/Odometry : odometry data from the robot.

<hr>

@section parameters ROS parameters

- @b host_name (string) : Host that the robot base is connected to e.g. 127.0.0.1
- @b port_name (string) : Port that the robot base is connected to e.g. 3000
- @b max_trans_vel (string) : Maximum translational velocity. Default: 0.5 m/s.
- @b max_rot_vel (string) : Maximum rotational velocity. Default: 100 deg/s.

- @b max_motor_speed (string) : Maximum velocity specified in motor units. Default 128
- @b diff_conv_factor (string) : ratio govern steering rate. default 0.1
- @b motor_scale_factor (string) : conversion between m/s and the robot command input (e.g. clicks/s). default 1000 
- @b encoder_conversion_factor(string) : conversion between incoming tocks and physical distance m. default 2500 (see below) 

- @b The encoders give 512 pulses per revolution, the diameter of a
- @b wheel is 52mm, and the robot has a gear of 1:8. So distance per pulse is π·52mm/512/8 = 0.04
- @b mm/pulse.

- @b wheelbase_size (string): physical distance between wheels in m. Default 0.065m

e.g. 

.04mm per 1 pulse
4mm per 100 pulses
4cm per 1000 pulses
4m per 10000 pulses
1m per 2500 pulses




@section Miabot input commands

- @bMiabot Pro command set
- @b   char routine name      descr
- @b   ---- ------------      ------
- @b{basic operations}
- @b   [t]  test              -> robot id string, like "<test#01>"
- @b   [Q]  version 	   -> version string, like "<Miabot Pro OS 2.3>"
- @b(  [#]  dipSwitches       - not included at present )
- @b   [`]  reboot            simulate power-on
- @b   [!]  flashLeds         do led test sequence (like startup)
- @b
- @b{speed control commands}
- @b   [-]  setSpeed          [-<byteleft><byteright>] set wheel speeds, binary scaled
- @b   [=]  setSpeedDec       [=<number> <number>] set wheels speeds, decimal
- @b   [s]  stop              like [=0,0])
- @b
- @b{wheel encoders access}
- @b   [:]  posOut            read encoder values => "<: llllllllll:rrrrrrrrrr>" (signed long decimal)
- @b                          [:=nn] sets repeat-rate at nn mSecs
- @b   [;]  posSet            write encoder values [;ll,rr] (signed long decimal)
- @b
- @b{step (fixed distance) movements}
- @b   [^]  stepForward       step forward (v^ distance)
- @b   [v]  stepBackward      step backward  (v^ distance)
- @b   [<]  stepLeft          rotate left (<> distance)
- @b   [>]  stepRight         rotate right (<> distance)
- @b   [d]  setDist	   get/set step distances
- @b                          [d] show v^ and <> step distances
- @b                          [d?<num>] set - ? is one of {v^<>}
- @b 
- @b   [x]  setRate           get/set step speeds
- @b                          [x] show v^ and <> step speeds
- @b                          [x?<num>] set - ? is one of {v^<>}
- @b
- @b   [?]  stopCheck	   step complete check -> <?=0>(stopped) -or- <?=1>(still moving)
- @b
- @b{"bigtrack" specified-distance movements}
- @b   [m]  rotateLeft        [m<byte>] rotate left by byte distance ([x<] speed)
- @b   [n]  rotateright       [n<byte>] rotate right by byte distance ([x<] speed)
- @b   [o]  forward           [o<byte>] move forward by byte distance ([xv] speed)
- @b   [p]  backward          [p<byte>] rotate right by byte distance ([xv] speed)
- @b  
- @b{parameter access}
- @b   [.] param              parameter access -
- @b                          [.<name>] read param
- @b                          [.<name>=<value>] write param
- @b                          [.] show all -> ' '=next, '/'=quit, '.'=rest
- @b                          [.=] reset all -> 'Y'=reset, other=quit
- @b
- @b{command sequence control - see also Miabot Pro Sequences}
- @b   [$]  clearSeq          erase stored sequence
- @b   [~]  doSeq             start sequence running 
- @b   [+]  addSeqCommand     [+<command...>] add next sequence command
- @b   [w]  waitTime          [w<num>] delay before next command
- @b(  [#]  endSeq          stop sequence - not used at present )
- @b
- @b{radio setup - see also Miabot Pro Radio-board Controls)}
- @b   [*]   setRadioName       [*<string>] change Bluetooth module name (normally "mid###")
- @b   [&]   setRadioNumber     [&<string>] change Bluetooth pass-id (normally "1234")
- @b   [#]   sendRadioCommand   [#<string>] send radio command string 
- @b                              ( .#T and .#rs also used )
- @b   [@]   Radcon_SetidCmd    [@xx:xx:xx:xx:xx:xx] set auto-connect id
- @b                              ( where 'xx' are 2-digit hex byte values )
- @b                              ( parameter setting .aEN=1 to enable use )
- @b
- @b{voltage inputs}
- @b   [V]   voltsSample      read expansion port voltages (with optional repeating)
- @b                          [V] read current channel
- @b                          [V{<num>}{=<num>}] read and set channel and/or repeat-rate :
- @b                            <num> = 0..7(single)  -or-  '-'(all 8)
- @b                            =<num> is rate
- @b                            N.B. [V=0] or [V=] stops repeating  
- @b{I2c access}
- @b  [i]   I2C_Access        [i<XX>...] read or write bytes from I2C bus :
- @b                            <XX> is hex byte I2C address (upper seven bits = always even value)
- @b                            [iXX?<num>]  read a number of bytes
- @b                            [iXX=<xx> <xx> ...]  write a number of bytes
- @b
- @b{servo pulsewidth output}
- @b  [P]   Servo_Access      [P<num> <num> ...] set n pulsewidth outputs 0..255 (up to .ebP)
- @b
- @b{I2C remote serial (for MSCcam operation)}
- @b  [T]   Serrmt_SimTx      [T <hex> <hex> ...]  send serial bytes via i2c link
- @b  [R]   Serrmt_SimRx      [R <hex> <hex> ...]  simulate received serial data from i2c
- @b  [X]   Serrmt_TerminalMode    switch to "terminal" mode
- @b  [G]   Serrmt_ReadLog    output + clear event log
- @b  [W]   Serrmt_ReadHWM    read+reset rx- and tx-buffer statistics
- @b
- @b{sonar fast-scan command}
- @b  [S]   Sonar_DoScan      do sonar scan (see control params .RG1-4, .rNB etc)
- @b
- @b
- @section Miabot input commands
- @b   code ident              .name  default    description
- @b   ----------              -----  -------    -----------
- @b {speed ramp controls}
- @b   V_RampTicks             .rT    10         //ramp update-rate (ticks)
- @b   V_RampIncrement         .rI    10         //ramp speed increment
         
- @b {pid calc controls - see also  PID Controls}
- @b  V_PidTickrate           .pT    5          //pid update-rate (ticks)
- @b  V_PidSpeedgain          .pVg   4          //speed-gain = ticks-to-speed scaling (-> about 1000 max)
- @b  V_PidOpOffs             .poK   0          //open-loop speed minimum non-zero output
- @b  V_PidOpGain             .poG   3          //open-loop speed gain
- @b  V_PidDownshift          .p>>   7          //pid-calc scaling (shift)
- @b  V_PidP                  .pP    50         //P-term gain
- @b  V_PidI                  .pI    10         //I-term gain
- @b  V_PidD                  .pD    0          //D-term gain
- @b  V_PidMaxIerr            .pMI   512L*8/2   //anti-windup I max (half a wheel turn)

- @b {position / deceleration controls}
- @b  V_PosSpeedScale         .xS    2          //dist_error-to-speed scaling (*PidDownShift)
- @b  V_PosTolerance          .xT    3          //min dist to switch-off
- @b  V_PosMaxdist            .xM    50000      //max dist for slowup-calc (avoids overflow errors!)
- @b
- @b {speed commands timeout}
- @b  V_SpeedTimeout          .vTO   0          //off by default (else mSecs)
- @b         
- @b {squeal reduction}
- @b  V_StopMinDist           .qX    2          //movement threshold level for power-off
- @b  V_StopTimeout           .qT    10         //no-movement timeout period
- @b         
- @b {miabot-like scaled (byte data) commands}
- @b   V_ByteScaleSpeed        .bV    0x0800     //256*8 = 1024 max speed
- @b   V_ByteScaleDistMove     .b^    5100
- @b   V_ByteScaleDistTurn     .b<    4300
- @b   V_ByteScaleSteps        .bQ    0          //off for ^v<> moves  by default
- @b         
- @b [i]: {i2c retry controls}
- @b   V_i2cRetries            .iN    0          //times to retry failed access (=none)
- @b   V_i2cRetryTime          .iT    0          //mSecs between retries

- @b {echo control}
- @b   V_bEcho                 .eE    0          //enable echo for all commands
- @b
- @b {serrmt transmit controls (tx = to i2c bus)}
- @b   V_i2s_nOwnAddress       .sAD   0xA2       //own bus address (receive control, really)
- @b   V_i2s_nContactAddress   .sAT   0xA0       //output target address (=MSCcam standard)
- @b   V_i2s_nTxBuffTrig       .sNB   4          //buffer threshold transmit trigger
- @b   V_i2s_nTxTimeTrig       .sNT   2          //timeout transmit trigger
- @b   V_i2s_bTxCharEna        .sCE   0          //enable break-char trigger
- @b   V_i2s_cTxCharTrig       .sCC   '\r'       //break-char
- @b {serrmt <R..> receive controls (rx = from i2c bus)}
- @b  V_i2s_nRxBuffTrig       .sN    4          //buffer threshold output trigger
- @b   V_i2s_nRxTimeTrig       .sT    2          //timeout output trigger
- @b   V_i2s_bRxCharEna        .sE    0          //enable break-char
- @b   V_i2s_cRxCharTrig       .sC    '\r'       //break-char

- @b [P]: {servo pulsewidth outputs}
- @b   V_nPwmOutputBits        .ebP   0          //output bits to use (mask: 0 - 31)
- @b   V_nPwmStartDelay        .PDT   100        //0.1sec start delay to avoid oscillations
- @b
- @b [S]: {sonar scan}
- @b   V_nSonarGroup1          .rG1   0xFF       //Ping group1 (default =all 8 at once)
- @b   V_nSonarGroup2          .rG2   0x00       // (= no 2nd group)
- @b   V_nSonarGroup3          .rG3   0x00       // (= no 3rd group)
- @b   V_nSonarGroup4          .rG4   0x00       // (= no 4th group)
- @b   V_nSonarRetryMsecs      .rRT   4          //retry milliseconds
- @b   V_nSonarRetries         .rRN   20         //retry count
- @b   V_nSonarFirstbyte       .rB1   1          //readback address (=light sensor)
- @b   V_nSonarReadbytes       .rNB   5          //bytes to read (=light + 2 echoes)
- @b   
- @b [@]: {auto-connect settings}
- @b   V_Radcon_bAutoConnect   .aEN   0          //enable auto-connect
- @b   V_Radcon_nStartMsecs    .aT0   5000       //delay to first try
- @b   V_Radcon_nRepMsecs      .aT1   10000      //delay between tries
- @b   V_Radcon_nID0           .ai0   0x00       //bluetooth address bytes...
- @b   V_Radcon_nID1           .ai1   0x00
- @b   V_Radcon_nID2           .ai2   0x00
- @b   V_Radcon_nID3           .ai3   0x00
- @b   V_Radcon_nID4           .ai4   0x00
- @b   V_Radcon_nID5           .ai5   0x00

- @b {main leds enable (2 bits = comms,power)
- @b   V_nLedsEna              .lnE   3          //both on by default

- @b (radio - see also Miabot Pro Radio-board Controls)
- @b   V_radio_tWait           .#T    2000       //time allowed for radio-commands
- @b   V_radio_bReset          .#rs   1          //set radio mode on boot (+ cancel it)
- @b
- @b Miabot Pro user Manual - http://www.merlinautomation.co.uk/resources/merlin_private/all_products/robots/miabot_pros/miabot_pro_BT/Miabot%20Pro%20User%20Manual%20v1.3.pdf
- @b

 **/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <miabot/miabotapi.hpp>

#include <iostream>
#include <sstream>

using namespace std;

// Operation modes for for this driver
#define LIVE_MODE 		1	// Odometry is received from the physical robot
#define SIMULATION_MODE		2	// Odometry is estimated
#define PLAYER_MODE_RECEIVE	4	// Odometry is received from player (which has a hard link to the robot) 
#define PLAYER_MODE_SEND	8	// Velocity commands can also be sent to the Player driver

#define MAX_BUFFER		1024


class MerlinMiabotNode
{

public:
	ros::NodeHandle rosNodeHandle;

	ros::Publisher odomPublisher;
	ros::Subscriber cmdVelSubscriber;
  	tf::TransformBroadcaster odomBroadcaster;

	// Default robot device settings
	string host;
	string port;
	string maxTransVel;
	string maxRotVel;
	string operationMode;		// could be 0=live, 1 = simulated, 2 = player
	string tfPrefix;
	string maxMotorSpeed;
	string diffConvFactor;
	string motorScaleFactor;
	string encoderConversionFactor;
	string wheelbaseSize;
private:
	// target velocity inputs received from ROS velocity subscription message
	double targetTranslationalVelocity;
	double targetRotationalVelocity;
	long lastLeftWheel;
	long lastRightWheel;

	// Current robot pose information
	double currentPoseX;
	double currentPoseY;
	double currentPoseYaw;
	double currentVelX;
	double currentVelY;
	double currentVelYaw;		
	bool firstTime;			// global used to detect first time encoder input

	string odomFrameID;
	ros::Time current_time;
	ros::Time last_time;

	MiabotAPI miabotAPI;		// Object encapsulates API with the robot

public:

	MerlinMiabotNode()
	{
		setupDefaultParameters();

		// set the callback entry point for encoder data
		miabotAPI.pParent = (void*) this;
		miabotAPI.encoderCallback = estimateCurrentPoseFromEncoders;

		// initialise time
		current_time = ros::Time::now();
		last_time = ros::Time::now();


	}

	~MerlinMiabotNode()
	{
	}

	int resetOdometry()
	{
		bool firstTime = true;
		currentPoseX =0.0;
		currentPoseY = 0.0;
		currentPoseYaw = 0.0;
		lastLeftWheel = -1;
		lastRightWheel = -1;

		miabotAPI.resetOdometry();

		return 0;
	}

	int displayPose()
	{
		ROS_INFO("Position: x: %f , y: %f , yaw: %f", currentPoseX, currentPoseY, currentPoseYaw);
	}

	int setupDefaultParameters()
	{
		
		host = "127.0.0.1";
		port = "3000";
		maxTransVel = "0.5";
		maxRotVel = "100";
		operationMode ="1";		// See defines above
		maxMotorSpeed = "128";
		diffConvFactor = "0.1";
		motorScaleFactor = "1000";
		encoderConversionFactor = "25000";
		wheelbaseSize = ".066"; // 6.6cm

		lastLeftWheel = -1;
		lastRightWheel = -1;

		// Read default driver parameters from ROS parameter handler
		ros::NodeHandle private_nh("~");

		private_nh.getParam("host_name", host);
		private_nh.getParam("port_name", port);
		private_nh.getParam("max_trans_vel", maxTransVel);
		private_nh.getParam("max_rot_vel", maxRotVel);
		private_nh.getParam("operation_mode", operationMode);
		private_nh.getParam("max_motor_speed", maxMotorSpeed);
		private_nh.getParam("diff_conv_factor",diffConvFactor);
		private_nh.getParam("motor_scale_factor",motorScaleFactor);
		private_nh.getParam("encoder_conversion_factor",encoderConversionFactor);
		private_nh.getParam("wheelbase_size",wheelbaseSize);

		private_nh.param("odometry_frame_id", odomFrameID, std::string("odom"));

		//tfPrefix = tf::getPrefixParam(rosNodeHandle);

		return 0;
	}

	int start()
	{
		// if we are in 'LIVE_MODE' attempt to initiate connection with robot
		if ((atoi(operationMode.c_str()) & LIVE_MODE) == LIVE_MODE)
		{ 
			ROS_INFO("Connecting to robot on host %s:%s",host.c_str(),port.c_str());

			int result = miabotAPI.open(host,atoi(port.c_str()));

			if (result == -1)
			{
				ROS_ERROR("Unable to open socket: %s:%s",host.c_str(),port.c_str());
				exit(-1);
			}
			else
			{
				ROS_INFO("Success::MiabotAPI connected with robot");
			}

		}

		resetOdometry();

		//create publisher for odometry information
		odomPublisher = rosNodeHandle.advertise<nav_msgs::Odometry>("odom", 1);

		// Create subscriber for velocity information
		cmdVelSubscriber = rosNodeHandle.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&MerlinMiabotNode::cmdVelReceived, this, _1));

		return(0);
	}

	int stop()
	{
		int status = 0;

		//rosNodeHandle.stop();
		status = miabotAPI.close();

		return(status);
	}

	// Callback handler for the input velocity message
	void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
	{
		// These are the requested velocity inputs
		targetTranslationalVelocity = cmd_vel->linear.x;
		targetRotationalVelocity = cmd_vel->angular.z;

		if ((atoi(operationMode.c_str()) & LIVE_MODE) == LIVE_MODE)
			sendMotorCommand();
	}


	// Set the velocity for each wheel and send to the robot
	// based upon the incoming twist message request
	void sendMotorCommand()
	{
		// convert targetTranslationalVelocity and targetRotationalVelocity into wheelspeeds
    		double rotationalTerm = (M_PI/180.0) * targetRotationalVelocity / strtod(diffConvFactor.c_str(),NULL);

		//ROS_INFO("Rotational_term: %f\n", rotationalTerm);
    		double leftVel = (targetTranslationalVelocity - rotationalTerm);
    		double rightVel = (targetTranslationalVelocity + rotationalTerm);

		double maxSpeed = strtod(maxMotorSpeed.c_str(),NULL); 
		
		// Apply wheel speed bounds
		if(fabs(leftVel) > maxSpeed)
		{
			if(leftVel > 0)
			{
				leftVel = maxSpeed;
				rightVel *= maxSpeed/leftVel;
			}
			else
			{
				leftVel = -maxSpeed;
				rightVel *= -maxSpeed/leftVel;
			}
		}

		if(fabs(rightVel) > maxSpeed)
		{
			if(rightVel > 0)
			{
				rightVel = maxSpeed;
				leftVel *= maxSpeed/rightVel;
			}
			else
			{
				rightVel = -maxSpeed;
				leftVel *= -maxSpeed/rightVel;
			}
		}

    		//ROS_INFO("Speeds in ticks: %f %f\n", leftVel, rightVel);

		miabotAPI.sendMotorCommand((int) (leftVel*atol(motorScaleFactor.c_str())), (int)(rightVel*atol(motorScaleFactor.c_str())));
	}		

	//
	// Simulation Mode:
	//			Given the current target input velocities and the current
	//			robot velocity work out the new achieveable velocity.
	//			Based around max_trans_vel & max_rot_vel parameters.
	// Live Mode:
	//			Since we are receiving actual odometry from the pysical robot we
	//			can just calculate the actual velocity information.
	void estimateCurrentVelocity()
	{
		// Use the incoming twist message to simulate the required odometry;
		currentVelX = cos(targetRotationalVelocity)*targetTranslationalVelocity;
		currentVelY = sin(targetRotationalVelocity)*targetTranslationalVelocity;
		currentVelYaw = targetRotationalVelocity;

		targetTranslationalVelocity=0;
		targetRotationalVelocity=0;
	}

	// Uses the estimated velocity information to calculate a new pose for the robot	
	void estimateCurrentPose(double dt)
	{

		// TODO: If in live mode we need to integrate the actual odometry to calculate an actual pose (dead reckoning)
		double delta_x = (currentVelX * cos(currentPoseYaw) - currentVelY * sin(currentPoseYaw)) * dt;
		double delta_y = (currentVelX * sin(currentPoseYaw) + currentVelY * cos(currentPoseYaw)) * dt;
		double delta_th = currentVelYaw * dt;

		currentPoseX += delta_x;
		currentPoseY += delta_y;
		currentPoseYaw += delta_th;
	}

	// Uses the estimated velocity information to calculate a new pose for the robot	
	static void estimateCurrentPoseFromEncoders(void * pParent, long leftWheel, long rightWheel)
	{
		MerlinMiabotNode * pThis = (MerlinMiabotNode*)pParent;

		if (!pThis->firstTime)
		{
			//ROS_INFO("Encoder Callback");


			//ROS_INFO("Encoder Callback pThis");

			pThis->current_time = ros::Time::now();

			double dt = (pThis->current_time - pThis->last_time).toSec();
		
			double eFactor = strtod(pThis->encoderConversionFactor.c_str(),NULL);
			double wheelbase = strtod(pThis->wheelbaseSize.c_str(),NULL);

			double rightWheelm = (leftWheel-pThis->lastLeftWheel)/eFactor;   // convert into m
			double leftWheelm = (rightWheel-pThis->lastRightWheel)/eFactor; // convert into m
			double deltaS = (rightWheelm+leftWheelm)/2;
			double deltaYaw = (leftWheelm-rightWheelm)/wheelbase;

			pThis->currentPoseX += deltaS*cos(pThis->currentPoseYaw);
			pThis->currentPoseY += deltaS*sin(pThis->currentPoseYaw);

			pThis->currentPoseYaw += deltaYaw;

			pThis->publishOdometryMessage();
		}

		pThis->lastLeftWheel = leftWheel;
		pThis->lastRightWheel = rightWheel;

		pThis->last_time = pThis->current_time;
		pThis->firstTime = false;
	}

	// Will receive odometry updates from the physical device and/or a player interface for example
	void doUpdate()
	{
		// TODO: in Live mode updates are driven from incoming odometry
		// TODO: in Simulation updates are driven from incoming twist messages 
		current_time = ros::Time::now();

		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time - last_time).toSec();

		estimateCurrentVelocity();
			
		estimateCurrentPose(dt);

		publishOdometryMessage();

		last_time = current_time;
	}


	int publishOdometryMessage()
	{
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(currentPoseYaw);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = currentPoseX;
		odom_trans.transform.translation.y = currentPoseY;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odomBroadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = currentPoseX;
		odom.pose.pose.position.y = currentPoseY;
		odom.pose.pose.position.z = 0.0; 
		odom.pose.pose.orientation = odom_quat;

		//publish the message
		odomPublisher.publish(odom);

		return 0;
	}


	// send the received buffer to the robot
	void sendBuffer(string * buffer)
	{
		miabotAPI.sendBuffer(buffer);
	}

};

void spinThread()
{
	ros::spin();

	ROS_INFO("Spin Thread Closed - Press <Enter> to complete exit.");

}


int main(int argc, char** argv)
{
	char buffer[MAX_BUFFER] ={0};

	ros::init(argc, argv, "MerlinMiabotDriver");

	MerlinMiabotNode merlinMiabotNode;

	ros::NodeHandle rosNode;
	boost::thread spin_thread = boost::thread(boost::bind(&spinThread));
	
	// Start up the robot
	if (merlinMiabotNode.start() != 0)
	{
		exit(-1);
	}

	ros::Rate r(1.0);

	/////////////////////////////////////////////////////////////////
	// Main loop; Scan the robot odometry or receive messages from 
	//            Player.
	while (rosNode.ok())
	{
		//merlinMiabotNode.doUpdate();
		
		if (feof(stdin)==0)
		{
			fgets(buffer, MAX_BUFFER, stdin);   /* remember, fgets appends the newline */

			if ((buffer[0]=='[') && (buffer[strlen(buffer)-2]==']'))
			{
				//ROS_INFO("Miabot Command: %s",buffer);
				merlinMiabotNode.sendBuffer(new string(buffer));
			}
			else
			{
				string str(buffer);
				if (str.compare(0,8,"position") == 0)
				{
					merlinMiabotNode.displayPose();
				}
				else if (str.compare(0,5,"reset") == 0)
				{
					merlinMiabotNode.resetOdometry();
					merlinMiabotNode.displayPose();
				}
			}

		}

		r.sleep();
	}

	ROS_INFO("Shutting down...");

	// Stop the robot
	merlinMiabotNode.stop();

	//ROS_INFO("Waiting for spin thread to join...");
	spin_thread.join();

	//ROS_INFO("Waiting for robot thread to join...");
	//robot_thread.join();

	ROS_INFO("Clean Exit.");
	return(0);
}

