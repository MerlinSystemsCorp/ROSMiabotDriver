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

#include <ros/ros.h>
#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/tcp.h>
#include <iostream>
#include <sstream>

using namespace std;

#define buflen 			1024

typedef void (*encoderCallback_t)(void * pParent, long leftWheel, long rightWheel);

// This class will encapsulate the actual API to the robot
// Should be easy to replace this class if the API changes ....
class MiabotAPI
{
public:
	int socketHandle;
	boost::thread receive_thread;
	bool isConnected;
	bool isClosingDown;
	char receiveBuffer[buflen];
	encoderCallback_t encoderCallback;
	void * pParent;
public:
	MiabotAPI();
	~MiabotAPI();

	void init();
	static void receiveThread(void * pParent);
	int open(string hostName, int portNum);
	int printFromSocket(int sd, char *buf);
	int close();
	void sendBuffer(string* packet);
	int sendMotorCommand(int leftVel, int rightVel);
	void resetOdometry();
	
};


