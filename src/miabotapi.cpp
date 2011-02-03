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

#include <miabot/miabotapi.hpp>



MiabotAPI::MiabotAPI()
{
	isConnected = false;
	isClosingDown = false;
	pParent = NULL;
	encoderCallback =NULL;
	

}

MiabotAPI::~MiabotAPI()
{
	socketHandle = -1;

	receive_thread.join();

	sleep(1);
}


void MiabotAPI::init()
{
	receive_thread = boost::thread(boost::bind(MiabotAPI::receiveThread,this));

	resetOdometry();
}


void MiabotAPI::sendBuffer(string * packet)
{

	write(socketHandle, packet->c_str(), packet->length());

}

void MiabotAPI::resetOdometry()
{
	if (isConnected)
	{
		sendBuffer(new string("[;0,0]"));
		sendBuffer(new string("[:=100]"));
	}
}

void MiabotAPI::receiveThread(void * pParent)
{
	MiabotAPI * pThis = (MiabotAPI *) pParent;

	ROS_INFO("MiabotAPI::receiveThread started.");

	while (!pThis->isClosingDown)
	{
		if (pThis->isConnected)
		{
			// Check the socket for input data
			if (pThis->printFromSocket(pThis->socketHandle, pThis->receiveBuffer) == 0) 
			{
				ROS_ERROR("MiabotAPI::receiveThread - error reading from socket (connect could have closed)");
				pThis->isConnected=false;
			}
			

			// execute callback to function to handle encoder results here....
		}
	}

	ROS_INFO("MiabotAPI::receiveThread is shutting down");
}

int MiabotAPI::printFromSocket(int sd, char *buf)
{
	int len = 1;

	while((!isClosingDown || isConnected) && ((len = recv(sd, buf, buflen-1,MSG_DONTWAIT)) > 0) )
	{
		// send output to stdout
		//buf[len+1]='\0';
		//write(1,buf,len);
		
		// process the buffer ...
		string buffer(buf);

		// check to see if we have a well formed encoder pack
		//<: +0000000000:+0000000000>
		if ((buffer.length() >= 27) && (buf[1] == '<') && (buf[2]==':') && (buf[27]='>'))
		{
			//ROS_INFO(buf);

			string leftWheelStr = buffer.substr(3,3+10);
			string rightWheelStr = buffer.substr(16,16+10);

			long leftWheel = strtol(leftWheelStr.c_str(),NULL,10); 
			long rightWheel = strtol(rightWheelStr.c_str(),NULL,10);

			ostringstream outs;
			outs << "<" << leftWheel << ":" << rightWheel << ">";
			string str = outs.str();
			//write(1, str.c_str(), str.length());

			if (encoderCallback != NULL)
			{
				// callback to the parent robot driver with the encoder info
				encoderCallback(pParent, leftWheel, rightWheel);
			}
			else
			{
				ROS_ERROR("MiabotAPI:: Callback function not installed.");
			}

		}
		else if (buf[2] == '<')
		{
			ROS_INFO(buf);
		}

	}
     
	return (len);
}

int MiabotAPI::open(string hostName, int portNum)
{
	socketHandle = socket(AF_INET, SOCK_STREAM, 0);  /* init socket descriptor */
	struct sockaddr_in sin;
	struct hostent *host = gethostbyname(hostName.c_str());

	/*** PLACE DATA IN sockaddr_in struct ***/
	memcpy(&sin.sin_addr.s_addr, host->h_addr, host->h_length);
	sin.sin_family = AF_INET;
	sin.sin_port = htons(portNum);

	/*** CONNECT SOCKET TO THE SERVICE DESCRIBED BY sockaddr_in struct ***/
	if (connect(socketHandle, (struct sockaddr *)&sin, sizeof(sin)) < 0)
	{
		ROS_ERROR("Error connecting to robot.");
		return 0;
	}

	/* Disable the Nagle (TCP No Delay) algorithm */
	int flag = 1;
	int ret = setsockopt( socketHandle, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(flag) );

	sleep(1);   /* give server time to reply */

	isConnected = true;

	sleep(1);   /* give server time to reply */

	init();
	
	return 1;
}

int MiabotAPI::sendMotorCommand(int leftVel, int rightVel)
{
	// Send the command out the socket
	ostringstream outs;
	outs << "[=" << leftVel << "," << rightVel << "]" << endl;
	string cmd = outs.str();		
	//ROS_INFO(cmd.c_str());
	sendBuffer(&cmd);

}

int MiabotAPI::close()
{
	ROS_INFO("MiabotAPI::close() - closing down");

	isClosingDown = true;
	isConnected=false;

	::close(socketHandle);

	return 1;
}
	
