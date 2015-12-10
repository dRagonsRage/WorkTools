#include "FileIO.h"
#include "Helpers.h"
#include "SensysEthernetInterface.h"
#include "SensysProtocol.h"

#include "stdafx.h"
#include "Socket.h"
#include "WinSock.h"
#include "Tracing.h"
#include "TcpIpComms/TcpIpMessaging.h"
#include "macros.h"
#include "DetectorProcessor.h"
#include "Controller.h"

using namespace Detectors;

using namespace Helpers;

SOCKET sensysTCPSocket = INVALID_SOCKET;


SensysEthernetInterface::SensysEthernetInterface(void)
{
}

SensysEthernetInterface::~SensysEthernetInterface(void)
{
}

void SensysEthernetInterface::Initialize(void)
{	
	std::ofstream out;
	std::ifstream in;
	
	//---------------------------------File Headers:-------------------------------------
	string title1 = "Sensys Detectors";
	string title2 = "Input Number";
	string title3 = "Operating State";
	string title1Space = "     ";
	string title2Space = "         ";
	string title3Space = "    ";
	//-----------------------------------------------------------------------------------
	string item1LeftSpace = "    ";
	string item1RightSpace = "                 ";
	string item2Space = "                     ";
	string item3Space = "       ";

	detcnt=0;
	socketBlockTimeOut = 0;
	tcpReceivedMsgLength = 0;
	noOfMsgParameters = 0;	
	pingErrorCount = 0;
	bPingSuccess = false;

	detectorNumber					= 0;
	numberOfTokens					= 0;
	ulSensysDetectorStatus			= 0;
	ulPreviousSensysDetectorMap		= 0;
	ulPreviousSystemOccupancyDetMap = 0;
	ulPreviousEdgeDetMap			= 0;

	ApTcpConnectionState = DISCONNECTED;

	rxBuffTail = 0;
	
	for(int detector = 0; detector < STIMULI; detector++)
	{
		usSystemCount[detector] = 0;
	}
	
	//If File needs to be re-written from scratch:
	remove(SENSYS_CONFIG_FILE);
	remove(SENSYS_CONFIG_FILE_NEW);

	TCPMessageTaskHandle = RTX_CreateTask(handleTCPMessagesTask, ETHERNET_TCP_PRIORITY, ETHERNET_TCP_STACK, "handleSensysTCPMessages");
	AcessPointPingTaskHandle = RTX_CreateTask(AcessPointPingTask, PING_PRIORITY, PING_STACK, "handleAcessPointPing");
	checkFaultOnSensorIDTaskHandle = RTX_CreateTask(checkFaultOnSensorIDTask, FAULT_ON_SENSOR_ID_PRIORITY, FAULT_ON_SENSOR_ID_STACK, "handleCheckFaultOnSensorID");
	
	//Check if Config file is available for read. If not, create a new one from scratch
	in.open(SENSYS_CONFIG_FILE, std::ios::in);
	if(!in)
	{
		//If input stream cannot open the file, create and write to file first
		out.open(SENSYS_CONFIG_FILE, std::ios::out);
		out<<title1 + title1Space + title2 + title2Space + title3 + title3Space<<endl;
		out.close();
	}
	else
	{
		in.close();
	}	
	TRACE("Sensys Ethernet Interface Initialised",0);
	
}

void checkFaultOnSensorIDTask(void)
{
	SEI.checkFaultOnSensorID();
}
void AcessPointPingTask(void)
{
	SEI.AccessPointPing();
}

void SensysEthernetInterface::checkFaultOnSensorID(void)
{
	string strSensorIDFromFile;
	string fileLine;
	string strDetectorNumber;
	int detectorNumber;
	
	while(TRUE)
	{
		try//String.substr can crash if the file exists but is empty. Catch any exceptions here.
		{
			//Read SENSYS_CONFIG_FILE and force high on Sensys Detector Inputs
			std::ifstream sensysConfigInputStream;
			sensysConfigInputStream.open(SENSYS_CONFIG_FILE,std::ios::in);
			while(std::getline(sensysConfigInputStream,fileLine))
			{
				TRACE("Test",0);
				if(fileLine.find("Sensys") == std::string::npos)//If line read is not the Column headers
				{
					//Extract Sensor ID
					strSensorIDFromFile = fileLine.substr(fileLine.find_first_not_of(" "),10);//strDetectorNumber should be somewhere in between
					strSensorIDFromFile = strSensorIDFromFile.substr(0, strSensorIDFromFile.find_first_of(" "));
					
					//Extract Detector Number
					strDetectorNumber = fileLine.substr(15,25);//strDetectorNumber should be somewhere in between
					strDetectorNumber = strDetectorNumber.substr((strDetectorNumber.find_first_not_of(" ")), 2);
					detectorNumber = atoi(strDetectorNumber.c_str());
					
					map_sensorID_To_elapsedTime[strSensorIDFromFile] += SENSOR_ID_FAULT_CHECK_INTERVAL;
					if((map_sensorID_To_elapsedTime[strSensorIDFromFile] > SENSOR_COMMS_TIMEOUT) 
															&&
					  ((map_sensorID_To_elapsedTime[strSensorIDFromFile]) < (SENSOR_COMMS_TIMEOUT + (SENSOR_ID_FAULT_CHECK_INTERVAL*2))))
					{
						//Only send once
						processSensysMsg(strSensorIDFromFile, "diagnostic",detectorNumber, UNDEFINED_SENSOR_STATUS);
					}
				}
			}
			sensysConfigInputStream.close();
		}
		catch(exception &e)
		{
			TRACE("Error reading from SensysConfig.txt",0);
			TRACE(e.what(),0);
		}
		RTX_DelayInMilliSeconds(SENSOR_ID_FAULT_CHECK_INTERVAL*MILLISECONDS_PER_SECOND);
	}
}

void SensysEthernetInterface::AccessPointPing(void)
{
	RTX_DelayInMilliSeconds(1*MILLISECONDS_PER_SECOND);
	while(true)
	{
		int ipOctet1, ipOctet2, ipOctet3, ipOctet4;
		DWORD sensysPingIP;
		int pingError; 
		bool bReadSettingsSuccess = false;
		static long elapsedTime = 0;
		const char* cPingError;
		
		SEI.strSensysIPAddress = SEI.readIpFromConfigFile();
		
		sensysPingIP = htonl(sensysIP);		
		
		ipOctet1 = (sensysPingIP)		&	0xFF;
		ipOctet2 = (sensysPingIP>>8)	&	0xFF;
		ipOctet3 = (sensysPingIP>>16)	&	0xFF;
		ipOctet4 = (sensysPingIP>>24)	&	0xFF;
		
		if(ipOctet1 < 0 || ipOctet2 < 0 || ipOctet3 < 0 || ipOctet4 < 0)
		{
			TRACE("Sensys AP Ping Error: Invalid IP Address", 0);
			pingResult = 1;	
		}
		else
		{
			BYTE pingIP[] = {ipOctet4, ipOctet3, ipOctet2, ipOctet1};
		
			//Ping an IP: http://www.on-time.com/rtos-32-docs/rtip-32/reference-manual/extended-api/xn-ping.htm		
			pingResult = xn_ping(1, 1, 0, pingIP, 0, NULL, RTX_MilliSecondsToTicks(100), &elapsedTime);	
		}
		
		if(pingResult != 0)
		{
			pingError = xn_getlasterror();
			cPingError = xn_geterror_string(pingError);
			TRACE("Sensys AP Ping Error: ", 0);
			TRACE(cPingError,0);
			pingErrorCount++;
			bPingSuccess = false;
			if(pingErrorCount%(AP_CONNECT_TIMEOUT/PING_INTERVAL) == 0)//Every 30 seconds currently
			{
				ApTcpConnectionState = DISCONNECTED;
				ucPingTimeOut = TRUE;
				pingErrorCount = 0;	//Prevent overflow			
			}
		}		
		else
		{
			pingErrorCount = 0;
			ucPingTimeOut = FALSE;
			bPingSuccess = true;					
		}	
		RTX_DelayInMilliSeconds(PING_INTERVAL*MILLISECONDS_PER_SECOND);		
	}
}
void handleTCPMessagesTask(void)
{
	SEI.handleTCPMessaging();
}


void SensysEthernetInterface::handleTCPMessaging()
{	
	RTX_DelayInMilliSeconds(5*MILLISECONDS_PER_SECOND);
	while(true)
	{		
		const char* cCloseSocketError;
		const char* cTcpReceiveError;
		switch(ApTcpConnectionState)
		{
			case CONNECTED:
				tcpReceivedMsgLength = TCPReceive(sensysTCPSocket,acTcpReceivedMsg, IP_RX_BUFF_LEN);
				TRACE("Testing TCP Received Message BLOCKING",0);
				if(tcpReceivedMsgLength > 0)
				{
					sensysCircularBuffer(acTcpReceivedMsg,tcpReceivedMsgLength);
				}
				else if(tcpReceivedMsgLength == 0)
				{
					ApTcpConnectionState = CONNECTED;
					RTX_DelayInMilliSeconds(1*MILLISECONDS_PER_SECOND);
				}
				else
				{
					ApTcpConnectionState = DISCONNECTED;
					//TRACE error from TCP receive
					cTcpReceiveError = xn_geterror_string(xn_getlasterror());
					TRACE(cTcpReceiveError,0);
					
					//Close socket if theres an error receiving TCP messages
					closeSensysSocketResult = closeSensysSocket();	
					if(closeSensysSocketResult != 0)
					{
						cCloseSocketError = xn_geterror_string(xn_getlasterror());
						TRACE(cCloseSocketError,0);
					}									
					RTX_DelayInMilliSeconds(1*MILLISECONDS_PER_SECOND);
				}	
			break;
			
			case DISCONNECTED:
				//Close currently open socket before connecting to the next socket
				closeSensysSocketResult = closeSensysSocket();	
				if(closeSensysSocketResult != 0)
				{
					cCloseSocketError = xn_geterror_string(xn_getlasterror());
					TRACE(cCloseSocketError,0);
				}
				//Assign and connect to new socket									
				connectToSensysAPResult = connectToSensysAP();
				if(connectToSensysAPResult == 0)
				{
					ApTcpConnectionState = CONNECTED;
				}
				else
				{
					ApTcpConnectionState = DISCONNECTED;
					RTX_DelayInMilliSeconds(1*MILLISECONDS_PER_SECOND);						
				}
			break;
		}
	}
}

void SensysEthernetInterface::sensysCircularBuffer(char acTcpReceivedMsg[IP_RX_BUFF_LEN], unsigned int tcpReceivedMsgLength)
{
	static int iTokenCount = 0;
	static unsigned int rxBuffHead=0;

	string circularBufferContents;

	std::string strSensysMsgStringLocal;
	vectorMsgParameters.clear();
	unsigned int bufferIndex;

	for(bufferIndex = 0; bufferIndex < tcpReceivedMsgLength; bufferIndex++, rxBuffHead &= CIRC_BUFF_MASK)
	{
		if(acTcpReceivedMsg[bufferIndex] != '\n')
		{
			tcpRxCircularBuffer[rxBuffHead] = acTcpReceivedMsg[bufferIndex];

			if (tcpRxCircularBuffer[rxBuffHead++] == '>')
			{
				if (++iTokenCount == 2)
				{           
					iTokenCount = 0;
					strSensysMsgStringLocal.clear();

					if (rxBuffHead < rxBuffTail)
					{
						strSensysMsgStringLocal.append(std::string(tcpRxCircularBuffer).c_str(), rxBuffTail, CIRC_BUFF_LEN/*32768*/-rxBuffTail);
						strSensysMsgStringLocal.append(std::string(tcpRxCircularBuffer).c_str(), rxBuffHead);
					}
					else
					{
						strSensysMsgStringLocal.append(std::string(tcpRxCircularBuffer).c_str(), rxBuffTail, rxBuffHead-rxBuffTail);
					}                                

					rxBuffTail = rxBuffHead;

					strSensysMsgType = loadSensysMsgParameters(strSensysMsgStringLocal);
					noOfMsgParameters = vectorMsgParameters.size();
					sensysMsgParser(strSensysMsgStringLocal, noOfMsgParameters);
				}
			}
		}     
	}
}


/********************************************************************************************************************
checkIPAddressValidity: Input Display address in format: 172.020.050.100(note the 3 digit octets with leading zeros)
*********************************************************************************************************************/
uchar SensysEthernetInterface::checkIPAddressValidity(std::string strIPAddress)
{
	string strIPBuf = "";
	string strDisplayAddr;
	int iDigits = 0;
	int ipOctet = 0;//Each number between the dots e.g 172 in 172.20.50.100
	
	
	for(unsigned int iStrLen = 0; iStrLen <= strIPAddress.length(); ++iStrLen)
	{
		
		if(strIPAddress[iStrLen] != '.' && iStrLen != strIPAddress.length())
		{
			strIPBuf.push_back(strIPAddress[iStrLen]);
			iDigits++;
		}
		else
		{
			//Fill leading 0's
			while(iDigits < 3)
			{
				strIPAddress.push_back('0');
				iDigits++;
			}
			
			ipOctet = std::atoi(strIPBuf.c_str());
			if(ipOctet > 255)
			{
				return FALSE;
			}
			
			for(unsigned int strIPBufIndex = 0; strIPBufIndex < strIPBuf.length(); strIPBufIndex++)
			{
				strDisplayAddr.push_back(strIPBuf[strIPBufIndex]);
			}
			
			if(iStrLen < strIPAddress.length())
			{
				strDisplayAddr.push_back('.');
			}
			iDigits = 0;
			strIPBuf.clear();
		}
	}
	return TRUE;
}

int SensysEthernetInterface::connectToSensysAP(void)
{
	int connectResult;// = SOCKET_ERROR;
	bool bReadSettingsSuccess = false;
	
	SEI.strSensysIPAddress = SEI.readIpFromConfigFile();
	
	sensysIP = inet_addr(SEI.strSensysIPAddress.c_str());
	
	sensysPort = SENSYS_PORT;
	
	socketBlockTimeOut = 1;
	
	//TCPCreateSocket creates sockets and connects to AP
	sensysTCPSocket = TCPCreateSocket(sensysIP, SENSYS_PORT, socketBlockTimeOut);
	
	if((sensysTCPSocket == INVALID_SOCKET) || (sensysTCPSocket == SOCKET_ERROR))
	{
		connectResult = SOCKET_ERROR;
	}	
	else
	{
		//Create socket and connect usually returns 0 if successful. http://www.on-time.com/rtos-32-docs/rtip-32/reference-manual/socket-api/connect.htm
		connectResult = 0;
	}
	
	//Check if still connected to assigned socket: http://www.on-time.com/rtos-32-docs/rtip-32/reference-manual/extended-api/xn-tcp-is-connect.htm
	if(xn_tcp_is_connect(sensysTCPSocket) == false)
	{
		connectResult = SOCKET_ERROR;
	}
	else
	{
		connectResult = 0;
	}	
		
	return connectResult;
}

int SensysEthernetInterface::closeSensysSocket()
{
	//Close socket:			http://www.on-time.com/rtos-32-docs/rtip-32/reference-manual/socket-api/closesocket.htm
	//Close socket options: http://www.on-time.com/rtos-32-docs/rtip-32/reference-manual/socket-api/setsockopt.htm	
	int closeSocketResult;	
	
	linger sensysLingerOption;
	//Hard close without timeout
	sensysLingerOption.l_onoff = 1;
	sensysLingerOption.l_linger = 0;
	setsockopt(sensysTCPSocket, SOL_SOCKET, SO_LINGER,(const char*)&sensysLingerOption.l_onoff ,(sizeof(sensysLingerOption)));
	
	closeSocketResult = closesocket(sensysTCPSocket);
	if(closeSocketResult == 0)
	{
		closeSocketResult = 0;
	}
	else
	{
		closeSocketResult = SOCKET_ERROR;
	}
	return closeSocketResult;
}



std::string SensysEthernetInterface::loadSensysMsgParameters(std::string strSensysTcpString)
{
	vectorMsgParameters.clear();//Clearing parameter list first
	
	std::string strMsgType;
	std::string strMsgParameter;

	for(int msgType = 0; msgType < NUMBER_OF_MSG_TYPES; msgType++)
	{
		if((strSensysTcpString.find(messageTypes[msgType])) != std::string::npos)
		{
			strMsgType = messageTypes[msgType];
			for(int msgParameter = 0; msgParameter < NUMBER_OF_SENSYS_MSG_PARAMETERS; msgParameter++)
			{
				strMsgParameter = sensysMsgParameters[msgType][msgParameter];
				if(strMsgParameter != "")
				{
					vectorMsgParameters.push_back(strMsgParameter);
				}
			}
		}
	}
	if(strMsgType == "")
	{
		TRACE("Sensys Message Type Invalid!",0);
	}
	
	return strMsgType;
}

void SensysEthernetInterface::sensysMsgParser(std::string strSensysTcpString, int noOfMsgParameters)
{
	long long llTimeStamp = 0;
	int iTimeStamp = 0;
	char* ptr;
	int inputNumber = 0;
	
	for(int parameter = 0; parameter < noOfMsgParameters; parameter++)
	{
		//Look for each parameter after scanning the message type in the Sensys message string
		parameterPosition = strSensysTcpString.find(vectorMsgParameters.at(parameter),strSensysMsgType.length());
		if(parameterPosition == std::string::npos)
		{
			//TRACE("Error: Could not find paramater",0);
			continue;
		}
		//Parameter value starts from right after "
		parameterValueStartPosition = strSensysTcpString.find("\"", parameterPosition) + 1;
		if(parameterValueStartPosition == std::string::npos)
		{
			TRACE("Error: Invalid Sensys message format",0);
			continue;
		}
		//Parameter value ends right before you'd find the next "
		parameterValueEndPosition = strSensysTcpString.find("\"", parameterValueStartPosition) - 1;
		if(parameterValueEndPosition == std::string::npos)
		{
			TRACE("Error: Invalid Sensys message format",0);
			continue;
		}

		parameterValueLength = parameterValueEndPosition - parameterValueStartPosition + 1;
		
		switch(parameter)
		{
			case PARAMETER_SENSOR_ID:
				for(int parameterValuePosition = 0; parameterValuePosition < parameterValueLength; parameterValuePosition++)
				{
					strMsgParameterValue.push_back(strSensysTcpString.at(parameterValueStartPosition + parameterValuePosition));
				}
				strSensorID = strMsgParameterValue;
			break;			
			
			case PARAMETER_TIMESTAMP:
				for(int parameterValuePosition = 0; parameterValuePosition < parameterValueLength; parameterValuePosition++)
				{
					strMsgParameterValue.push_back(strSensysTcpString.at(parameterValueStartPosition + parameterValuePosition));
				}
				//Timestamp from the sensors arrive in this format: Timestamp = "1432740124.590". The timestamp is then converted to long long format
				if(strMsgParameterValue.length() > 3)
				{
					iTimeStamp = (int)strtol(&strMsgParameterValue[strMsgParameterValue.length() - 3],&ptr,10);//Gets the digits after the decimal point i.e Milliseconds				
					llTimeStamp = _atoi64(&strMsgParameterValue[0])*1000;//Gets the seconds
					llTimeStamp = llTimeStamp + iTimeStamp;//Timestamp in Total Milliseconds
					sensysDetectorTimeStamp = llTimeStamp;//In Milliseconds	
				}													
			break;
			
			case PARAMETER_STATUS:
				for(int parameterValuePosition = 0; parameterValuePosition < parameterValueLength; parameterValuePosition++)
				{
					strMsgParameterValue.push_back(strSensysTcpString.at(parameterValueStartPosition + parameterValuePosition));
				}
				detectorStatus = atoi(strMsgParameterValue.c_str());
				//TRACE("SENSYS EVENT STATUS = %d", detectorStatus);				
			break;
		
			case PARAMETER_ADDRESS1://case of address1
				//012345678
				//3f-0-15-4 is the address represented here as channelMapInHexFormat-shelfNumber-SlotNumber-ChannelNumber respectively
				for(int parameterValuePosition = 0; parameterValuePosition < parameterValueLength; parameterValuePosition++)
				{
					strMsgParameterValue.push_back(strSensysTcpString.at(parameterValueStartPosition + parameterValuePosition));
					if((strMsgParameterValue[parameterValuePosition]) == '-')
					{
						enDashCount++;
					}
				}
				if(enDashCount != EN_DASHES_IN_ADDRESS1_PARAM)
				{
					TRACE("Error: Invalid Sensys message format",0);
				}
				numberOfTokens = enDashCount + 1;			
				detectorNumber = enDashAddress1Tokenizer(strMsgParameterValue,numberOfTokens);			
				enDashCount = 0;
				//StoreDetInfo(detectorNumber, detectorStatus, strSensorID);
			break;
			
			default:
				for(int parameterValuePosition = 0; parameterValuePosition < parameterValueLength; parameterValuePosition++)
				{
					strMsgParameterValue.push_back(strSensysTcpString.at(parameterValuePosition + parameterValueStartPosition));
				}
			break;
		}

		//TRACE(&strMsgParameterValue[0],0);
		strMsgParameterValue.clear();
		tokenLength = 0;
		tokenPosition = 0;
	}
	
	llLatestTimeStamp = (unsigned long)sensysDetectorTimeStamp;
	llSensysTimestamp[detectorNumber-1] = 	sensysDetectorTimeStamp;
	millisecondsSinceTimeStamp = 0;
	processSensysMsg(strSensorID,strSensysMsgType, detectorNumber, detectorStatus);
}

void SensysEthernetInterface::updateSensysConfigFile(std::string sensorID, int detectorNumber, std::string strSensysMsgType )
{
	string writeFileLine;
	string readFileLine;
	ifstream in;
	unsigned int fileLinesCount;
	uchar ucSensorIDInList = FALSE;
	uchar ucRewriteFile = FALSE;
	
	//---------------------------------File Headers:-------------------------------------
	string title1 = "Sensys Detectors";
	string title2 = "Input Number";
	string title3 = "Operating State";
	string title1Space = "     ";
	string title2Space = "         ";
	string title3Space = "    ";
	//-----------------------------------------------------------------------------------

	string strDetectorNumber = "  ";
	if(detectorNumber < 10)
	{
		strDetectorNumber[0] = '0';
		strDetectorNumber[1] = '0' + detectorNumber;
	}
	else
	{
		strDetectorNumber[0] = '0' + detectorNumber;
	}
	
	ucSensorIDInList = checkSensorIDList(sensorID,SENSYS_CONFIG_FILE);
	
	if(!ucSensorIDInList)
	{
		if(strSensysMsgType == "diagnostic")
		{
			writeAppendToFile(SENSYS_CONFIG_FILE, "    " + sensorID + "                 " + strDetectorNumber + "                     " + "Fault ");
		}
		else
		{
			writeAppendToFile(SENSYS_CONFIG_FILE, "    " + sensorID + "                 " + strDetectorNumber + "                     " + "Normal");
		}
	}
	else
	{
		v_detectorConfigInfo.clear();
		loadSensorDataFromFile(SENSYS_CONFIG_FILE);
		fileLinesCount = v_detectorConfigInfo.size();
		TRACE("Test",0);
		for(unsigned int readLineIndex = 0; readLineIndex < v_detectorConfigInfo.size(); readLineIndex++)
		{
			readFileLine = v_detectorConfigInfo.at(readLineIndex);
			if(readFileLine.find(sensorID) != string::npos)
			{
				if(strSensysMsgType == "diagnostic")
				{
					if(readFileLine.find("Normal") != std::string::npos)
					{
						v_detectorConfigInfo.at(readLineIndex) = "    " + sensorID + "                 " + strDetectorNumber + "                     " + "Fault ";
						ucRewriteFile = TRUE;
					}					
				}
				
				if (((strSensysMsgType == "event") || (strSensysMsgType == "watchdog")))
				{
					if((readFileLine.find("Fault") != std::string::npos))
					{
						v_detectorConfigInfo.at(readLineIndex) = "    " + sensorID + "                 " + strDetectorNumber + "                     " + "Normal";
						ucRewriteFile = TRUE;
					}					
				}
			}
		}
		TRACE("TEST",0)
		if(ucRewriteFile == TRUE)
		{
			for(unsigned int writeLineIndex = 0; writeLineIndex < v_detectorConfigInfo.size(); writeLineIndex++)
			{
				writeFileLine = v_detectorConfigInfo.at(writeLineIndex);
				writeAppendToFile(SENSYS_CONFIG_FILE_NEW,writeFileLine);
			}
			remove(SENSYS_CONFIG_FILE);
			rename(SENSYS_CONFIG_FILE_NEW,SENSYS_CONFIG_FILE);
			ucRewriteFile = FALSE;
		}
	}
}
uchar SensysEthernetInterface::checkSensorIDList(std::string sensorID, std::string strFileName)
{
	ifstream in;
	string fileLine;
	in.open(strFileName.c_str(),std::ios::in);
	while(std::getline(in,fileLine))//in)
	{
		TRACE("Test",0);
		if(fileLine.find(sensorID) != std::string::npos)//If sensorID is already in the list:
		{
			return TRUE;
		}
	}
	in.close();
	
	return FALSE;
}

void SensysEthernetInterface::processSensysMsg(std::string strSensorID,std::string strSensysMsgType, int detectorNumber,  int detectorStatus)
{
	int inputNumber = 0;
	
	if(((strSensysMsgType == "event") || (strSensysMsgType == "watchdog")) && (detectorNumber > 0))
	{
		inputNumber = detectorNumber - 1;
		if(detectorStatus == 1)
		{
			ulSensysDetectorMap |= 0x01<<inputNumber;
			if(inputNumber < 8)
			{
				SEI.acSensysDet[0] |= 0x01<<inputNumber;
			}
			else if(inputNumber < 16)
			{
				acSensysDet[1] |= 0x01<<(inputNumber-8);
			}
			else if(inputNumber < 24)
			{
				acSensysDet[2] |= 0x01<<(inputNumber-16);
			}
			else if(inputNumber < 32)
			{
				acSensysDet[3] |= 0x01<<(inputNumber-24);
			}
		}
		else if(detectorStatus == 0)
		{ 
			ulSensysDetectorMap &= ~(0x01<<inputNumber);
			if(inputNumber < 8)
			{
				acSensysDet[0] &= ~(0x01<<inputNumber);
			}
			else if(inputNumber < 16)
			{
				acSensysDet[1] &= ~(0x01<<(inputNumber-8));
			}
			else if(inputNumber < 24)
			{
				acSensysDet[2] &= ~(0x01<<(inputNumber-16));
			}
			else if(inputNumber < 32)
			{
				acSensysDet[3] &= ~(0x01<<(inputNumber-24));
			}
		}
		map_sensorID_To_elapsedTime[strSensorID] = 0;
	}
	else if((strSensysMsgType == "diagnostic") && (detectorNumber > 0))
	{
		//Permanent call on diagnostic packet send
		inputNumber = detectorNumber - 1;
		ulSensysDetectorMap |= 0x01<<inputNumber;
		if(inputNumber < 8)
		{
			SEI.acSensysDet[0] |= 0x01<<inputNumber;
		}
		else if(inputNumber < 16)
		{
			acSensysDet[1] |= 0x01<<(inputNumber-8);
		}
		else if(inputNumber < 24)
		{
			acSensysDet[2] |= 0x01<<(inputNumber-16);
		}
		else if(inputNumber < 32)
		{
			acSensysDet[3] |= 0x01<<(inputNumber-24);
		}
	}
	
	//Updates vehcile statistics using current status of all the sensys detectors
	updateVehicleStatistics(ulSensysDetectorMap);	
	//Update Sensys config file if necessary from latest available information
	updateSensysConfigFile(strSensorID, detectorNumber,strSensysMsgType );
	
	TRACE("Test",0);
}


int SensysEthernetInterface::enDashAddress1Tokenizer(std::string strMsgParameterValue,int numberOfTokens)
{
	int detectorAddress = 0;
	for(int token = 0; token < numberOfTokens; token++)
	{
		tokenLength = strMsgParameterValue.find('-',tokenPosition) - tokenPosition;
		strToken = strMsgParameterValue.substr(tokenPosition, tokenLength);
		tokenPosition = tokenPosition + tokenLength + 1;
		switch(token)
		{
			//Byte 1 - Hex format of the mapped channel.
			case MAPPED_CHANNEL_IN_HEX_FORMAT:
				//Convert hex if needed
			break;
			
			case SHELF_NUMBER://Byte 2 - Shelf number should always be 0
				if(atoi(strToken.c_str()) != 0)
				{
					TRACE("Shelf Number incorrect!", 0);
					detectorAddress = 0;
				}
			break;
			
			case SLOT_NUMBER://Byte 3 - Slot number
				if(atoi(strToken.c_str()) > 15)
				{
					TRACE("Slot Number incorrect!", 0);
					detectorAddress = 0;
				}
				else
				{
					slotNumber = atoi(strToken.c_str());
				}
			break;
			
			case CHANNEL_NUMBER://Byte 4 - Channel Number
				if(atoi(strToken.c_str()) > 4)
				{
					TRACE("Channel Number incorrect!", 0);
					detectorAddress = 0;
				}
				else if(atoi(strToken.c_str()) < 1)
				{
					detectorAddress = 0;
				}
				else
				{
					channelNumber = atoi(strToken.c_str());
				}
			break;
			
			default:
				detectorAddress = 0;
			break;
		}
	}
	
	if(slotNumber > 0)
	{
		detectorAddress = (slotNumber-1)*CHANNELS_PER_SLOT + channelNumber;
	}
	else
	{
		TRACE("slotNumber invalid",0);
		detectorAddress = 0;
	}
	
	//Detector mapping allows for only "STIMULI" number of detectors which is 32 currently.	
	if(detectorAddress > STIMULI)
	{
		detectorAddress = 0;
	}
	
	//TRACE("Detector Number: %d", detectorAddress);
	
	//returns detectorAddress as 0 for any invalid input messages
	return detectorAddress;
}

unsigned char SensysEthernetInterface::getSensysAPConnectionStatus()
{
	int rtipBoolConnect;
	//http://www.on-time.com/rtos-32-docs/rtip-32/reference-manual/extended-api/xn-tcp-is-connect.htm
	rtipBoolConnect =  xn_tcp_is_connect(sensysTCPSocket);
	return ((unsigned char)rtipBoolConnect);
}

void SensysEthernetInterface::SensysDetConfig(unsigned long SysDetectorConfig, SPEED_PAIR_CONFIG SpeedPairConfig[])
{	
	std::ifstream sensysConfigInputStream;
	
	std::string fileLine;
	std::string strDetectorNumber;
	int inputNumber;
	
	string title1 = "Sensys Detectors";
	string title2 = "Input Number";
	string title3 = "Operating State";
	string title1Space = "     ";
	string title2Space = "         ";
	string title3Space = "    ";
	string item1LeftSpace = "    ";
	string item1RightSpace = "                 ";
	string item2Space = "                     ";
	string item3Space = "       ";
	
	//Assign config from Controller.cpp
	ulConfiguredSysDetectors = SysDetectorConfig;
	
	try//String.substr can crash if the file exists but is empty. Catch any exceptions here.
	{
		//Read SENSYS_CONFIG_FILE and force high on Sensys Detector Inputs
		sensysConfigInputStream.open(SENSYS_CONFIG_FILE,std::ios::in);
		while(sensysConfigInputStream)//std::getline(sensysConfigInputStream,fileLine))
		{
			std::getline(sensysConfigInputStream,fileLine);
			if(fileLine.find("Sensys") == std::string::npos)//If line read is not the Column headers
			{
				strDetectorNumber = fileLine.substr(15,25);//strDetectorNumber should be somewhere in between
				strDetectorNumber = strDetectorNumber.substr((strDetectorNumber.find_first_not_of(" ")), 2);
				inputNumber = ((atoi(strDetectorNumber.c_str())) - 1);
				ulSensysDetectorMap |= 0x01<<inputNumber;
				if(inputNumber < 8)
				{
					SEI.acSensysDet[0] |= 0x01<<inputNumber;
				}
				else if(inputNumber < 16)
				{
					acSensysDet[1] |= 0x01<<(inputNumber-8);
				}
				else if(inputNumber < 24)
				{
					acSensysDet[2] |= 0x01<<(inputNumber-16);
				}
				else if(inputNumber < 32)
				{
					acSensysDet[3] |= 0x01<<(inputNumber-24);
				}
			}		
		}
		sensysConfigInputStream.close();
	}
	catch(exception &e)
	{
		TRACE("Error reading from SensysConfig.txt",0);
		TRACE(e.what(),0);
	}
		
	for(int speedDetector = 0; speedDetector < SPEED_DETECTORS; speedDetector++)
	{
		SpeedDetectorPair[speedDetector].ucDistance		=	SpeedPairConfig[speedDetector].ucDistance;
		SpeedDetectorPair[speedDetector].ucFirstLoop	=	SpeedPairConfig[speedDetector].ucFirstLoop;
		SpeedDetectorPair[speedDetector].ucSecondLoop	=	SpeedPairConfig[speedDetector].ucSecondLoop;		
	}
}

void SensysEthernetInterface::updateVehicleStatistics(unsigned long ulSensysDetectorMap)
{			
	/*NOTE: System detectors refer to detectors used for calculating System count and Occupancy.*/
	//System Count
	ulSystemCountDetMap = ulSensysDetectorMap & ulConfiguredSysDetectors;				
	ulDetectorMask = (B32(00000000,00000000,00000000,00000001));
	for(int detector = 0; detector < STIMULI; detector++)
	{			
		if((!(ulPreviousSystemCountDetMap & ulDetectorMask)) && (ulSystemCountDetMap & ulDetectorMask))
		{
			usSystemCount[detector]++;							
		}
		ulDetectorMask <<= 1;
	}
	
	//System Occupancy	
	ulDetectorMask = (B32(00000000,00000000,00000000,00000001));	
	ulSystemOccupancyDetMap = ulSensysDetectorMap & ulConfiguredSysDetectors;		
	for(int detector = 0; detector < STIMULI; detector++)
	{			
		if(!(ulPreviousSystemOccupancyDetMap & ulDetectorMask) && (ulSystemOccupancyDetMap & ulDetectorMask))
		{
			ulOccupancyStart[detector] = (unsigned long)llSensysTimestamp[detector];						
		}
		
		if((ulPreviousSystemOccupancyDetMap & ulDetectorMask) && !(ulSystemOccupancyDetMap & ulDetectorMask))
		{
			if(ulOccupancyStart[detector] != 0)
			{
				ulOccupancyInterval[detector]		= (unsigned long)(llSensysTimestamp[detector] - ulOccupancyStart[detector]);
				ulSumOfOccupancyIntervals[detector] = ulSumOfOccupancyIntervals[detector] + ulOccupancyInterval[detector];
			}
		}					
		ulDetectorMask <<= 1;
	}			
	
	//Determine the rising edges		
	ulRisingEdgeDetMap =  ulSensysDetectorMap	^	ulPreviousEdgeDetMap;
	ulRisingEdgeDetMap =  ulRisingEdgeDetMap    &	ulSensysDetectorMap;	
	
	//Determine the falling edges
	ulFallingEdgeDetMap = ulSensysDetectorMap   |   ulPreviousEdgeDetMap;
	ulFallingEdgeDetMap = ulFallingEdgeDetMap   ^   ulSensysDetectorMap;	

	//Speed calculations 
	//Note: Speed calculations are done for each speed pair
	for(int speedDetectorPair = 0; speedDetectorPair < SPEED_DETECTORS; speedDetectorPair++)
	{			
		if ((1L<<(SpeedDetectorPair[speedDetectorPair].ucFirstLoop)) & (ulRisingEdgeDetMap))
		{
			ulTimeAtLeadingEdge[speedDetectorPair] = llLatestTimeStamp;
		} 	
		
		//Getting vehicle pulse length for classification
		if ((1L<<(SpeedDetectorPair[speedDetectorPair].ucFirstLoop)) & (ulFallingEdgeDetMap))
		{
			if (ulTimeAtLeadingEdge[speedDetectorPair] != 0)
			{
				ulTimeOnLeadingEdge[speedDetectorPair] = llLatestTimeStamp - ulTimeAtLeadingEdge[speedDetectorPair];
			}
		}
		
		
		if ((1L<<(SpeedDetectorPair[speedDetectorPair].ucSecondLoop)) & (ulRisingEdgeDetMap))
		{
			if (ulTimeAtLeadingEdge[speedDetectorPair] != 0)
			{
				ulLeadToTrailEdgeInterval[speedDetectorPair] = ((llLatestTimeStamp) - (ulTimeAtLeadingEdge[speedDetectorPair]));
				//ulTimeAtLeadingEdge[speedDetectorPair] = 0;
				//Speed Detector statistics sent on each detector 
				// -----------------------------------------------------------
				//	Note:
				//	GPIO version 2.105:
				//	Data[7-14] =	Pulse width of speed for each detector pair
				//	Data[15-22] =	Pulse width of leading speed pair loop for 
				//					vehicle classification
				// -----------------------------------------------------------
				//- ProcessSpeedClassification works in Decimeters. Hence the use of the correction factor of 10.
				acSensysDetectorData[7+speedDetectorPair]  = (unsigned char)(ulLeadToTrailEdgeInterval[speedDetectorPair]/10);//Speed Pulse 
				acSensysDetectorData[15+speedDetectorPair] = (unsigned char)(ulTimeOnLeadingEdge[speedDetectorPair]/10);//Classification Pulse					
				ProcessSpeedAndClassification((SpeedAndClassificationMsg*)&acSensysDetectorData[7]);
			}	
		}		
	}
						
	ulPreviousSystemCountDetMap		= 	ulSystemCountDetMap;
	ulPreviousSystemOccupancyDetMap = 	ulSystemOccupancyDetMap;
	ulPreviousEdgeDetMap			=	ulSensysDetectorMap;	
}

void SensysEthernetInterface::getSensysStatistics(void)
{
	//Counts and Occupancy gathered here. To be sent to CPU when requested every RMS_REPORT_INTERVAL. 
	//For Speed and Detector states on each detector change, look at updateVehicleStatistics
	ulDetectorMask = (B32(00000000,00000000,00000000,00000001));
	for(int detector = 0; detector < STIMULI; detector++)
	{	
		//Is the sensys detector still high at the end of the statistics gathering window?
		if(SEI.ulSystemOccupancyDetMap & SEI.ulDetectorMask)
		{
			//If there was a vehicle was on the sensys detector before start of the statistics gathering window
			if(ulOccupancyStart[detector] == 0)
			{
				ulSumOfOccupancyIntervals[detector] = (RMS_REPORT_INTERVAL)*(MILLISECONDS_PER_MINUTE);
			}
			//If there is a vehicle on the sensys detector at the end of the statistics gathering window
			else if(ulOccupancyStart[detector] != 0)
			{
				ulOccupancyInterval[detector]		= (llLatestTimeStamp + millisecondsSinceTimeStamp) - ulOccupancyStart[detector];
				ulSumOfOccupancyIntervals[detector] =  ulSumOfOccupancyIntervals[detector] + ulOccupancyInterval[detector];
			}								
		}		
		ulDetectorMask <<= 1;
		//Occupancy in Percentage
		ulOccupancy[detector] = ((100*(ulSumOfOccupancyIntervals[detector]))/
								((RMS_REPORT_INTERVAL)*(MILLISECONDS_PER_MINUTE)));
	}
}


void SensysEthernetInterface::vehicleStatsReset()
{	
	ulDetectorMask = (B32(00000000,00000000,00000000,00000001));
//----------------------------------------Clearing all parameters for start of next window----------------------------------------------//
	for(int detector = 0; detector < STIMULI; detector++)
	{
		usSystemCount[detector]						= 0;	
		
		//if sensys detector is still high, start the next window timestamp at the end of the previous timestamp.			
		if(ulSystemOccupancyDetMap & ulDetectorMask)
		{
			ulOccupancyStart[detector] = llLatestTimeStamp + millisecondsSinceTimeStamp;											
		}
		else
		{
			ulOccupancyStart[detector] = 0;
		}		
		ulDetectorMask <<= 1;		
		
		ulOccupancy[detector]						= 0;
		ulSumOfOccupancyIntervals[detector]			= 0;
	}		
	
	for(int speedDetectorPair = 0; speedDetectorPair < SPEED_DETECTORS; speedDetectorPair++ )
	{
		ulLeadToTrailEdgeInterval[speedDetectorPair]	= 0;
		ulTimeOnLeadingEdge[speedDetectorPair]			= 0;
	}
	
	millisecondsSinceTimeStamp = 0;
}

// -----------------------------------------------------------------------------
//	Function:		ProcessSpeedAndClassification
//
//	Description:	extracts speed information and determines whether there has
//					been a speed violation. If there has, a phase extension 
//					request is set for that phase by the controller object
//					each received SpeedAndClassificationMsg has information for 
//					each defined system detector
//
// -----------------------------------------------------------------------------
void	ProcessSpeedAndClassification(const SpeedAndClassificationMsg* SpeedMsg)
{
    Controller* controller = Controller::getController() ;

	if (controller == NULL)
    {
        return;
    }



	
	TRACE ("ProcessSpeedAndClassification: ", 0);
	//Wprintf (pDiagWin,"\n\rLengths: ");

	map<int,int> SpeedToSystem = controller->GetSpeedToSystemMap();

	// For each speed detector
	for (uchar ucSpeedDetectorIndex = 0; ucSpeedDetectorIndex < SpeedToSystem.size(); ucSpeedDetectorIndex++)
	{
		uchar ucCurrentSpeedPulse = SpeedMsg->SpeedPulse[ucSpeedDetectorIndex];
		uchar ucClassificationPulse = SpeedMsg->ClassificationPulse[ucSpeedDetectorIndex];

		if (!((ucCurrentSpeedPulse == 0) && (ucClassificationPulse == 0)))
		{
			//ASSERT(!((ucCurrentSpeedPulse == 0) && (ucClassificationPulse == 0)));  // can't both be zero 

			ASSERT(SpeedToSystem.find(ucSpeedDetectorIndex) != SpeedToSystem.end());

			int iSystemIndex = SpeedToSystem[ucSpeedDetectorIndex];

			if (iSystemIndex == 1)
			{
				int halt = 1;
			}


			// cast as a speed detector
			CSystemDetector* systemDetector = controller->GetSystemDetector(iSystemIndex);


			ASSERT(systemDetector != NULL);
			if (systemDetector != NULL)
			{
				

				uchar ucLastSpeed = systemDetector->GetLastInstantSpeed();
				TRACE (" Last Instant Speed: %d", ucLastSpeed);
				CONTTRACE ("\nDetector No: %d", ucSpeedDetectorIndex);
				CONTTRACE (" Speed Pulse: %d", ucCurrentSpeedPulse);
				CONTTRACE (" Classification Pulse: %d", ucClassificationPulse);

				if (ucCurrentSpeedPulse != 0)
				{
					systemDetector->ProcessRawSpeed(ucCurrentSpeedPulse);
				}

				//if (!((ucCurrentSpeedPulse == 0) && (ucClassificationPulse == 0)))
				if (
					(ucClassificationPulse != 0) 
					&& 
					(ucClassificationPulse != VEHICLE_CLASSIFICATION_INVALID) 
					)
				{
					systemDetector->ProcessRawLength(ucClassificationPulse);
				}
			} // end found system detector
		} 

	} // for each system detector
}



// -----------------------------------------------------------------------------
//	Function:		ProcessSystemDetectorInfo
//
//	Description:	extracts and stores system detector information
//
// -----------------------------------------------------------------------------
void ProcessSystemDetectorInfo(const SystemDetectorInfoMsg* SystemDetectors)
{
    Controller* controller = Controller::getController() ;
	static bool bFirstRequest = true;
	
	if (controller == NULL)
    {
        return;
    }
    
    //SystemDetectorNo detectorNo;
    
    SystemDetectorInfoMsg* systemDetectorInfoMsg 
        = (SystemDetectorInfoMsg*)SystemDetectors ;
    
    //Trace the data structure
	TRACE ("ProcessSystemDetectorInfo -->", 0);
	TRACE ("msgTypeId=%x", systemDetectorInfoMsg -> msgTypeId);
	TRACE ("msgLength=%x", systemDetectorInfoMsg -> msgLength);
	//nDisplayCount++;
	
	//Wprintf (pDiagWin,"--- ProcessSystemDetectors: %d --- \n\r",nDisplayCount);
	

	int nByteCounter = 0;
	ushort ucNewByte = 0;
	ushort count = 0;
	ushort usCounts[SYSTEM_DETECTORS];
	uchar  ucOccupancies[SYSTEM_DETECTORS];
	ushort mask = 0xF0;
	uchar *pStart = &(systemDetectorInfoMsg -> counts[0]);


	// Print Counts
	TRACE ("new counts [",0);
	//Wprintf (pDiagWin,"new counts [");
	for(int iSystemDetIndex = 0; iSystemDetIndex < SYSTEM_DETECTORS; iSystemDetIndex++)
	{
		// get lower
		ucNewByte = *(pStart + nByteCounter);
		nByteCounter++;
		count |= ucNewByte;

		// get upper
		ucNewByte = *(pStart + nByteCounter);
		nByteCounter++;
		count |= (ucNewByte << 8);

		//Wprintf (pDiagWin,"SysDet: DetNo: %d,Count %d\n\r",iSystemDetIndex ,count);

		//Wprintf (pDiagWin," %d",count);
		
		CONTTRACE (" %d", count);

		usCounts[iSystemDetIndex] = count;
		//reset
		count = 0;
		ucNewByte = 0;
	}
	CONTTRACE (" ]",0);
	//Wprintf (pDiagWin," ]\n\r");


	TRACE ("occupancies [",0);
	//Wprintf (pDiagWin,"occupancies [");
	for(int iSystemDetIndex = 0; iSystemDetIndex < SYSTEM_DETECTORS; iSystemDetIndex++)
	{
		//Wprintf (pDiagWin," %d",systemDetectorInfoMsg -> occupancies[iSystemDetIndex]);
		CONTTRACE (" %x", systemDetectorInfoMsg -> occupancies[iSystemDetIndex]);
		ucOccupancies[iSystemDetIndex] = systemDetectorInfoMsg -> occupancies[iSystemDetIndex];
	}
	CONTTRACE (" ]",0);
	//Wprintf (pDiagWin," ]\n\r");

	TRACE ("ProcessSystemDetectorInfo <--",0);

	vector<CSystemDetector*> SystemDetectors;


	map<int,int> OccupancyToSystem = controller->GetOccupancyToSystemMap();

	controller->GetConfiguredSystemDetectors(SystemDetectors);

	// first send all data to Jsent 

	for (uint iOccupancyIndex = 0; iOccupancyIndex < OccupancyToSystem.size(); iOccupancyIndex++)
	{
		ASSERT(OccupancyToSystem.find(iOccupancyIndex) != OccupancyToSystem.end());
	
		int iSystemIndex = OccupancyToSystem[iOccupancyIndex];

		//if (bFirstRequest)  // if this is the first time 
		//{
		//	SystemDetectors[iSystemIndex]->Reset();	
		//}
		SystemDetectors[iSystemIndex]->setLastReportedVehicleCount (usCounts[iOccupancyIndex]) ;
		SystemDetectors[iSystemIndex]->setLastReportedOccupancyPercentage ((Percentage)ucOccupancies[iOccupancyIndex]) ;

	}

	if (bFirstRequest)
	{
		bFirstRequest = false;
	}
	else
	{
		//	only send through message to rms once the returned 
		//	system detector data has been received and processed
		//	- do not send on controller start-up as the 'last' message is invalid
		controller->SetSystemDetectorUpdateRequired(true);
	}

    
    return;
    
}	// ProcessSystemDetectorInfo


std::string SensysEthernetInterface::readIpFromConfigFile()
{
	std::ifstream sensysSettingsInputStream;
	std::string sensysIPFromFile = "";
	std::string strDisplayIPAddress = "";
	bool bValidIpAddress;
	
	//File open
	sensysSettingsInputStream.open(SENSYS_SETTINGS_FILE,ios::in);
	if(sensysSettingsInputStream.fail())
	{
		TRACE("Error opening file for reading",0);
	}
	else
	{
		sensysSettingsInputStream>>sensysIPFromFile;		
	}
	
	//File close
	sensysSettingsInputStream.close();
	
	strDisplayIPAddress = SEI.getSensyDisplayIPAddress(sensysIPFromFile);
	bValidIpAddress = checkIPAddressValidity(strDisplayIPAddress);
	if(bValidIpAddress == false || sensysIPFromFile == "")
	{
		sensysIPFromFile = "0.0.0.0";
	}
	return sensysIPFromFile;
}

void SensysEthernetInterface::writeIpToSettingsFile(string strIPAddressToFile)
{
	uchar ucValidIpAddress;
	std::ofstream sensysSettingsOutputStream;
	std::string strDisplayIPAddress = getSensyDisplayIPAddress(strIPAddressToFile);
	ucValidIpAddress = checkIPAddressValidity(strDisplayIPAddress);
	
	if(ucValidIpAddress && strIPAddressToFile != "")
	{
		//File open
		sensysSettingsOutputStream.open(SENSYS_SETTINGS_FILE,ios::out);
		
		if(sensysSettingsOutputStream.fail())
		{
			TRACE("Error opening file for write",0);
		}
		else
		{
			sensysSettingsOutputStream<<SEI.strSensysIPAddress<<endl;
		}
		//File close
		sensysSettingsOutputStream.close();
	}
	else//Set to a default IP to display:
	{
		strIPAddressToFile = "0.0.0.0";
		TRACE("Error on writing invalid IP Address to file",0);
	}
}

/*********************************************************************************************************************
Displays IP Address in 172.020.050.100 format with leading zeroes. Used to display to user and editing from there.
*********************************************************************************************************************/
string SensysEthernetInterface::getSensyDisplayIPAddress(string strSensysIPAdress)
{
	string strIPBuf = "";
	string strDisplayAddr;
	int iDigits = 0;
	for(unsigned int iStrLen = 0; iStrLen <= strSensysIPAdress.length(); ++iStrLen)
	{
		
		if(strSensysIPAdress[iStrLen] != '.' && iStrLen != strSensysIPAdress.length())
		{
			strIPBuf.push_back(strSensysIPAdress[iStrLen]);
			iDigits++;
		}
		else
		{
			
			while(iDigits < 3)
			{
				strDisplayAddr.push_back('0');
				iDigits++;
			}
			
			for(unsigned int strIPBufIndex = 0; strIPBufIndex < strIPBuf.length(); strIPBufIndex++)
			{
				strDisplayAddr.push_back(strIPBuf[strIPBufIndex]);
			}
			
			if(iStrLen < strSensysIPAdress.length())
			{
				strDisplayAddr.push_back('.');
			}
			iDigits = 0;
			strIPBuf.clear();
		}
	}
	return strDisplayAddr;
}


/*********************************************************************************************************************
Leading 0's may cause connection issues. getSensyIPAddress() function strips leading 0's from the Display IP Address
*********************************************************************************************************************/
string SensysEthernetInterface::getSensyIPAddress(string strSensyDisplayIPAddress)
{
	string strIPBuf;
	string strSensysAddr;
	unsigned int iDigits = 0;
	int buffIndex = 0;
	int strSensysAddrIndex = 0;
	for(unsigned int iStrLen = 0; iStrLen <= strSensyDisplayIPAddress.length(); ++iStrLen)
	{
		if(strSensyDisplayIPAddress[iStrLen] != '.' && iStrLen != strSensyDisplayIPAddress.length())
		{
			strIPBuf.push_back(strSensyDisplayIPAddress[iStrLen]);
		}
		else
		{		
			if(strIPBuf[strSensysAddrIndex] != '0')
			{
				while(strSensysAddrIndex < 3)
				{
					strSensysAddr.push_back(strIPBuf[strSensysAddrIndex]);
					strSensysAddrIndex++;
				}
			}
			else
			{
				strSensysAddrIndex++;
				if(strIPBuf[strSensysAddrIndex] != '0')
				{
					while(strSensysAddrIndex < 3)
					{
						strSensysAddr.push_back(strIPBuf[strSensysAddrIndex]);
						strSensysAddrIndex++;
					}
				}
				else
				{
					strSensysAddrIndex++;
					strSensysAddr.push_back(strIPBuf[strSensysAddrIndex]);
				}
				
			}
			strSensysAddrIndex = 0;
			strIPBuf.clear();
			if(iStrLen != strSensyDisplayIPAddress.length())
			{
				strSensysAddr.push_back('.');
			}
		}
	}
	return strSensysAddr;
}

/******************************************************************
Gathers list of active sensys detectors from file
******************************************************************/
void SensysEthernetInterface::loadSensorDataFromFile(std::string strFileName)
{
	std::ifstream in;
	std::string fileLine;

	//Read
	in.open(strFileName.c_str(),std::ios::in);
	while(std::getline(in,fileLine))
	{
		v_detectorConfigInfo.push_back(fileLine);
	}
	in.close();
}

void SensysEthernetInterface::writeToFile(std::string strFileName, std::string writeLine)
{
	std::ofstream out;
	//Write
	out.open(strFileName.c_str(), std::ios::out);
	out<<writeLine<<std::endl;
	out.close();
}

void SensysEthernetInterface::writeAppendToFile(std::string strFileName, std::string writeLine)
{
	std::ofstream out;
	//Write
	out.open(strFileName.c_str(), std::ios::out|std::ios::app);
	out<<writeLine<<std::endl;
	out.close();
}