#include "RTX.h"
#include "Movac4.h"
#include <string>
#include <vector>
#include <fstream>
#include <map>

#define MILLISECONDS_PER_SECOND						1000
#define MILLISECONDS_PER_MINUTE						60*1000
#define PING_INTERVAL								5
#define AP_CONNECT_TIMEOUT							30//=30 seconds of unnsuccessful pings means disconnected
#define SENSOR_ID_FAULT_CHECK_INTERVAL				30//Check if there has been comms from sensor every SENSOR_ID_FAULT_CHECK_INTERVAL seconds
#define SENSOR_COMMS_TIMEOUT						60

#define SENSYS_EVENT_MESSAGES_PER_SECOND			8

#define ETHERNET_TCP_PRIORITY						7
#define ETHERNET_TCP_STACK							40*8192

#define PING_PRIORITY								7
#define PING_STACK									4*8192

#define FAULT_ON_SENSOR_ID_PRIORITY					7
#define FAULT_ON_SENSOR_ID_STACK					4*8192

#define SENSYSSTATS_PRIORITY								7//Not sure what priority to set here
#define SENSYSSTATS_STACK									4*8192

#define IP_RX_BUFF_LEN								150//4096
#define CIRC_BUFF_LEN								0x8000		// Must always be (2^n)
#define CIRC_BUFF_MASK								0x7FFF		// Must always be (2^n)-1

#define SENSYS_PORT									9125//Will always be the same port

#define MAX_AP_CONNECTS								2//Maximum number of AP's that can connect to this Sensys Server

#define ACDET_SIZE									4//Same as globals.h Line 83. 32 Detectors split into 4.

#define CHANNELS_PER_SLOT							4
#define EN_DASHES_IN_ADDRESS1_PARAM					3
#define NUMBER_OF_MSG_TYPES							3
#define NUMBER_OF_SENSYS_MSG_PARAMETERS				7
#define UNDEFINED_SENSOR_STATUS	3

#define NUMBER_OF_EVENT_PARAMETERS					7 
#define NUMBER_OF_WATCHDOG_PARAMETERS				5
#define NUMBER_OF_DIAGNOSTIC_PARAMETERS				7

#define PARAMETER_SENSOR_ID							0
#define PARAMETER_TIMESTAMP							1
#define PARAMETER_STATUS							2
#define PARAMETER_ADDRESS1							3

#define MAPPED_CHANNEL_IN_HEX_FORMAT				0
#define SHELF_NUMBER								1
#define SLOT_NUMBER									2
#define CHANNEL_NUMBER								3

#define TOTAL_DETECTORS								32*16

#define SENS_INACTIVE								0
#define SENS_ACTIVE									1
#define SENS_FAULT									2

#define SENSYS_CONFIG_FILE                          "SensysConfig.txt"
#define SENSYS_CONFIG_FILE_NEW                      "SensysConfig.txt.new"
#define SENSYS_SETTINGS_FILE                        "SensysSettings.txt"

enum EApConnectStatus
{
	CONNECTED,
	DISCONNECTED
};

typedef struct
{
	uchar ucFirstLoop;
	uchar ucSecondLoop;
	uchar ucDistance;
}SPEED_PAIR_CONFIG;


typedef struct detInfo
{
	std::string ID;
	int detNum;
	int status;
} detInfo;

void handleTCPMessagesTask();
void AcessPointPingTask();
void checkFaultOnSensorIDTask();

class SensysEthernetInterface
{
public:
	SensysEthernetInterface(void);
	~SensysEthernetInterface(void);
	
	void Initialize(void);
	
	int connectToSensysAP(void);
	int closeSensysSocket(void);
	
	void AccessPointPing(void);
	void handleTCPMessaging(void);
	void checkFaultOnSensorID(void);
	unsigned char getSensysAPConnectionStatus(void);
	void sensysCircularBuffer(char acTcpReceivedMsg[IP_RX_BUFF_LEN], unsigned int tcpReceivedMsgLength);
	std::string loadSensysMsgParameters(std::string strSensysTcpString);
	void sensysMsgParser(std::string strSensysTcpString, int noOfMsgParameters);
	void updateSensysConfigFile(std::string sensorID, int detectorNumber, std::string strSensysMsgType);
	unsigned char checkSensorIDList(std::string sensorID, std::string strFileName);
	void processSensysMsg(std::string strSensorID, std::string strSensysMsgType, int detectorNumber,  int detectorStatus);
	int enDashAddress1Tokenizer(std::string strMsgParameterValue,int numberOfTokens);
	void loadSensorDataFromFile(std::string strFileName);
	void updateVehicleStatistics(unsigned long ulSensysDetectorMap);
	void getSensysStatistics();
	void vehicleStatsReset();
	unsigned char checkIPAddressValidity(std::string strIPAddress);
	void writeToFile(std::string strFileName, std::string writeLine);
	void writeAppendToFile(std::string strFileName, std::string writeLine);
	
	unsigned long ulConfiguredSysDetectors;
	void SensysDetConfig(unsigned long SysDetectorConfig, SPEED_PAIR_CONFIG SpeedPairConfig[]);
	SPEED_PAIR_CONFIG SpeedDetectorPair[STIMULI];
	unsigned long millisecondsSinceTimeStamp;

	EApConnectStatus ApTcpConnectionState;
	std::string strSensysIPAddress;
	DWORD sensysIP;
	WORD sensysPort;
	int connectToSensysAPResult;
	int closeSensysSocketResult;
	
	int pingErrorCount;
	int pingResult;
	bool bPingSuccess;
	unsigned char ucPingTimeOut;
	
	detInfo sensysDetectors[TOTAL_DETECTORS];		
		
	int tcpReceivedMsgLength;
	int socketBlockTimeOut;
	char acTcpReceivedMsg[IP_RX_BUFF_LEN];
	
	unsigned char acSensysDet[ACDET_SIZE];
	unsigned char acSensysDetectorData[256];
	unsigned char sensysDataDetector0to7;
	unsigned char sensysDataDetector8to15;
	unsigned char sensysDataDetector16to23;
	unsigned char sensysDataDetector24to31;
	
	std::string strMsgParameterValue;
	std::string strToken;
	std::string strSensorID;
	
	std::vector <std::string> vectorMsgParameters;	
	std::string strSensysMsgType;
	
	std::vector <std::string> v_detectorConfigInfo;
	//elapsed time here denotes the last time an event or watchdog message was received for that specific sensorID
	std::map<std::string, int> map_sensorID_To_elapsedTime;
	
	size_t parameterPosition;
	size_t parameterValueStartPosition;
	size_t parameterValueEndPosition;	
	size_t tokenPosition;
	
	int noOfMsgParameters;
	int parameterValueLength;
	int tokenLength;
	int enDashCount;//Google "en dash" or "-" character
	int numberOfTokens;
	
	int detectorStatus;
	int detectorNumber; 
	int slotNumber;
	int channelNumber;
	int detcnt;
	
	unsigned long ulSensysDetectorStatus;
	unsigned long ulSensysDetectorMap;
	unsigned long ulPreviousSensysDetectorMap;
	unsigned long ulSensysDetectorMapMask;
	
	unsigned _int64 sensysDetectorTimeStamp;
	
	char tcpRxCircularBuffer[CIRC_BUFF_LEN];
	unsigned int rxBuffTail;

	RTX_TaskHandle TCPMessageTaskHandle;
	RTX_TaskHandle AcessPointPingTaskHandle;
	RTX_TaskHandle checkFaultOnSensorIDTaskHandle;
	
	//Statistics
	unsigned long ulSystemCountDetMap;
	unsigned long ulDetectorMask;
	
	unsigned long ulSystemOccupancyDetMap;
	
	unsigned short usSystemCount[STIMULI];
	unsigned long ulOccupancyStart[STIMULI];
	unsigned long ulOccupancyInterval[STIMULI];
	unsigned long ulSumOfOccupancyIntervals[STIMULI];
	unsigned long ulOccupancy[SYSTEM_DETECTORS];
	
	/*ulSystemCountDetMap and ulSystemOccupancyDetMap should be the same. Separated for clarity */
	unsigned long ulRisingEdgeDetMap;
	unsigned long ulFallingEdgeDetMap;
	
	unsigned long ulPreviousSystemCountDetMap;
	unsigned long ulPreviousSystemOccupancyDetMap;
	unsigned long ulPreviousEdgeDetMap;
	
	
	long long llSensysTimestamp[STIMULI]; 
	unsigned long ulTimeAtLeadingEdge[STIMULI];
	unsigned long ulTimeOnLeadingEdge[STIMULI];
	unsigned long llLatestTimeStamp;
	
	long long ulLeadToTrailEdgeInterval[SPEED_DETECTORS];	
	
	//std::ifstream sensysSettingsInputStream;
	//std::ofstream sensysSettingsOutputStream;
	
	std::string readIpFromConfigFile();
	void writeIpToSettingsFile(std::string strIPAddress);
	std::string getSensyDisplayIPAddress(std::string strSensysIPAdress);
	std::string getSensyIPAddress(std::string strSensyDisplayIPAddress);
};
extern SensysEthernetInterface SEI;