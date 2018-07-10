/*  Project Tracker API
	Automated Precision, Inc.
	Copyright © 1998-2001 by Automated Precision, Inc.

	 SUBSYSTEM:    TrackerAPI.dll Application
	 FILE:         TrackerAPI.cpp
	 AUTHOR:       Christopher Eunsoo  LEE

	 OVERVIEW
	 ========
	 Header file for implementation of CTracker Class.
*/
#ifndef _TRACKERAPI_
#define _TRACKERAPI_

// Standard DLL Macro Stuff
//
#if !defined (__TRACKER)
#define	TRACKERAPI __declspec(dllimport)
#else
#define	TRACKERAPI __declspec(dllexport)
#endif

#include <time.h>
#include <windows.h>

#define	No_of_SysParams 135
#define Number_of_Factors 24
#define Number_of_Functions 14
static int ErrorMessageNumber;				// Error Message Number

#pragma pack(8)
typedef struct {
	bool			Warm_Up_Time;			//	true:System Warming-Up false:System is ready.
	bool			Laser_Path_Error;		//	true:Laser Beam-Path Error false:Laser Beam-Path is O.K.
	bool			Laser_Dist_Error;		//	true:Laser Distance Status Error false:Distance Status is O.K.
	bool			External_Switch;		//	true:External Switch Contact false:External Switch Open
	bool			Filtering_Switch;		//	true:Filtering Switch On false:Filtering Switch Off
	clock_t			Time_Stamp;				//	Tracker Time Stamp in ms
	unsigned char	Operation_Mode;			//	Tracker Operation Mode
											//	0 -> Servo Free Mode (Idle)
											//	1 -> Servo Engaged Mode (Servo)
											//	2 -> Tracking Mode (Track)
											//	3 -> Losing target during Tracking (Track Idle)
											//	4 -> Not Used by API user (Internal Use Only)
											//	5 -> Searching the Encoder Index (Index Searching)
											//	6 -> Not Used by API user (Internal Use Only)
											//	7 -> Target Scan Search Mode (Search)
											//	8 -> AZ Axis Motor Run-Away
											//	9 -> EL Axis Motor Run-Away
	float			Laser_Intensity;		//	The Laser intensity is between 0.0-1.0
	float			Laser_Distance;			//	If the tracker mode is not in the Tracking Mode, the Laser Distance is 0.
	float			Current_Position_AZ;	//	Azimuth in Degree
	float			Current_Position_EL;	//	Elevation in Degree
	float			Air_Temperature;		//	Weather Sensor Information : Air Temperature (Centigrade) 
	float			Air_Pressure;			//	Weather Sensor Information : Air Pressure	 (mm/Hg)
	float			Current_Position_X;		//	X in mm
	float			Current_Position_Y;		//	Y in mm
	float			Current_Position_Z;		//	Z in mm
	float			Target_Velocity;		//	Velocity (mm/sec)
	float			Photo_X;				//	Calibrated Photo Sensor Eccentric in mm (X-direction)
	float			Photo_Y;				//	Calibrated Photo Sensor Eccentric in mm (Y-direction)
	float			Level_X;				//	Calibrated Level Sensor in ArcSec (X-direction)
	float			Level_Y;				//	Calibrated Level Sensor in ArcSec (Y-direction)
	bool			LevelX_OverLimit;		//	true:Out of the limit false:Within limit
	bool			LevelY_OverLimit;		//	true:Out of the limit false:Within limit
} REALTIME_INFO;

#pragma pack(8)
typedef struct {
	unsigned long	Captured_Points;		// Captured Number Of Points in the FIFO.
	unsigned long	Retrieved_Points;		// Retrieved Number Of Points from the FIFO.
} FIFO_INFO;

#pragma pack(8)
typedef struct {
	clock_t			Time_Stamp;				//	Tracker Time Stamp in ms
	unsigned char	Operation_Mode;			//	Tracker Operation Mode
											//	0 -> Servo Free Mode (Idle)
											//	1 -> Servo Engaged Mode (Servo)
											//	2 -> Tracking Mode (Track)
											//	3 -> Losing target during Tracking (Track Idle)
											//	4 -> Not Used by API user (Internal Use Only)
											//	5 -> Searching the Encoder Index (Index Searching)
											//	6 -> Not Used by API user (Internal Use Only)
											//	7 -> Target Scan Search Mode (Search)
	float			Laser_Distance;			//	If the tracker mode is not in the Tracking Mode, the Laser Distance is 0.
	float			Current_Position_AZ;	//	Azimuth in Degree
	float			Current_Position_EL;	//	Elevation in Degree
	float			Current_Position_X;		//	X in mm
	float			Current_Position_Y;		//	Y in mm
	float			Current_Position_Z;		//	Z in mm
	float			Level_X;				//	Calibrated Level Sensor in ArcSec (X-direction)
	float			Level_Y;				//	Calibrated Level Sensor in ArcSec (Y-direction)
	bool			LevelX_OverLimit;		//	true:Out of the limit false:Within limit
	bool			LevelY_OverLimit;		//	true:Out of the limit false:Within limit
} FIFO_RECORD;

#pragma pack(8)
typedef struct {
	bool	JogMode;		// true:Absolute Jogging, false:Incremental Jogging
	float	Azimuth;		// Azimuth Jogging Target
	float	Elevation;		// Elevation Jogging Target
} TARGET;

#pragma pack(8)
typedef struct {
	bool	JogMode;		// true:Absolute Jogging, false:Incremental Jogging
	bool	InPosition;		// true:Check the in-positioning, false:Fly on the jog operation
	float	Azimuth;		// Azimuth Jogging Target
	float	Elevation;		// Elevation Jogging Target
} TARGET_EXTENDED;

#pragma pack(8)
typedef struct {
	float			Reset_Laser_Distance;	//	If the tracker mode is not in the Tracking Mode, the Laser Distance is 0.
	float			Reset_Position_AZ;		//	Azimuth in Degree
	float			Reset_Position_EL;		//	Elevation in Degree
} RESET_ANGULAR, ANGULAR_DATA;

#pragma pack(8)
typedef struct {
	float			Reset_Position_X;		//	X in mm
	float			Reset_Position_Y;		//	Y in mm
	float			Reset_Position_Z;		//	Z in mm
} RESET_CARTESIAN, CARTESIAN_DATA;

#pragma pack(8)
typedef struct {
	double x;
	double y;
	double z;
	double w;
} D_VECTOR4;

#pragma pack(8)
typedef struct {
	double x;
	double y;
	double z;
} D_VECTOR3;

#pragma pack(8)
typedef struct {
	double x;
	double y;
} D_VECTOR2;


/////////////////////////////////////////////////////////////////////////////
// CTracker
// See TrackerAPI.cpp for the implementation of this class
//
class TRACKERAPI CTracker
{
// For the Internal Use Purpose Only.
private:
	unsigned char SerialPortIndex, BaudRateIndex;
	unsigned char Successful_Initialization;
	PVOID	pAPItracker;
	REALTIME_INFO TrackerMonitoringBuffer;
	REALTIME_INFO Real_Time_Data;
	bool	RealTime_Capturing;
	unsigned long m_lNumber_of_Points;
	unsigned long m_lCapturedPoints;
	unsigned long m_lRetrievedPoints;
	float	m_fRequiredParameter;
	clock_t	m_tWaitingDelay, TriggerTimer;
	float	m_fMinimumTriggerDistance;
	float	m_fVelocityBand;
	bool	m_bSemaphore_CriticalSection;
	unsigned char ProcedureSequencer[Number_of_Functions];
	unsigned char StabilityCounter;
	double	Reset_Inposition_Band;
	D_VECTOR4 Reset_Point;
	D_VECTOR4 Photo_Diff;
	D_VECTOR2 Current_AZEL;
	double	PriorValues[8];

	unsigned long m_lPoints;
	int		ExponentialFilterWeight;
	int		Number_of_AZ_Data, Number_of_EL_Data;
	float*	pElevation_Table;
	float*	pAzimuth_Table;
	void*	pTrackerSystemParam;
	char	PrmFileName[16], ModelNumber[16], SerialNumber[16], LicenseOwner[32];
	float	PhotoIntensity_X, PhotoIntensity_Y,
			CalibrationFactors[Number_of_Factors], CalculatedCalibrationFactors[6],
			Uncalibrated_Distance, Reset_Distance;
	D_VECTOR4 CurrentAngle;
	double	BirdBathAngle, BirdBathDistance,
			Gimbal_Coeff, Gimbal_Temp, In_Position[2], Softlimits[4];
	FIFO_INFO FIFO_Info;
	clock_t	ReadingTimer, DelayTimer, WeatherUpdate, WeatherUpdateInterval;
	bool	FilterApply, LaserDistanceError, FreshQVCdata, Level_Mode;
	int		NumberOfLevelTransDataPoints;
	float*	pLevelTrans_Xv;				// Spline function discrete points for the Level Sensor
	double*	pLevelTrans_Coeff;			// Spline function Coefficients for the Level Sensor
	int		NumberOfLevelBeamDataPoints;
	float*	pLevelBeam_Xv;				// Spline function discrete points for the Level Sensor
	double*	pLevelBeam_Coeff;			// Spline function Coefficients for the Level Sensor
	bool	bTriggerProcedure;
	CARTESIAN_DATA* pTemporaryPointer;
	CARTESIAN_DATA temp_ResetPosition;
	FIFO_RECORD* pRetrievalPointer;
	float*	pAllocatedMemory;
	D_VECTOR4* pReading_Buffer;
	D_VECTOR4  Averaging_Buffer;
	D_VECTOR2  InPositionDelta;
	unsigned int Point_Counter;
	unsigned char FacingStepNumber;
	float	VerifyPoints[3*4*5];
	float	Repeatabilities[4*2];
	int		DataSetNumber;
	bool	ErrorFlag, TriggerFired, TogglingSwitch;
	unsigned char FacingSequencer;
	float	FacingAngles[2];
	D_VECTOR4 FirstFacePoint, SecondFacePoint, BackSightPoint;
	float	Laser_Reset_Distance;
	bool	GoBackToBirdBath, Turning_Direction, Blocked_Beam_Path;
	double	In_Position_Check;
	
	D_VECTOR4 Sample_Points[4];
	unsigned int initial_counter;
	double	MinDistance;

	bool	FacingStatus;
	D_VECTOR2 saveSightAngles;
	D_VECTOR4 SightPointsCopy;
	float	Averaging_Counter;

	HANDLE	hAPI_EventHandle, hWorkerThread;
	DWORD	dwWorkerId, dwExitCode;
	static	DWORD RealTime_Monitoring(CTracker*);	// Realtime Data Reading Thread
	void	SetupDataStructure(float*, void*, float*);
	double	exponential_filter(int, bool, double, int);
	float	calibrate_encoder(double, int, float*);
	bool	FacingOperation(unsigned char*, float*, unsigned char, float*, double*, bool);
	void	CalibrationProcedure(float*, double*, double*, float*);
	int		mid_value(float, float, float);
	bool	GoToBirdBath(unsigned char*);
	bool	TwoFaceOperation(int, unsigned char*, unsigned char*, unsigned char*, unsigned char*, float*, int*, bool*, bool*, unsigned char*);
	unsigned char m_iRTCapture;
	bool	TrackerParametersSetup(char*, int*);
	void	TrackerPrmFileCopy(void);
	void	LogFileOperation(int, float*, float*, float*);
	void	CubicSplines(float*, float*, int, double*);
	float	CalcSpline(bool, float*, double*, int, float, bool*);
	void	tridiag(double*, int, double*);
	double	PolyCalc(double, double*, int);
	void	ConvertCartesianToSphericalCoordinates(CARTESIAN_DATA*);

protected:
	HANDLE	TaskHandle;

public:
	CTracker(void);
	~CTracker(void);

//	Tracker API Identification Information
	void getTrackerApiObject(char*		/* receiver */);	// Get Tracker API ObjectName
	void getTrackerApiRevision(char*	/* receiver */);	// Get Tracker API RevisionNumber
	void getTrackerApiRelease(char*		/* receiver */);	// Get Tracker API ReleaseDate

	void getControlVersion(char*		/* receiver */);	// Get Controller Firmware Version String
	void getModelNumber(char*			/* receiver */);	// Get Model Number String
	void getSerialNumber(char*			/* receiver */);	// Get Sensor Serial Number String
	void getRemoteTriggerID(char*		/* receiver */);	// Get Remote Trigger Identifier String
	bool setRemoteTriggerID(int,		/* RemoteTriggerID in Decimal(2 digits) */
							int* = &ErrorMessageNumber	/* Error Message Number */);
	void getLicenseOwner(char*			/* receiver */);	// Get License Owner

	virtual char* errorMsg(int	/* Error Number */);	// Returns Error Message string pointer
	virtual bool TrackerInitialization	(int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerInitialization	(char*,					// Configuration File Path+Name
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerInitialization	(LPCTSTR, /* ComPort */	// Controller initialization function
										 char*,					// Configuration File Path+Name
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerInitialization	(LPCTSTR, /* ComPort */	// Controller initialization function
									 	 DWORD,	  /* BaudRate */// Serial Communication Baudrate
										 char*,					// Configuration File Path+Name
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerInitialization	(bool*,		/* true:Initializing Sequence-Done,
													   false:Initializing Sequence-On Initializing Operation */
										 bool,		/* true:Abort the Initializing Procedure,
													   false:On Initializing Operation */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerInitialization	(bool*,		/* true:Initializing Sequence-Done,
													   false:Initializing Sequence-On Initializing Operation */
										 bool,		/* true:Abort the Initializing Procedure,
													   false:On Initializing Operation */
										 char*,		// Configuration File Path+Name
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerInitialization	(bool*,		/* true:Initializing Sequence-Done,
													   false:Initializing Sequence-On Initializing Operation */
										 bool,		/* true:Abort the Initializing Procedure,
													   false:On Initializing Operation */
										 LPCTSTR,	/* ComPort */	// Controller initialization function
										 char*,		// Configuration File Path+Name
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerInitialization	(bool*,		/* true:Initializing Sequence-Done,
													   false:Initializing Sequence-On Initializing Operation */
										 bool,		/* true:Abort the Initializing Procedure,
													   false:On Initializing Operation */
										 LPCTSTR,	/* ComPort */	// Controller initialization function
									 	 DWORD,		/* BaudRate */// Serial Communication Baudrate
										 char*,		// Configuration File Path+Name
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual void TrackerResetPosition	(RESET_ANGULAR*);  /* Tracker Reseting Position in Angular Coordinates */
	virtual bool TrackerResetOperation	(bool*, /* true:Resetting Sequence done, false:On Resetting Operation */
										 bool,	/* true:Abort the Resetting Procedure, false:On Resetting Operation */
										 RESET_ANGULAR*,  /* Tracker Reseting Position in Angular Coordinates */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual void TrackerResetPosition	(RESET_CARTESIAN*);  /* Tracker Reseting Position in XYZ Cartesian Coordinates */
	virtual bool TrackerResetOperation	(bool*, /* true:Resetting Sequence done, false:On Resetting Operation */
										 bool,	/* true:Abort the Resetting Procedure, false:On Resetting Operation */
										 RESET_CARTESIAN*,  /* Tracker Reseting Position in XYZ Cartesian Coordinates */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerTargetSearch	(bool*, /* true:Searching Sequence done, false:On Searching Operation */
										 bool,	/* true:Abort the Searching Procedure, false:On Searching Operation */
										 float,	/* target searching frequency */
										 float, /* target searching multiplier */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerHomingOperation	(bool*, /* true:Homing Sequence done, false:On Homing Operation */
										 bool,	/* true:Abort the Homing Procedure, false:On Homing Operation */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	// Return the pointer for the Realtime-based Data
	virtual REALTIME_INFO* TrackerMonitoringData(int /* Exponential Filter Strength from 1:No filter,
														the bigger, then the heavier filtration and the slower reponses*/);
	virtual bool TrackerChangingMode	(int,	/* Operation Mode Number */
												//	0 -> Servo Free Mode (Idle)
												//	1 -> Servo Engaged Mode (Servo)
												//	2 -> Tracking Mode (Track)
												//	3 -> Losing target during Tracking (Track Idle)
												//	4 -> Not Used by API user (Internal Use Only)
												//	5 -> Searching the Encoder Index (Index Searching)
												//	6 -> Not Used by API user (Internal Use Only)
												//	7 -> Target Scan Search Mode (Search)
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerJoggingMotion	(TARGET*, /*	Jogging Mode and Target Point */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerJoggingMotion	(bool*,		/* true:Jogging Sequence done, false:On Jogging Operation */
										 bool,		/* true:Abort the Jogging Procedure, false:On Jogging Operation */
										 TARGET_EXTENDED*, /* Jogging Mode and Target Point */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerCapturingOperation(int, /* CaptureMode	-3:Unacceptable
																-2:Unacceptable
																-1:Unacceptable
																 0:Unacceptable
																 1:Unacceptable
																 2:PC Realtime Static FrontSight Capturing
																 3:PC Realtime Static FrontBackSight Capturing
																 4:PC Realtime Temporal Capturing
																 5:Unacceptable	*/
										   FIFO_RECORD*,  /* Memory Pointer for the Data Retrieved */
										   unsigned long, /* Number of Points to be captured */
										   unsigned long, /* In case of Realtime Static Capturing	: Averaging Number of Points
															 In case of Realtime Temporal Capturing	: Capturing Interval(ms) */
										   int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerCapturingOperation(int, /* CaptureMode	-3:PC Realtime Automatic FrontBackSight Capturing
																-2:PC Realtime Automatic FrontSight Capturing
																-1:Unacceptable
																 0:Unacceptable
																 1:Unacceptable
																 2:Unacceptable
																 3:Unacceptable
																 4:Unacceptable
																 5:Unacceptable	*/
										   FIFO_RECORD*,  /* Memory Pointer for the Data Retrieved */
										   unsigned long, /* Number of Points to be captured */
										   clock_t,		  /* Waiting Delay in ms */
										   float,		  /* Minimum Trigger Distance in mm */
										   float,		  /* Velocity Band in mm/sec */
										   unsigned long, /* Averaging Number of Points	*/
										   int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerCapturingOperation(int, /* CaptureMode	-3:Unacceptable
																-2:Unacceptable
																-1:Unacceptable
																 0:Unacceptable
																 1:Unacceptable
																 2:Unacceptable
																 3:Unacceptable
																 4:Unacceptable
																 5:PC Realtime Spatial Capturing	*/
										   FIFO_RECORD*,  /* Memory Pointer for the Data Retrieved */
										   unsigned long, /* Number of Points to be captured */
										   float,		  /* In case of Realtime Spatial Capturing	: Capturing Distance(mm) */
										   int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerCapturingOperation(int, /* CaptureMode	-3:Unacceptable
																-2:Unacceptable
																-1:Unacceptable
																 0:Controller Static Capturing
																 1:Controller Temporal Capturing
																 2:Unacceptable
																 3:Unacceptable
																 4:Unacceptable
																 5:Unacceptable	*/
										   unsigned long, /* Number of Points to be captured */
										   unsigned long, /* In case of FIFO Static Capturing		: Averaging Duration(ms)[1-3000]
															 In case of FIFO Temporal Capturing		: Capturing Interval(ms)[1-3000] */
										   int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual	bool TrackerCommandTrigger	(int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerFifoRetrieval	(bool*, /* true:FIFO Retrieval Sequence done, false:On FIFO Retrieving Operation */
										 bool,	/* true:Abort the Retrieving Procedure, false:On Retrieving Operation */
										 FIFO_RECORD*,	/* Memory Pointer for the Data Retrieved */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerTaskingStop		(int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual void TrackerFIFOinformation(FIFO_INFO*,	/* returned Fifo Information */ 
										int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerVerifyingOperation(bool*, /* true:Verifying Sequence-Done,
													 false:Verifying Sequence-On Verifying Operation */
										   bool,  /* true:Abort the Verifying Procedure, false:On Verifying Operation */
										   bool*, /* In Position flag for the AZ,EL 
													 Point A : true if in-pos is within +/-0.5deg(EL 0.0)
													 Point B : true if in-pos is within +/-0.5deg(EL 0.0)
													 Point C : true if in-pos is within +/-5.0deg(EL +55.0)
													 Point D : true if in-pos is within +/-5.0deg(EL -55.0) */
										   bool*, /* Data PickUp flag  true(command):read the current point, false(return):operation done */
										   unsigned char*, /* Position Indicator
													 Point A : return value = 1
													 Point B : return value = 2
													 Point C : return value = 3
													 Point D : return value = 4	*/
										   float*, /* Calibration Result - Single Dimensional 4th order float Array
												      The final result is returned when the Calibration Procedure has been done.
													  The ratio is between 0.0 and 1.0.
													  Array [0] : Squareness Calibration Result
													  Array [1] : T-axis offset Calibration Result
													  Array [2] : Z-axis offset Calibration Result
													  Array [3] : T-Beam Deviation Calibration Result
													  Array [4] : Z-Beam Deviation Calibration Result
													  Array [5] : T-V Distance Calibration Result	*/
										   int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerVerifyingOperation(int,	/* QVC Method - 0 : Single Cycle QVC Operation, 1 : Three Cycle QVC Operation */
										   bool*, /* true:Verifying Sequence-Done,
													 false:Verifying Sequence-On Verifying Operation */
										   bool,  /* true:Abort the Verifying Procedure, false:On Verifying Operation */
										   bool*, /* In Position flag for the AZ,EL 
													 Point A : true if in-pos is within +/-0.5deg(EL 0.0)
													 Point B : true if in-pos is within +/-0.5deg(EL 0.0)
													 Point C : true if in-pos is within +/-5.0deg(EL +55.0)
													 Point D : true if in-pos is within +/-5.0deg(EL -55.0) */
										   bool*, /* Data PickUp flag  true(command):read the current point, false(return):operation done */
										   unsigned char*, /* Position Indicator
													 Point A : return value = 1
													 Point B : return value = 2
													 Point C : return value = 3
													 Point D : return value = 4	*/
										   float*, /* Calibration Result - Single Dimensional 4th order float Array
												      The final result is returned when the Calibration Procedure has been done.
													  The ratio is between 0.0 and 1.0.
													  Array [0] : Squareness Calibration Result
													  Array [1] : T-axis offset Calibration Result
													  Array [2] : Z-axis offset Calibration Result
													  Array [3] : T-Beam Deviation Calibration Result
													  Array [4] : Z-Beam Deviation Calibration Result
													  Array [5] : T-V Distance Calibration Result	*/
										   int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerQvcUpdate(int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual void TrackerDefaultRecovery(void);
	virtual bool TrackerCheckingOperation(bool*, /* true:Checking Sequence-Done,
													false:Checking Sequence-On Checking Operation */
										  bool,  /* true:Abort the Checking Procedure, false:On Checking Operation */
										  unsigned char, /* Averaging Time in sec for the Position Data (recommendation: 5 sec)*/
										  float*, /* Backsight Result - Single Dimensional 2nd order float Array
													 The final result is returned when the Backsight Procedure has been done.
													 Array [0] : Checking Results for Azimuth BackSight Difference
													 Array [1] : Checking Results for Elevation BackSight Difference */
										  int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerBacksightOperation(bool*, /* true:BackSighting Sequence-Done,
													 false:BackSighting Sequence-On BackSighting Operation */
										   bool,  /* true:Abort the BackSighting Procedure, false:On BackSighting Operation */
										   int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerToolingBall		(bool*, /* true:Calculating Sequence-Done,
										 		   false:Calculating Sequence-On Calculating Operation */
										 bool,  /* true:Abort the Calculating Procedure, false:On Calculating Operation */
										 float, /* SMR Diameter(mm) */
										 float, /* Tooling Ball Diameter(mm) */
										 float, /* Center Error Tolerance(mm) */
										 unsigned int, /* Number of Calculation (10 - 1500) */ 
										 float*, /* Calculation Progress(%) */
										 CARTESIAN_DATA*, /* Tooling Ball Center point */
										 float*, /* Average Error */
										 float*, /* Maximum Error */
										 float*, /* RMS Error */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerNestReading		(bool*, /* true:Readinging Sequence-Done,
										 		   false:Reading Sequence-On Reading Operation */
										 bool,  /* true:Abort the Reading Procedure, false:On Reading Operation */
										 unsigned int, /* Number of Readings for the averaging operation (10 - 1500) */ 
										 float*, /* Reading Progress(%) */
										 CARTESIAN_DATA*, /* SMR Center point */
										 float*, /* Average Error */
										 float*, /* Maximum Error */
										 float*, /* RMS Error */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerSetEnvironment	(float, /* Manual Air Pressure in mm/Hg. If it is 0.0, then the Automatic Sensor value will be applied.(580.0mm/Hg - 800.0mm/Hg) */
										 float,	/* Manual Air Temperature in centigrade. If it is 0.0, then the Automatic Sensor value will be applied.(5.0C - 45.0C) */
										 float,	/* Relative Humidity in precentage. It should be set correctly whenever this function is called. (1%-100%) */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerSetEnvironment	(unsigned char, /* Updating time interval (1sec - 60sec). If it is 0sec, then the Updating Data is disabled. */ 
										 float, /* Manual Air Pressure in mm/Hg. If it is 0.0, then the Automatic Sensor value will be applied.(580.0mm/Hg - 800.0mm/Hg) */
										 float,	/* Manual Air Temperature in centigrade. If it is 0.0, then the Automatic Sensor value will be applied.(5.0C - 45.0C) */
										 float,	/* Relative Humidity in precentage. It should be set correctly whenever this function is called. (1%-100%) */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerGetEnvironment	(float*, /* Air Pressure in mm/Hg. If it is over the range (580.0mm/Hg - 800.0mm/Hg), then Error. */
										 float*, /* Air Temperature in centigrade. If it is over the range (5.0C - 45.0C), then Error. */
										 float*, /* Relative Humidity in precentage. If it is over the range (1%-100%), then Error.  */
										 int* = &ErrorMessageNumber	/* Error Message Number */);
	virtual bool TrackerClosingOperation(int* = &ErrorMessageNumber	/* Error Message Number */);
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif
