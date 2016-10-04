#pragma once

#include <HardwareSerial.h>

/*
** WaypointReader.h
** By Dylan Katz
**
** Reads waypoint/junction information from SD cards
*/

namespace elcano {

const int END           = 0x04000;
const int GOAL          = 0x02000;
const int CONES         = 5;
const int MAX_DISTANCE  = 0x3ffffff;
const int MAX_MAPS      = 10;
const int MAX_JUNCTIONS = 30;

struct Waypoint {
	int index;
	long latitude;
	long longitude;
	unsigned long time_ms;
	long speed_mmPs;
	long east_mm;
	long north_mm;
	int east_x1k;
	int north_x1k;
	long sigma_mm;
	
	void write_as_SerialData(HardwareSerial *);
};

struct Junction {
	long east_mm;
	long north_mm;
	int destination[4];
	long distance_mm[4];
};

struct JunctionReader {
	float longitude = 0;
	float latitude = 0;
	char *map_name = NULL;
	Junction *junctions = NULL;
	int junctions_count = 0;
	
	/*! \brief This code parses the files, and reads to the values in this struct

	Complete Error Codes:

	 0     Good
	-1     Bad map definition file name
	-2     Unable to find good map to use
	-3     Bad waypoint definition file name
	 aabb  Error parsing map file (line: aa, parse error: bb (see below))
	 1aabb Error parsing waypoint file (line: aa, parse error: bb (see below))

	Waypoint Definition Parsing Errors:
	
	2,4,6,8,10,12,14,16,18 Bad commas
	1,3                    Bad signed number (-?[0-9]+)
	5,7,9,11               Bad number or end ([0-9]+ / 'END')
	13,15,17,19            Bad number        ([0-9]+)
	20                     Bad end-of-line   (\r \n? / \n)
	
	Map Definition Parsing Errors:
	
	2,4 Bad commas
	1,3 Bad floating number (-?[0-9]+(.[0-9]+)?)
	5   Bad file name       ([0-9a-zA-Z._-]+)
	6   Bad end-of-line (\r \n? / \n)
	*/
	int initialize(const char *, const Waypoint &);
	
	static void dump_error_string(int, HardwareSerial *);
};

}
