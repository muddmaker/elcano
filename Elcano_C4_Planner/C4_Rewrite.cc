#include <SD.h>
#include <Elcano_Serial.h>
#include <WaypointReader.h>
#include <PathFinder.h>

elcano::Waypoint location;
elcano::ParseState ps;
elcano::SerialData dt;
elcano::JunctionReader jr;

void setup() {
	Serial.begin(9600);
	Serial1.begin(9600);
	Serial2.begin(9600);
	
	dt.clear();
	ps.dt = &dt;
	ps.dev = &Serial1;
	
	/*
	location.east_mm   = ;
	location.north_mm  = ;
	location.east_x1k  = ;
	location.north_x1k = ;
	*/
	
	int r = jr.initialize("MAP_DEFS.TXT", location);
	elcano::JunctionReader::dump_error_string(r, &Serial);
}

void loop() {
	elcano::ParseStateError pse = ps.update();
	if (pse == elcano::ParseStateError::success) {
		if (dt.kind == elcano::MsgType::goal) {
			Junction *j = find_path(jr.junctions, jr.junctions_count, /* start */, /* end */);
		} else {
			dt.write(&Serial2);
		}
	}
}
