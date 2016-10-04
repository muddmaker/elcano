#include <WaypointReader.h>
#include <Elcano_Serial.h>
#include <SD.h>

/*
** WaypointReader.cpp
** By Dylan Katz
**
** Reads waypoint/junction information from SD cards
*/

namespace elcano {

/*
entry <- ('-'? num _ ','_)^2 (('END' / num) _ ',' _)^4 (num _ ','_)^3 num _ nl
map <- (flt _ ',' _)^2 [0-9a-zA-Z_.]+ _ nl
flt <- '-'? num ('.' num)?
num <- [0-9]+
nl <- '\r' '\n'? / '\n'
_ <- [ \t]*
*/

static inline void spaces(File *f) {
	for (; f->peek() == ' ' || f->peek() == '\t'; f->read());
}

static inline bool newline(File *f) {
	if (f->peek() == '\r') {
		f->read();
		if (f->peek() == '\n') f->read();
		return true;
	} else if (f->peek() == '\n') {
		f->read();
		return true;
	}
	return false;
}

static inline bool number(File *f, long *n) {
	if (f->peek() < '0' || f->peek() > '9') return false;
	for (; f->peek() >= '0' && f->peek() <= '9'; f->read())
		*n = 10 * *n + f->peek() - '0';
	return true;
}

static inline bool signed_number(File *f, long *n) {
	bool neg = false;
	if (f->peek() == '-') {
		neg = true;
		f->read();
	}
	if (!number(f, n)) return false;
	if (neg) *n = -(*n);
	return true;
}

static inline bool number_floating(File *f, float *n) {
	float factor = 1;
	if (f->peek() == '-') {
		factor = -1;
		f->read();
	}
	if (f->peek() < '0' || f->peek() > '9') return false;
	for (; f->peek() >= '0' && f->peek() <= '9'; f->read())
		*n = 10 * *n + f->peek() - '0';
	if (f->peek() != '.') {
		*n *= factor;
		return true;
	}
	f->read();
	for (; f->peek() >= '0' && f->peek() <= '9'; f->read()) {
		factor *= 0.1;
		*n = 10 * *n + f->peek() - '0';
	}
	*n *= factor;
	return true;
}

static inline bool number_or_end(File *f, long *n) {
	if (f->peek() == 'E') {
		f->read();
		if (f->peek() == 'N') {
			f->read();
			if (f->peek() == 'D') {
				f->read();
				*n = END;
				return true;
			}
		}
		return false;
	}
	return number(f, n);
}

static inline bool filename(File *f, char **n) {
	const int start = f->position();
	while ((f->peek() >= '0' && f->peek() <= '9') ||
	       (f->peek() >= 'a' && f->peek() <= 'z') ||
	       (f->peek() >= 'A' && f->peek() <= 'Z') ||
	        f->peek() == '_' || f->peek() == '.'  || f->peek() == '-')
		f->read();
	const int end = f->position();
	if (end - start == 0) return false;
	*n = new char[end - start + 1];
	for (f->seek(start); f->position() < end; f->read())
		n[f->position() - start] = f->peek();
	n[end - start] = '\0';
	return true;
}

static inline int map(File *f, float *lat, float *lon, char **name) {
	if (!number_floating(f, lat)) return 1;
	spaces(f);
	if (f->peek() != ',') return 2; else f->read();
	spaces(f);
	if (!number_floating(f, lon)) return 3;
	spaces(f);
	if (f->peek() != ',') return 4; else f->read();
	spaces(f);
	if (!filename(f, name)) return 5;
	spaces(f);
	if (!newline(f)) return 6;
	return 0;
}

static inline int entry(File *f, Junction *j) {
	if (!signed_number(f, &j->east_mm)) return 1;
	spaces(f);
	if (f->peek() != ',') return 2; else f->read();
	spaces(f);
	if (!signed_number(f, &j->north_mm)) return 3;
	spaces(f);
	if (f->peek() != ',') return 4; else f->read();
	spaces(f);
	for (int i = 0; i < 4; ++i) {
		if (!number_or_end(f, j->destination + i)) return 5 + 2 * i;
		spaces(f);
		if (f->peek() != ',') return 6 + 2 * i; else f->read();
		spaces(f);
	}
	for (int i = 0; i < 3; ++i) {
		if (!number(f, j->distance_mm + i)) return 13 + 2 * i;
		spaces(f);
		if (f->peek() != ',') return 14 + 2 * i; else f->read();
		spaces(f);
	}
	if (!number(f, j->distance_mm + 3)) return 19;
	spaces(f);
	if (!newline(f)) return 20;
	return 0;
}

static inline int maps(File *f, float *lat, float *lon, char **name,
                       int max, int *count) {
	int r;
	*count = 0;
	for (int i = 0; i < max; ++i) {
		spaces(f);
		if (f->peek() == -1)
			return 0;
		r = map(f, lat + i, lon + i, name + i);
		if (r != 0)
			return 100 * i + r;
		else
			++(*count);
	}
}

static inline int entries(File *f, Junction *j, int max, int *count) {
	int r;
	*count = 0;
	for (int i = 0; i < max; ++i) {
		spaces(f);
		if (f->peek() == -1)
			return 0;
		r = entry(f, j + i);
		if (r != 0)
			return 100 * i + r;
		else
			++(*count);
	}
}

int JunctionReader::initialize(const char *map_defs_name, const Waypoint &current_location) {
	File map_defs = SD.open(map_defs_name, FILE_READ);
	if (!map_defs) return -1;
	
	int count;
	int closest_id = -1;
	long closest_dist2 = MAX_DIST;
	float *latitudes  = new float[MAX_MAPS];
	float *longitudes = new float[MAX_MAPS];
	char **file_names = new char*[MAX_MAPS];
	int r = maps(&map_defs, latitudes, longitudes, file_names, MAX_MAPS, &count);
	
	map_defs.close();
	if (r != 0) goto err;
	
	for (int i = 0; i < count; ++i) {
		long lat_diff = latitudes[i] - current_location.latitude;
		long lon_diff = longitudes[i] - current_location.longitude;
		long dist2 = lat_diff * lat_diff + lon_diff * lon_diff;
		if (dist2 < closest_dist2) {
			closest_id = i;
			closest_dist2 = dist2;
		}
	}
	
	if (closest_id >= 0) {
		latitude = latitudes[closest_id];
		longitude = longitudes[closest_id];
		map_name = file_names[closest_id];
		for (int i = 0; i < count; ++i) if (i != closest_id) delete file_names[i];
		delete latitudes;
		delete longitudes;
		delete file_names;
		
		File junction_defs = SD.open(map_name, FILE_READ);
		if (!junction_defs) return -2;
		
		junctions = new Junction[MAX_JUNCTIONS];
		r = entries(&waypoint_defs, junctions, MAX_JUNCTIONS, &junctions_count);
		return (r != 0) ? r + 10000 : 0;
	}
	r = -3;
err:
	for (int i = 0; i < count; ++i) delete file_names[i];
	delete file_names;
	delete latitudes;
	delete longitudes;
	return r;
}

void JunctionReader::dump_error_string(int r, HardwareSerial *dev) {
	if (r >= 10001 && r <= 10020 + 100 * MAX_JUNCTIONS) {
		dev->print("Waypoint Definition Error!\n\tfile: ");
		r -= 10000;
		dev->println(local_data::map_name);
		dev->print("\tline: ");
		dev->println((r / 100) + 1);
		dev->print("\terror: ");
		switch (r % 100) {
		case 2:  case 4:  case 6:  case 8:
		case 10: case 12: case 14: case 16:
		case 18: dev->println("bad comma"); break;
		case 1:  case 3:  dev->println("bad signed number (-?[0-9]+)"); break;
		case 5:  case 7:  case 9:
		case 11: dev->println("bad number or end ([0-9]+ / 'END')"); break;
		case 13: case 15: case 17:
		case 19: dev->println("bad number ([0-9]+)"); break;
		case 20: dev->println("bad end of line (\\r \\n? / \\n)"); break;
		default: dev->println("unknown");
		}
	} else if (r >= 1 && r <= 20 + 100 * MAX_MAPS) {
		dev->print("Map Definition Error!\n\tline: ");
		dev->println((r / 100) + 1);
		dev->print("\terror: ");
		switch (r % 100) {
		case 2:  case 4: dev->println("bad comma"); break;
		case 1:  case 3: dev->println("bad floating number (-?[0-9]+(.[0-9]+)?)"); break;
		case 5:  dev->println("bad file name"); break;
		case 6:  dev->println("bad end of line (\\r \\n? / \\n)"); break;
		default: dev->println("unknown");
		}
	} else if (r < 0) {
		dev->print("File Error!\n\terror: ");
		switch (r) {
		case -1: dev->println("\tbad map def file name"); break;
		case -2: dev->println("\tbad waypoint def file name"); break;
		case -3: dev->println("\tunable to load map defs"); break;
		default: dev->println("\tunknown");
		}
	}
}

void Waypoint::write_as_SerialData(int32_t i, HardwareSerial *dev) {
	static SerialData dt;
	dt.clear();
	float angle    = atan2(north_x1k, east_x1k) * 180 / PI;
	dt.kind        = MsgType::seg;
	dt.number      = i;
	dt.posE_cm     = east_mm    / 10;
	dt.posN_cm     = north_mm   / 10;
	dt.speed_cmPs  = speed_mmPs / 10;
	dt.bearing_deg = (long)(-angle) + 90;
	dt.write(dev);
}

} // namespace elcano
