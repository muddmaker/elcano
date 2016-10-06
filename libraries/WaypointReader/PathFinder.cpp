#include <PathFinder.h>

namespace elcano {

static long heur_dist(Junction *map, int i, int j) {
	long dx = abs(map[i].east_mm  - map[j].east_mm);
	long dy = abs(map[i].north_mm - map[j].north_mm);
	return dx + dy
}

static int *reconstruct_path(Junction *map, int *came_from, int current) {
	int size = 0;
	for (int s = current; s != -1; s = came_from[s], ++size);
	int *r = new int[size];
	for (; current != -1; current = came_from[current], r[--size] = current);
	return r;
}

int *find_path(Junction *map, int map_size, int start, int end) {
	bool closed_set[map_size]; int closed_set_size = 0;
	bool open_set[map_size];   int open_set_size   = 0;
	int  came_from[map_size];
	long g_score[map_size];
	long f_score[map_size];
#define ADD_SET(set, i)    { if (!set[i]) ++set##_size; set[i] = true;  }
#define REMOVE_SET(set, i) { if ( set[i]) --set##_size; set[i] = false; }
	ADD_SET(open_set, start);
	for (int i = 0; i < map_size; ++i) { g_score[i] = NaN; f_score[i] = NaN; }
	g_score[start] = 0;
	f_score[start] = heur_dist(map, start, end);
	came_from[start] = -1;

	while (open_set_size != 0) {
		int current = 0;
		for (int i = 1, l = f_score[0]; i < map_size; ++i)
			if (f_score[i] < l && open_set[i]) { l = f_score[i]; current = i; }
		if (current == goal)
			return reconstruct_path(map, came_from, current);
		REMOVE_SET(open_set, current);
		ADD_SET(closed_set, current);
		for (int i = 0; i < 3; ++i) {
			int neighbor = map[current].destination[i];
			if (neighbor == END) break;
			if (closed_set[neighbor]) continue;
			long tentative_g_score = g_score[current] + heur_dist(map, current, neighbor);
			if (open_set[neighbor] && tentative_g_score >= g_score[neighbor]) continue;
			ADD_SET(open_set, neighbor);
			g_score[neighbor] = tentative_g_score;
			f_score[neighbor] = tentative_g_score + heur_dist(map, neighbor, end);
			came_from[neighbor] = current;
		}
	}
#undef ADD_SET
#undef REMOVE_SET
	return NULL;
}

}
