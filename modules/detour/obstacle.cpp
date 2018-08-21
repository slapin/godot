#include "obstacle.h"
#include "detour.h"
DetourNavigationObstacle::DetourNavigationObstacle() :
	radius(5.0f),
	height(5.0f),
	mesh(0),
	id(0)
{
}

DetourNavigationObstacle::~DetourNavigationObstacle()
{
	if (mesh && id > 0)
		mesh->remove_obstacle(id);
}
void DetourNavigationObstacle::_bind_methods()
{
}

