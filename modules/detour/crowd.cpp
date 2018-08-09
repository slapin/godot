#include "crowd.h"

DetourCrowdManager::DetourCrowdManager():
	Spatial()
{
}
void DetourCrowdManager::set_navmesh(Ref<DetourNavigationMesh> &navmesh)
{
}
void DetourCrowdManager::add_agent(Ref<Spatial> &agent, int mode)
{
}
bool DetourCrowdManager::_set(const StringName &p_name, const Variant &p_value) {
	String name = p_name;
	return false;
}
bool DetourCrowdManager::_get(const StringName &p_name, Variant &r_ret) const {
	String name = p_name;
	return false;
}
void DetourCrowdManager::_get_property_list(List<PropertyInfo> *p_list) const {
	p_list->push_back(PropertyInfo(Variant::NODE_PATH, "add-spatial", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Spatial"));
}
