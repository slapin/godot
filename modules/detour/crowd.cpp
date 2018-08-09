#include "crowd.h"

DetourCrowdManager::DetourCrowdManager():
	Spatial()
{
}
DetourCrowdManager::~DetourCrowdManager()
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
	print_line(String() + "setting " + name);
	if (name == "add_object") {
		if (p_value.get_type() == Variant::NIL)
			return false;
		String path = p_value;
		NodePath npname = p_value;
		print_line(String() + "setting spatial " + path);
		agent_paths.push_back(npname);
		modes.push_back(0);
		print_line(String() + "agent count: " + itos(agent_paths.size()));
		agents.clear();
		for (int i = 0; i < agent_paths.size(); i++)
		{
			Spatial *obj = (Spatial *)get_node(agent_paths[i]);
			if (obj)
				agents.push_back(obj);
		}
		return true;
	} else if (name.begins_with("agents")) {
		int idx = name.get_slice("/", 1).to_int();
		String what = name.get_slice("/", 2);
		if (what == "path") {
			if (agent_paths.size() > idx) {
				NodePath path = p_value;
				agent_paths.write[idx] = path;
			} else {
				NodePath path = p_value;
				agent_paths.push_back(path);
			}
			agents.clear();
			for (int i = 0; i < agent_paths.size(); i++)
			{
				Spatial *obj = (Spatial *)get_node(agent_paths[i]);
				if (obj)
					agents.push_back(obj);
			}
			return true;
		} else if (what == "mode") {
			int mode = p_value;
			if (modes.size() > idx)
				modes.write[idx] = mode;
			else
				modes.push_back(mode);
		}
	}
	return false;
}
bool DetourCrowdManager::_get(const StringName &p_name, Variant &r_ret) const {
	String name = p_name;
	if (name.begins_with("agents")) {
		int idx = name.get_slice("/", 1).to_int();
		String what = name.get_slice("/", 2);
		if (what == "path") {
			r_ret = agent_paths[idx];
			return true;
		} else if (what == "mode") {
			if (modes.size() > idx)
				r_ret = modes[idx];
			else
				r_ret = 0;
			return true;
		}
	}
	return false;
}
void DetourCrowdManager::_get_property_list(List<PropertyInfo> *p_list) const {
	if (agents.size() > 0) {
		for (int i = 0; i < agent_paths.size(); i++) {
			p_list->push_back(PropertyInfo(Variant::NODE_PATH, "agents/" + itos(i) + "/path", PROPERTY_HINT_NONE, ""));
			p_list->push_back(PropertyInfo(Variant::INT, "agents/" + itos(i) + "/mode", PROPERTY_HINT_ENUM, "normal,signal"));
		}
	}
	p_list->push_back(PropertyInfo(Variant::NODE_PATH, "add_object", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Spatial"));
}
void DetourCrowdManager::_notification(int p_what) {
	switch(p_what) {
		case NOTIFICATION_ENTER_WORLD:
			agents.clear();
			for (int i = 0; i < agent_paths.size(); i++)
			{
				Spatial *obj = (Spatial *)get_node(agent_paths[i]);
				if (obj)
					agents.push_back(obj);
			}
			break;
		case NOTIFICATION_TRANSFORM_CHANGED:
		case NOTIFICATION_EXIT_WORLD:
			agents.clear();
			break;
	}
}
void DetourCrowdManager::_bind_methods()
{
}

