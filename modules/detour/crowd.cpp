#include "crowd.h"
#include <DetourCrowd.h>

#define MAX_AGENTS 20

DetourCrowdManager::DetourCrowdManager():
	Spatial(),
	crowd(0),
	dirty(false),
	initialized(false),
	max_agent_radius(0.0f),
	max_agents(20)
{
}
DetourCrowdManager::~DetourCrowdManager()
{
	if(crowd) {
		print_line("freeing crowd");
		if (initialized)
			dtFreeCrowd(crowd);
		else
			dtFree(crowd);
	}
}
void DetourCrowdManager::set_navigation_mesh(const Ref<DetourNavigationMesh> &navmesh)
{
	this->navmesh = navmesh;
	create_crowd();
}
Ref<DetourNavigationMesh> DetourCrowdManager::get_navigation_mesh() const
{
	return navmesh;
}
void DetourCrowdManager::add_agent(Object *agent, int mode)
{
	Spatial *obj = Object::cast_to<Spatial>(agent);
	if (!obj)
		return;
	AgentData *new_agent = memnew(AgentData);
	new_agent->obj = obj;
	new_agent->mode = mode;
	Vector3 pos = obj->get_global_transform().origin;
	dtCrowdAgentParams params{};
	agents.push_back(new_agent);
	params.userData = new_agent;
	int id = crowd->addAgent(&pos.coord[0], &params);
	new_agent->id = id;

	if (max_agents < agents.size()) {
		max_agents = agents.size() + 20;
		create_crowd();
	}
	print_line("add_agent");
}
void DetourCrowdManager::remove_agent(Object *agent)
{
	Spatial *obj = Object::cast_to<Spatial>(agent);
	if (obj)
		return;
	for (int i = 0; i < agents.size(); i++)
		if (agents[i]->obj == obj) {
			agents.remove(i);
			/* remove from crowd too */
			break;
		}
}
void DetourCrowdManager::clear_agent_list()
{
	agents.clear();
}
void DetourCrowdManager::set_target(const Vector3& position)
{
}
void DetourCrowdManager::set_velocity(const Vector3& position)
{
}
void DetourCrowdManager::reset_target()
{
}
void DetourCrowdManager::set_max_agents(int max_agents)
{
	this->max_agents = max_agents;
	create_crowd();
}
void DetourCrowdManager::set_max_agent_radius(float radius)
{
	max_agent_radius = radius;
	create_crowd();
}
void DetourCrowdManager::process_agent(dtCrowdAgent *agent)
{
	Vector3 position;
	Vector3 velocity;
	Vector3 desired_velocity;
	memcpy(&position, agent->npos, sizeof(float) * 3);
	memcpy(&velocity, agent->vel, sizeof(float) * 3);
	memcpy(&desired_velocity, agent->dvel, sizeof(float) * 3);
	dtCrowdAgentParams params = agent->params;
	AgentData *data = (AgentData *)params.userData;
	if (!data || !data->obj)
		return;
	if (!data->obj->is_inside_tree())
		return;
	Transform transform = data->obj->get_global_transform();
	if (velocity.length() == 0.0f)
		velocity = transform.basis[2];
	data->obj->look_at_from_position(position, position + velocity, Vector3(0, 1, 0));
}
Vector3 DetourCrowdManager::_find_nearest_point(const Vector3 &point, int query_filter, uint64_t *nearest_ref)
{
	if (!navmesh.is_valid() || !crowd)
		return point;
	dtPolyRef nearestRef = 0;
	dtNavMesh *dtnav = navmesh->get_navmesh();
	Vector3 ret;
	if (!dtnav) {
		*nearest_ref = (uint64_t)nearestRef;
		return point;
	}
//	Vector3 ret =  navmesh->find_nearest_point(point, Vector3(*reinterpret_cast<const Vector3 *>(crowd->getQueryExtents())), crowd->getFilter(query_filter), &nearestRef);
//	*nearest_ref = (uint64_t)nearestRef;
	return ret;
}
void DetourCrowdManager::set_agent_target_position(int id, const Vector3 &position)
{
	uint64_t pref;
	Vector3 close = _find_nearest_point(position, 0, &pref);
	dtPolyRef nearestRef = (dtPolyRef)pref;
	crowd->requestMoveTarget(agents[id]->id, nearestRef, &close.coord[0]);

}
void DetourCrowdManager::_notification(int p_what) {
	switch(p_what) {
		case NOTIFICATION_READY:
			print_line("ready");
			create_crowd();
			set_process(true);
			break;
		case NOTIFICATION_ENTER_TREE:
			print_line("enter tree");
			create_crowd();
			break;
		case NOTIFICATION_TRANSFORM_CHANGED:
			break;
		case NOTIFICATION_EXIT_TREE:
			print_line("exit tree");
			agents.clear();
			break;
		case NOTIFICATION_PROCESS:
			float delta = get_process_delta_time();
			// update_crowd(delta);
			if (crowd && navmesh.is_valid() && agents.size() > 0) {
				crowd->update(delta, NULL);
				Vector<dtCrowdAgent *> active_agents;
				active_agents.resize(agents.size());
				int nactive = crowd->getActiveAgents(&active_agents.write[0], agents.size());
				for (int i = 0; i < nactive; i++)
					process_agent(active_agents[i]);
			}
			break;
	}
}
void DetourCrowdManager::update_crowd(float delta)
{
	if (dirty)
		create_crowd();
}
bool DetourCrowdManager::create_crowd()
{
	dirty = true;
	if (!navmesh.is_valid())
		return false;
	print_line("navmesh valid");
	if (crowd) {
		print_line("freeing crowd");
		if (initialized)
			dtFreeCrowd(crowd);
		else
			dtFree(crowd);
		print_line("freed crowd");
		initialized = false;
	}
	print_line("befoore alloc crowd");
	crowd = dtAllocCrowd();
	print_line("created crowd");
	if (max_agent_radius == 0.0f)
		max_agent_radius = navmesh->get_agent_radius();
	print_line("max agent randius: " + rtos(max_agent_radius));
	if (navmesh->get_navmesh() != NULL)
		print_line("good navmesh");
	else
		print_line("bad navmesh");
	if (!crowd->init(max_agents, max_agent_radius, navmesh->get_navmesh()))
		return false;
	dirty = false;
	initialized = true;
	return true;
}
void DetourCrowdManager::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_navigation_mesh", "navmesh"), &DetourCrowdManager::set_navigation_mesh);
	ClassDB::bind_method(D_METHOD("get_navigation_mesh"), &DetourCrowdManager::get_navigation_mesh);
	ClassDB::bind_method(D_METHOD("add_agent", "agent"), &DetourCrowdManager::add_agent);
	ClassDB::bind_method(D_METHOD("remove_agent", "agent"), &DetourCrowdManager::remove_agent);
	ClassDB::bind_method(D_METHOD("clear_agent_list"), &DetourCrowdManager::clear_agent_list);
	ClassDB::bind_method(D_METHOD("get_agent_obj", "id"), &DetourCrowdManager::get_agent_obj);
	ClassDB::bind_method(D_METHOD("get_agent_mode", "id"), &DetourCrowdManager::get_agent_mode);
	ClassDB::bind_method(D_METHOD("get_agent_count"), &DetourCrowdManager::get_agent_count);
	ClassDB::bind_method(D_METHOD("set_target", "position"), &DetourCrowdManager::set_target);
	ClassDB::bind_method(D_METHOD("set_velocity", "velocity"), &DetourCrowdManager::set_velocity);
	ClassDB::bind_method(D_METHOD("reset_target"), &DetourCrowdManager::reset_target);
	ClassDB::bind_method(D_METHOD("set_max_agents", "max_agents"), &DetourCrowdManager::set_max_agents);
	ClassDB::bind_method(D_METHOD("get_max_agents"), &DetourCrowdManager::get_max_agents);
	ClassDB::bind_method(D_METHOD("set_max_agent_radius", "max_agent_radius"), &DetourCrowdManager::set_max_agent_radius);
	ClassDB::bind_method(D_METHOD("get_max_agent_radius"), &DetourCrowdManager::get_max_agent_radius);
}

// void DetourCrowdManager::agent_update_cb(dtCrowdAgent *ag, float dt)
//{
//}
bool Crowd::_set(const StringName &p_name, const Variant &p_value) {
	print_line("_set");
	if (!manager)
		return false;
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
		update_agent_list();
		_change_notify();
		return true;
	} else if (name == "nav_mesh") {
		if (p_value.get_type() == Variant::NODE_PATH) {
			NodePath path = p_value;
			DetourNavigationMeshInstance *nmi = (DetourNavigationMeshInstance*)get_node(path);
			if (nmi) {
				manager->set_navigation_mesh(nmi->get_navmesh());
				_change_notify();
				print_line("navmesh set from path");
				return true;
			}
		} else if (p_value.get_type() == Variant::OBJECT) {
			Ref<Resource> ov = p_value;
			if (ov.is_valid()) {
				manager->set_navigation_mesh((Ref<DetourNavigationMesh>)ov);
				print_line("navmesh set from resource");
				_change_notify();
				return true;
			}
		} else
			print_line(String() + "type: " + itos(p_value.get_type()));
		return false;
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
		} else if (what == "mode") {
			int mode = p_value;
			if (modes.size() > idx)
				modes.write[idx] = mode;
			else
				modes.push_back(mode);
		} else if (what == "remove") {
			bool rm = p_value;
			if (rm) {
				agent_paths.remove(idx);
				modes.remove(idx);
			}
		}
		_change_notify();
		update_agent_list();
		return true;
	}
	return false;
}
bool Crowd::_get(const StringName &p_name, Variant &r_ret) const {
	print_line("_get");
	if (!manager)
		return false;
	String name = p_name;
	if (name == "nav_mesh") {
		r_ret = manager->get_navigation_mesh();
		return true;
	} else if (name.begins_with("agents")) {
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
		} else if (what == "remove") {
			r_ret = false;
			return true;
		}
	}
	return false;
}
void Crowd::_get_property_list(List<PropertyInfo> *p_list) const {
	print_line("_get_property_list");
	if (!manager)
		return;
	if (manager->get_agent_count() > 0) {
		for (int i = 0; i < agent_paths.size(); i++) {
			p_list->push_back(PropertyInfo(Variant::NODE_PATH, "agents/" + itos(i) + "/path", PROPERTY_HINT_NONE, ""));
			p_list->push_back(PropertyInfo(Variant::INT, "agents/" + itos(i) + "/mode", PROPERTY_HINT_ENUM, "normal,signal"));
			p_list->push_back(PropertyInfo(Variant::BOOL, "agents/" + itos(i) + "/remove", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR));
		}
	}
	p_list->push_back(PropertyInfo(Variant::NODE_PATH, "add_object", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Spatial"));
	if (!manager->get_navigation_mesh().is_valid())
		p_list->push_back(PropertyInfo(Variant::NODE_PATH, "nav_mesh", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "DetourNavigationMeshInstance"));
	else
		p_list->push_back(PropertyInfo(Variant::OBJECT, "nav_mesh", PROPERTY_HINT_RESOURCE_TYPE, "DetourNavigationMesh"));
}

Crowd::Crowd() :
	manager(0)
{
}
Crowd::~Crowd()
{
}

void Crowd::_notification(int p_what) {
	switch(p_what) {
		case NOTIFICATION_READY:
			print_line("a");
			if (!manager)
				return;
			else
				update_agent_list();
			break;
		case NOTIFICATION_ENTER_TREE:
			print_line("b");
			manager = Object::cast_to<DetourCrowdManager>(get_parent());
			if (!manager)
				return;
			else {
				print_line("manager set");
				update_agent_list();
			}
			break;
		case NOTIFICATION_EXIT_TREE:
			print_line("c");
			manager = NULL;
			break;
		case NOTIFICATION_PROCESS:
			float delta = get_process_delta_time();
			// update_crowd(delta);
			break;
	}
}
String Crowd::get_configuration_warning()
{
	String ret;
	print_line("get_configuration_warning");
	if (!is_inside_tree())
		return ret;
	if (!manager)
		ret += TTR("Incorrect instancing. ");
	if (!Object::cast_to<DetourCrowdManager>(get_parent()))
		ret += TTR("Should be parented to DetourCrowdManager. ");
	if (manager && !manager->get_navigation_mesh().is_valid())
		ret += TTR("No navmesh data are set to function. ");
	return ret;
}
void Crowd::update_agent_list()
{
	if (!is_inside_tree())
		return;
	if (!manager)
		return;
	print_line("update_agent_list");
	manager->clear_agent_list();
	print_line("update_agent_list 1");
	for (int i = 0; i < agent_paths.size(); i++) {
		print_line("update_agent_list 2: " + itos(i));
		if (String(agent_paths[i]).length() > 0) {
			print_line("update_agent_list 3: " + itos(i));
			Spatial *obj = (Spatial *)get_node(agent_paths[i]);
			print_line("update_agent_list 4: " + itos(i));
			if (obj) {
				print_line("update_agent_list 5: " + itos(i));
				manager->add_agent(obj, modes[i]);
				print_line("object added ok 0");
			}
			print_line("update_agent_list 6: " + itos(i));
		} else {
			print_line("update_agent_list 7: " + itos(i));
			manager->add_agent(NULL, modes[i]);
		}
	}
	print_line("update_agent_list done");
}
void Crowd::_bind_methods()
{
}

