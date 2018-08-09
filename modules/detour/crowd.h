#include "detour.h"
#ifndef CROWD_H
#define CROWD_H
class DetourCrowdManager : public Spatial {
protected:
	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;

	void _notification(int p_what);
	void _update_visibility();
	static void _bind_methods();
public:
	DetourCrowdManager();
	~DetourCrowdManager();
	void set_navmesh(Ref<DetourNavigationMesh> &navmesh);
	void add_agent(Ref<Spatial> &agent, int mode);
};
#endif
