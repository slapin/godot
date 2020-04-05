#include <core/object.h>
#include <scene/main/node.h>

class btSoftBody;

class Skirt: public Object {
	GDCLASS(Skirt, Object);
public:
	static Skirt *get_instance();
	Skirt();
	~Skirt();
	void physics_process();
	void connect_signals();
private:
	static Skirt *instance;
	btSoftBody *bt_soft_body;
	Mutex *mutex;
protected:
	int size_x, size_y;
	struct constraint {
		float distance;
		int p1;
		int p2;
	};
	void add_constraint(int p1, int p2, float distance);
	void remove_constraint(int id);
	void create_constraints(int skeleton_id);
	static void _bind_methods();
	Vector<struct constraint> constraints;
	Vector<List<int> > bone_chains;
	HashMap<int, Vector3> facing;
	Vector<int> triangles;
	Vector<int> nodes;
	HashMap<int, Vector<float> > particles;
	HashMap<int, Vector<float> > particles_prev;
	HashMap<int, Vector<float> > accel;
	void build_bone_list(int skeleton_id, List<int> *bones, List<int> *root_bones);
	void build_bone_chain(int skeleton_id, int root_bone, List<int> *chain);
	void sort_chains(int skeleton_id);
	void verlet_init(int skeleton_id);
	void verlet_step(int skeleton_id, float delta);
	void forces_step(int skeleton_id, float delta);
	void constraints_step(int skeleton_id, float delta);
	float distance(int skeleton_id, int p1, int p2);
	friend class SkirtUpdate;
	void update_bones();
};

class SkirtUpdate: public Node {
	GDCLASS(SkirtUpdate, Node);
	friend class Skirt;
	Skirt *skirt;
	void _notification(int p_what);
public:
	SkirtUpdate();
};
