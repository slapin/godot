#include <core/object.h>
#include <scene/main/node.h>
#include <scene/3d/immediate_geometry.h>

class Skirt;
class Skeleton;

class SkirtDebug: public ImmediateGeometry {
	GDCLASS(SkirtDebug, ImmediateGeometry);
	friend class Skirt;
	Skirt *skirt;
	void _notification(int p_what);
	void draw_debug(int skel_id);
public:
	SkirtDebug();
};

class Skirt: public Object {
	GDCLASS(Skirt, Object);
public:
	static Skirt *get_instance();
	Skirt();
	~Skirt();
	void physics_process();
	void connect_signals();
	float damping;
	float stiffness;
private:
	static Skirt *instance;
	Mutex *mutex;
protected:
	int size_x, size_y;
	struct constraint {
		float distance;
		int p1;
		int p2;
	};
	struct collider {
		StringName name;
		Transform xform, xform_parent, xform_rest, xform_custom;
		Vector3 offset;
		Vector3 end_offset;
		int bone, parent, end_bone;
		float h;
		float radius;
		void create_from_bone(Skeleton *skel, const String &bone, const String &end_bone, float height, float r);
		void update(Skeleton *skel);
		Vector3 p1, p2;
	};
	HashMap<int, HashMap<int, struct collider> > colliders;
	struct collider *create_from_bones(int joint_bone);
	int pelvis_bone;
	int left_bone;
	int right_bone;
	void add_constraint(int p1, int p2, float distance);
	void remove_constraint(int id);
	void create_constraints(int skeleton_id);
	static void _bind_methods();
	Vector<struct constraint> constraints;
	Vector<List<int> > bone_chains;
	HashMap<int, Transform> facing;
	Vector<int> triangles;
	Vector<int> nodes;
	HashMap<int, Vector<float> > particles;
	HashMap<int, Vector<float> > particles_prev;
	HashMap<int, Vector<float> > accel;
	HashMap <int, HashMap<int, Transform> > pinning_bones;
	HashMap <int, HashMap<int, Transform> > parent_bones;
	void build_bone_list(int skeleton_id, List<int> *bones, List<int> *root_bones);
	void build_bone_chain(int skeleton_id, int root_bone, List<int> *chain);
	int get_next_bone(int chain, int chain_pos);
	int get_prev_bone(int chain, int chain_pos);
	void sort_chains(int skeleton_id);
	void verlet_init(int skeleton_id);
	void verlet_step(int skeleton_id, float delta);
	void forces_step(int skeleton_id, float delta);
	void constraints_step(int skeleton_id, float delta);
	float distance(int skeleton_id, int p1, int p2);
	friend class SkirtUpdate;
	friend class SkirtDebug;
	void update_bones();
	Transform get_parent_bone_transform(int skel_id, int bone);
	Transform get_pinning_bone_transform(int skel_id, int bone);
};

class SkirtUpdate: public Node {
	GDCLASS(SkirtUpdate, Node);
	friend class Skirt;
	Skirt *skirt;
	void _notification(int p_what);
public:
	SkirtUpdate();
};
