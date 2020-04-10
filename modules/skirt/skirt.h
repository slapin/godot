#include <core/object.h>
#include <scene/main/node.h>
#include <scene/3d/immediate_geometry.h>

class Skirt;
class Skeleton;

class SkirtDebug: public ImmediateGeometry {
	GDCLASS(SkirtDebug, ImmediateGeometry);
	friend class Skirt;
	Skirt *skirt;
	HashMap<int, Vector<int> > stale_collisions;
	void _notification(int p_what);
	void draw_debug(int skel_id);
public:
	SkirtDebug();
};

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
	Vector3 change;
	Vector3 toffset, toffset_mod;
	int bone, parent, end_bone;
	float h;
	float radius;
	void create_from_bone(const Skeleton *skel,
		const String &bone,
		const String &end_bone,
		float height, float r,
		const Vector3 &cv = Vector3(1, 1, 1),
		const Vector3 &offset = Vector3());
	void update(const Skeleton *skel);
	bool is_colliding(Vector3 p, Vector3 *penetration);
	Vector3 p1, p2;
};

class SkirtSimulation {
	int skeleton_id;
	Vector<float> particles;
	Vector<float> particles_prev;
	Vector<float> accel;
protected:
	float damping;
	float stiffness;
	int size_x, size_y;
	Vector3 gravity;
	Vector3 external_pos_prev;
public:
	HashMap<int, struct collider> colliders;
	List<int> debug_penetration_list;

	SkirtSimulation();
	const float *get_particles() const;
	float *get_particles();
	const float *get_particles_prev() const;
	float *get_particles_prev();
	const float *get_accel() const;
	float *get_accel();
	void init(int size_x, int size_y);
	void verlet_step(float delta);
	void forces_step(float delta, const Transform &external_pos);
	void constraints_step(float delta,
		const struct constraint *c,
		int count, const Vector3 *pin);
	void set_particle(int id, const Vector3 &pt);
	void set_particle_prev(int id, const Vector3 &pt);
	Vector3 get_particle(int id);
	Vector3 get_particle_prev(int id);
	float distance(int p1, int p2);
	void add_collider(int bone, struct collider &col);
	void update_colliders(const Skeleton *skel);
	void process_collisions();
	inline int get_size_x() const
	{
		return size_x;
	}
	inline int get_size_y() const
	{
		return size_y;
	}
};

class Skirt: public Object {
	GDCLASS(Skirt, Object);
public:
	static Skirt *get_instance();
	Skirt();
	~Skirt();
	void physics_process();
	void connect_signals();
	float stiffness;
private:
	static Skirt *instance;
	Mutex *mutex;
protected:
	HashMap<int, SkirtSimulation> sim_hash;
	int size_x, size_y;
	struct collider *create_from_bones(int joint_bone);
	void add_constraint(int p1, int p2, float distance);
	void remove_constraint(int id);
	void create_constraints(int skeleton_id);
	static void _bind_methods();
	Vector<struct constraint> constraints;
	Vector<List<int> > bone_chains;
	HashMap<int, Transform> facing;
	Vector<int> triangles;
	Vector<int> nodes;
	HashMap <int, HashMap<int, Transform> > pinning_bones;
	HashMap <int, HashMap<int, Transform> > parent_bones;
	void build_bone_list(int skeleton_id, List<int> *bones, List<int> *root_bones);
	void build_bone_chain(int skeleton_id, int root_bone, List<int> *chain);
	int get_next_bone(int chain, int chain_pos);
	int get_prev_bone(int chain, int chain_pos);
	void sort_chains(int skeleton_id);
	void verlet_init(int skeleton_id);
	void constraints_step(int skeleton_id, float delta);
	friend class SkirtUpdate;
	friend class SkirtDebug;
	void update_bones();
	Transform get_parent_bone_transform(int skel_id, int bone);
	Transform get_pinning_bone_transform(int skel_id, int bone);
	Vector3 get_particle(const float *, int id) const;
	void set_particle(float *particles, int id, const Vector3 &p);
	float * get_particles_w(int skeleton_id);
	const float * get_particles(int skeleton_id) const;
	float * get_particles_prev_w(int skeleton_id);
	const float * get_particles_prev(int skeleton_id) const;
};

class SkirtUpdate: public Node {
	GDCLASS(SkirtUpdate, Node);
	friend class Skirt;
	Skirt *skirt;
	void _notification(int p_what);
public:
	SkirtUpdate();
};
