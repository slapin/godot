#include <cstdio>
#include <ctime>
#include <cassert>
#include <sys/time.h>
#include <scene/3d/skeleton.h>
#include <scene/main/node.h>
#include <scene/main/scene_tree.h>
#include <scene/main/viewport.h>
#include "skirt.h"

Skirt *Skirt::instance;

Skirt *Skirt::get_instance() {
	printf("instance is %p\n", instance);
	return instance;
}
void Skirt::add_constraint(int p1, int p2, float distance) {
	struct constraint c;
	c.p1 = p1;
	c.p2 = p2;
	c.distance = distance;
	constraints.push_back(c);
}
void Skirt::remove_constraint(int id) {
	constraints.remove(id);
}
/* looping on X but not Y */
void Skirt::create_constraints(int skeleton_id) {
	int i, j;
	for (i = 0; i < size_y + 1; i++) {
		for (j = 0; j < size_x; j++) {
			int base_p = i * size_x + j;
			if (j > 0) {
				float d = sim_hash[skeleton_id].distance(base_p, base_p - 1);
				add_constraint(base_p, base_p - 1, d * 1.1f);
			}	
			else
				add_constraint(base_p, base_p + size_x - 1, sim_hash[skeleton_id].distance(base_p, base_p + size_x - 1) * 1.1f);
			if (j < size_x - 1)
				add_constraint(base_p, base_p + 1, sim_hash[skeleton_id].distance(base_p, base_p + 1) * 1.1f);
			else
				add_constraint(base_p, base_p + 1 - size_x, sim_hash[skeleton_id].distance(base_p, base_p + 1 - size_x) * 1.1f);
			if (i > 0)
				add_constraint(base_p, base_p - size_x, sim_hash[skeleton_id].distance(base_p, base_p - size_x) * 1.1);
			if (i < size_y)
				add_constraint(base_p, base_p + size_x, sim_hash[skeleton_id].distance(base_p, base_p + size_x) * 1.1);
		}
	}
	printf("constraints: %d\n", constraints.size());
}

void Skirt::build_bone_list(int skeleton_id, List<int> *bones, List<int> *root_bones) {
	Skeleton *skel =
			Object::cast_to<Skeleton>(ObjectDB::get_instance(skeleton_id));
	if (skel) {
		for (int bone_id = 0; bone_id < skel->get_bone_count(); bone_id++) {
			if (skel->get_bone_name(bone_id).begins_with("skirt")) {
				int parent = skel->get_bone_parent(bone_id);
				if (parent < 0)
					root_bones->push_back(bone_id);
				else if (!skel->get_bone_name(parent).begins_with("skirt"))
					root_bones->push_back(bone_id);
				bones->push_back(bone_id);
			}
		}
	}
}
void Skirt::build_bone_chain(int skeleton_id, int root_bone, List<int> *chain) {
	Skeleton *skel =
			Object::cast_to<Skeleton>(ObjectDB::get_instance(skeleton_id));
	if (skel) {
		int i;
		chain->push_back(root_bone);
		int parent = root_bone;
		bool found = false;
		do {
			found = false;
			for (i = 0; i < skel->get_bone_count(); i++) {
				if (parent == skel->get_bone_parent(i)) {
					chain->push_back(i);
					found = true;
					parent = i;
					break;
				}
			}
		} while (found == true);
	}
}

void Skirt::verlet_init(int skeleton_id) {
	int i, j;
	Skeleton *skel =
			Object::cast_to<Skeleton>(ObjectDB::get_instance(skeleton_id));
	if (!skel)
		return;
	for (i = 0; i < size_y; i++) {
		for (j = 0; j < size_x; j++) {
			List<int> chain = bone_chains[j];
			if (chain.size() <= i)
				continue;
			int bone_id = bone_chains[j][i];
			//			int root_bone_id = bone_chains[j][0];
//			printf("skel: %d i = %d j = %d bone_id = %d\n", skeleton_id, i, j, bone_id);
			Transform pose = skel->get_bone_global_pose(bone_id);
			Vector3 pos = pose.origin;
			sim_hash[skeleton_id].set_particle(i * size_x + j, pos);
			sim_hash[skeleton_id].set_particle_prev(i * size_x + j, pos);
		}
	}
	for (j = 0; j < size_x; j++) {
		sim_hash[skeleton_id].set_particle(size_y * size_x + j,
			sim_hash[skeleton_id].get_particle((size_y - 1) * size_x + j) + Vector3(0, -0.05f, 0));
		sim_hash[skeleton_id].set_particle_prev(size_y * size_x + j,
			sim_hash[skeleton_id].get_particle_prev((size_y - 1) * size_x + j) + Vector3(0, -0.05f, 0));
	}
}

void Skirt::sort_chains(int skeleton_id) {
	int i;
	struct ChainCompare {
		Skeleton *skel;
		Vector3 midpoint;
		_FORCE_INLINE_ bool operator()(const List<int> &a, const List<int> &b) const {
			Transform pose1 = skel->get_bone_global_pose(a[0]);
			Transform pose2 = skel->get_bone_global_pose(b[0]);
			Vector3 dir1 = pose1.origin - midpoint;
			Vector3 dir2 = pose2.origin - midpoint;
			float angle1 = Vector2(dir1.x, dir1.z).angle();
			float angle2 = Vector2(dir2.x, dir2.z).angle();
			return angle1 < angle2;
		}
	};
	Skeleton *skel =
			Object::cast_to<Skeleton>(ObjectDB::get_instance(skeleton_id));
	if (!skel)
		return;
	Vector<Vector3> midpoints;
	for (i = 0; i < bone_chains.size(); i++) {
		const List<int> &chain = bone_chains[i];
		int chain_pos = 0;
		for (const List<int>::Element *ce = chain.front(); ce; ce = ce->next()) {
			if (midpoints.size() < chain_pos + 1)
				midpoints.resize(chain_pos + 1);
			int bone_id = ce->get();
			Transform pose = skel->get_bone_global_pose(bone_id);
			midpoints.write[chain_pos] += pose.origin;
			chain_pos++;
		}
	}
	for (i = 0; i < midpoints.size(); i++)
		midpoints.write[i] /= (float)bone_chains.size();
	SortArray<List<int>, ChainCompare> c;
	c.compare.skel = skel;
	c.compare.midpoint = midpoints[0];
	c.sort(bone_chains.ptrw(), bone_chains.size());
	size_x = bone_chains.size();
	size_y = midpoints.size();
	triangles.resize((size_x - 1) * (size_y) * 6);
	nodes.resize(size_x * (size_y + 1));
	int triangle_offset = 0;
	for (i = 0; i < size_y + 1; i++) {
		int j;
		for (j = 0; j < size_x; j++) {
			List<int> chain = bone_chains[j];
			if (chain.size() <= i)
				continue;
			int bone_id = bone_chains[j][i];
			if (i < size_y && j < size_x - 1) {
				triangles.write[triangle_offset++] = i * size_x + j + size_x;
				triangles.write[triangle_offset++] = i * size_x + j;
				triangles.write[triangle_offset++] = i * size_x + j + 1;
				triangles.write[triangle_offset++] = i * size_x + j + size_x;
				triangles.write[triangle_offset++] = i * size_x + j + 1;
				triangles.write[triangle_offset++] = i * size_x + j + size_x + 1;
			}
			int base_p = i * size_x + j;
			nodes.write[base_p] = bone_id;
		}
	}
}


void Skirt::constraints_step(int skeleton_id, float delta) {
	int i, j, k;
	Skeleton *skel =
			Object::cast_to<Skeleton>(ObjectDB::get_instance(skeleton_id));
	Vector<Vector3> pins;
	pins.resize(size_x);
	for (i = 0; i < size_x; i++) {
		int bone = bone_chains[i][0];
		Transform pin = skel->get_bone_global_pose(bone);
		pins.ptrw()[i] = pin.origin;
	}
	sim_hash[skeleton_id].constraints_step(delta,
		constraints.ptr(), constraints.size(),
		pins.ptr());
}

void Skirt::physics_process() {
	List<Node *> node_list;
	List<Node *>::Element *e;
	int i;

	SceneTree *sc = SceneTree::get_singleton();
	sc->get_nodes_in_group("characters", &node_list);
	List<int> skeletons;
	for (e = node_list.front(); e;
			e = e->next()) {
		Node *current = e->get();
		List<Node *> queue;
		if (!current->is_in_group("female"))
			continue;
		if (current->has_meta("skeleton_rid")) {
			Variant val = current->get_meta("skeleton_rid");
			int id = val;
			skeletons.push_back(id);
		} else {
			queue.push_back(current);
			while (!queue.empty()) {
				Node *item = queue.front()->get();
				Skeleton *skel = Object::cast_to<Skeleton>(item);
				if (skel) {
					int skel_rid = skel->get_instance_id();
					skeletons.push_back(skel_rid);
					current->set_meta("skeleton_rid", skel_rid);
					break;
				}
				queue.pop_front();
				for (i = 0; i < item->get_child_count(); i++)
					queue.push_back(item->get_child(i));
			}
		}
	}
	if (skeletons.empty())
		return;
	for (List<int>::Element *s = skeletons.front(); s; s = s->next()) {
		int skel_id = s->get();
		Skeleton *skel = Object::cast_to<Skeleton>(ObjectDB::get_instance(skel_id));
		if (skel) {
			List<int> bones_start;
			List<int> bones;
			if (bone_chains.empty()) {
				build_bone_list(skel_id, &bones, &bones_start);
				for (List<int>::Element *c = bones_start.front(); c; c = c->next()) {
					int bone_id = c->get();
					List<int> chain;
					build_bone_chain(skel_id, bone_id, &chain);
					if (chain.size() > 0)
						bone_chains.push_back(chain);
				}
				sort_chains(skel_id);
			}
			if (!sim_hash.has(skel_id)) {
				struct collider pelvis_l_col, pelvis_r_col, left_col, right_col;
				struct col_data {
					struct collider col;
					String b1;
					String b2;
					float h;
					float r;
					Vector3 cv;
				};
				printf("simhash %d, %d\n", skel_id, size_x * size_y);
				sim_hash[skel_id] = SkirtSimulation();
				sim_hash[skel_id].init(size_x, size_y + 1);
				static struct col_data cdata[] = {
					{pelvis_l_col, "pelvis_L", "upperleg01_L", 0.05f, 0.07f, Vector3(1.0, 1.0, 2.0)},
					{pelvis_r_col, "pelvis_R", "upperleg01_R", 0.05f, 0.07f, Vector3(1.0, 1.0, 2.0)},
					{left_col, "upperleg02_L", "lowerleg01_L", 0.3f, 0.06f, Vector3(1.0, 1.0, 0.99)},
					{right_col, "upperleg02_R", "lowerleg01_R", 0.3f, 0.06f, Vector3(1.0, 1.0, 0.99)},
				};
				for (i = 0;i < (int)(sizeof(cdata) / sizeof(cdata[0])); i++) {
					cdata[i].col.create_from_bone(skel, cdata[i].b1, cdata[i].b2, cdata[i].h, cdata[i].r, cdata[i].cv);
					assert(cdata[i].col.bone < skel->get_bone_count());
					sim_hash[skel_id].add_collider(cdata[i].col.bone, cdata[i].col);
				}
				assert(sim_hash[skel_id].colliders.size() == 4);
				verlet_init(skel_id);
				if (constraints.size() == 0) {
					create_constraints(skel_id);
					printf("constraints: %d\n", constraints.size());
				}
			}
			sim_hash[skel_id].forces_step(skel->get_physics_process_delta_time());
			sim_hash[skel_id].verlet_step(skel->get_physics_process_delta_time());
			constraints_step(skel_id, skel->get_physics_process_delta_time());
		}
	}
}

void Skirt::_bind_methods() {
	ClassDB::bind_method(D_METHOD("connect_signals"), &Skirt::connect_signals);
	ClassDB::bind_method(D_METHOD("physics_process"), &Skirt::physics_process);
}
void Skirt::connect_signals() {
	SceneTree *sc = SceneTree::get_singleton();
	sc->connect("physics_frame", this, "physics_process");
	Node *root = Object::cast_to<Node>(sc->get_root());
	if (root) {
		Node *skirt_update = new SkirtUpdate();
		root->add_child(skirt_update);
		skirt_update->set_name("skirt_update");
		printf("created bone updater\n");
		SkirtDebug *skirt_debug = new SkirtDebug();
		root->add_child(skirt_debug);
		skirt_debug->set_name("debug_draw");
		skirt_debug->set_as_toplevel(true);
	}
}
#define START_PROFILING(n) struct timeval n ## _tv_start; gettimeofday(&n ## _tv_start, NULL);
#define END_PROFILING(n) \
	struct timeval n ## _tv_end; gettimeofday(&n ## _tv_end, NULL); \
	float n ## _r1 = n ## _tv_end.tv_sec - n ## _tv_start.tv_sec; \
	float n ## _r2 = n ## _tv_end.tv_usec - n ## _tv_start.tv_usec; \
	float n ## _r = (float)n ## _r1 * 1000.0f + (float)n ## _r2 / 1000.0f; \
	printf("timing: %s %f\n", #n, n ## _r);
void Skirt::update_bones() {
	int i, j;
	int update_counter = 0;
	List<int> skeletons;
	if (sim_hash.empty())
		return;
	sim_hash.get_key_list(&skeletons);
	Vector<int> bone_chain_sizes;
	bone_chain_sizes.resize(bone_chains.size());
	int *sizes = bone_chain_sizes.ptrw();
	for (i = 0; i < bone_chains.size(); i++)
		sizes[i] = bone_chains[i].size();
	for (List<int>::Element *e = skeletons.front(); e; e = e->next()) {
		int skel_id = e->get();
		Skeleton *skel = Object::cast_to<Skeleton>(ObjectDB::get_instance(skel_id));
		if (!skel)
			continue;
		const float *p = get_particles(skel_id);

		for (i = 0; i < size_x; i++) {
			Transform pose, parent_pose, rest_pose, custom_pose;
			j = 0;
			for (const List<int>::Element *b = bone_chains[i].front(); b; b = b->next()) {
				Vector3 pos;
				int bone = b->get();
				if (j == 0) {
					int parent = skel->get_bone_parent(bone);
					parent_pose = get_parent_bone_transform(skel_id, parent);
				}
				pose = skel->get_bone_pose(bone);
				rest_pose = skel->get_bone_rest(bone);
				custom_pose = skel->get_bone_custom_pose(bone);
				pos.x = p[(j * size_x + i) * 3 + 0];
				pos.y = p[(j * size_x + i) * 3 + 1];
				pos.z = p[(j * size_x + i) * 3 + 2];
				Transform parent_inv = parent_pose.affine_inverse();
				Transform rest_inv = rest_pose.affine_inverse();
				Transform custom_inv = custom_pose.affine_inverse();
				Transform localize = custom_inv * rest_inv * parent_inv;
				Vector3 local_pos = localize.xform(pos);
				pose.origin = local_pos;
				Vector3 target;
				if (j < sizes[i]) {
					target.x = p[((j + 1) * size_x + i) * 3 + 0];
					target.y = p[((j + 1) * size_x + i) * 3 + 1];
					target.z = p[((j + 1) * size_x + i) * 3 + 2];
					Vector3 local_target = localize.xform(target);
					Vector3 local_up = localize.xform(Vector3(0.0f, 1.0f, 0.0f));
					pose = pose.looking_at(local_target, local_up);
				}
				skel->set_bone_pose(bone, pose);
				parent_pose = parent_pose * rest_pose * custom_pose * pose;
				update_counter++;
				j++;
			}
		}
		struct collider pelvis_col, left_col, right_col;
		sim_hash[skel_id].update_colliders(skel);
	}
}

int Skirt::get_next_bone(int chain, int chain_pos) {
	if (bone_chains.size() == 0)
		return -1;
	if (chain < 0 || chain >= bone_chains.size())
		return -1;
	int use_chain = (chain + 1) % bone_chains.size();
	const List<int> &chain_list = bone_chains[use_chain];
	if (chain_pos < 0 || chain_pos >= chain_list.size())
		return -1;
	return chain_list[chain_pos];
}

int Skirt::get_prev_bone(int chain, int chain_pos) {
	if (bone_chains.size() == 0)
		return -1;
	if (chain < 0 || chain >= bone_chains.size())
		return -1;
	int use_chain = (chain + size_x - 1) % bone_chains.size();
	const List<int> &chain_list = bone_chains[use_chain];
	if (chain_pos < 0 || chain_pos >= chain_list.size())
		return -1;
	return chain_list[chain_pos];
}

Skirt::Skirt(): Object(), stiffness(0.9) {
	instance = this;
	mutex = Mutex::create();
	call_deferred("connect_signals");
}

Skirt::~Skirt() {
	instance = NULL;
	memdelete(mutex);
}

Transform Skirt::get_parent_bone_transform(int skel_id, int bone)
{
		if (parent_bones.has(skel_id) && parent_bones[skel_id].has(bone))
			return parent_bones[skel_id][bone];
		Skeleton *skel = Object::cast_to<Skeleton>(ObjectDB::get_instance(skel_id));
		if (!skel)
			return Transform();
		Transform pose = skel->get_bone_global_pose(bone);
		parent_bones[skel_id][bone] = pose;
		return pose;
}

void SkirtUpdate::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			skirt = Skirt::get_instance();
			printf("instance %p\n", skirt);
			set_process_internal(true);
			set_process_priority(2);
			break;
		case NOTIFICATION_INTERNAL_PROCESS:
			if (skirt) {
				List<int> skeletons;
				skirt->update_bones();
				skirt->pinning_bones.clear();
				skirt->parent_bones.clear();
				skirt->sim_hash.get_key_list(&skeletons);
				for (List<int>::Element *e = skeletons.front(); e; e = e->next()) {
					int skel_id = e->get();
					Skeleton *skel = Object::cast_to<Skeleton>(ObjectDB::get_instance(skel_id));
					if (!skel)
						continue;
					for (int i = 0; i < skirt->size_x; i++) {
						int bone = skirt->bone_chains[i][0];
						int parent = skel->get_bone_parent(bone);
						Transform parent_pose = skel->get_bone_global_pose(parent);
						skirt->parent_bones[skel_id][parent] = parent_pose;
					}

				}
			}
			break;
		case NOTIFICATION_EXIT_TREE:
			set_process_internal(false);
			break;
	}
}
SkirtUpdate::SkirtUpdate() :
		Node(), skirt(NULL) {
}
Vector3 Skirt::get_particle(const float *p, int id) const
{
	Vector3 ret;
	ret.coord[0] = p[id * 3 + 0];
	ret.coord[1] = p[id * 3 + 1];
	ret.coord[2] = p[id * 3 + 2];
	return ret;
}
void Skirt::set_particle(float *p, int id, const Vector3 &pt)
{
	p[id * 3 + 0] = pt.coord[0];
	p[id * 3 + 1] = pt.coord[1];
	p[id * 3 + 2] = pt.coord[2];
}
const float *Skirt::get_particles(int skeleton_id) const
{
	return sim_hash[skeleton_id].get_particles();
}

float *Skirt::get_particles_w(int skeleton_id)
{
	return sim_hash[skeleton_id].get_particles();
}
const float *Skirt::get_particles_prev(int skeleton_id) const
{
	return sim_hash[skeleton_id].get_particles_prev();
}

float *Skirt::get_particles_prev_w(int skeleton_id)
{
	return sim_hash[skeleton_id].get_particles_prev();
}
