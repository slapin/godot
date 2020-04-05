#include <cstdio>

#include <BulletSoftBody/btSoftBodyHelpers.h>

#include "skirt.h"
#include <scene/3d/skeleton.h>
#include <scene/main/node.h>
#include <scene/main/scene_tree.h>
#include <scene/main/viewport.h>

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
	for (i = 0; i < size_y; i++) {
		for (j = 0; j < size_x; j++) {
			int base_p = i * size_x + j;
			if (j > 0)
				add_constraint(base_p, base_p - 1, distance(skeleton_id, base_p, base_p - 1));
			else
				add_constraint(base_p, base_p + size_x - 1, distance(skeleton_id, base_p, base_p + size_x - 1));
			if (j < size_x - 1)
				add_constraint(base_p, base_p + 1, distance(skeleton_id, base_p, base_p + 1));
			else
				add_constraint(base_p, base_p + 1 - size_x, distance(skeleton_id, base_p, base_p + 1 - size_x));
			if (i > 0)
				add_constraint(base_p, base_p - size_x, distance(skeleton_id, base_p, base_p - size_x));
			if (i < size_y - 1)
				add_constraint(base_p, base_p + size_x, distance(skeleton_id, base_p, base_p + size_x));
		}
	}
}

float Skirt::distance(int skeleton_id, int p1, int p2) {
	Vector3 vp1;
	Vector3 vp2;
	memcpy(&vp1.coord[0], &particles[skeleton_id].ptr()[p1 * 3], sizeof(float) * 3);
	memcpy(&vp2.coord[0], &particles[skeleton_id].ptr()[p2 * 3], sizeof(float) * 3);
	return vp1.distance_to(vp2);
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
void Skirt::build_facing_data(int skeleton_id) {
	int i;
	Skeleton *skel =
			Object::cast_to<Skeleton>(ObjectDB::get_instance(skeleton_id));
	if (!skel)
		return;
	Vector<Vector3> midpoints;
	for (i = 0; i < bone_chains.size(); i++) {
		const List<int> &chain = bone_chains[i];
		int chain_pos = 0;
		for (const List<int>::Element *ce = chain.front(); ce; ce = ce->next()) {
			int bone = ce->get();
			int next_bone = -1;
			if (ce->next())
				next_bone = ce->next()->get();
			Transform pose, next_pose, result;
			pose = skel->get_bone_global_pose(bone);
			if (next_bone >= 0) {
				next_pose = skel->get_bone_global_pose(next_bone);
				Vector3 dir = next_pose.origin - pose.origin;
				int cprev_bone = get_prev_bone(i, chain_pos);
				int cnext_bone = get_next_bone(i, chain_pos);
				Transform lpose = skel->get_bone_global_pose(cprev_bone);
				Transform rpose = skel->get_bone_global_pose(cnext_bone);
				Vector3 left1 = lpose.origin - pose.origin;
				Vector3 left2 = pose.origin - rpose.origin;
				Vector3 left = left1.linear_interpolate(left2, 0.5f);
				Vector3 up = -dir.cross(left);
				result.origin = pose.origin;
				result.basis[0] = left.normalized();
				result.basis[1] = up.normalized();
				result.basis[2] = -dir.normalized();
				result = pose.looking_at(next_pose.origin, Vector3(0.0f, 1.0f, 0.0f));
				int parent = skel->get_bone_parent(bone);
				Transform parent_pose = skel->get_bone_global_pose(parent);
				Transform parent_inv = parent_pose.affine_inverse();
				facing[bone] = parent_inv * result;
			}
			chain_pos++;
		}
	}
}

void Skirt::verlet_init(int skeleton_id) {
	int i;
	Skeleton *skel =
			Object::cast_to<Skeleton>(ObjectDB::get_instance(skeleton_id));
	if (!skel)
		return;
	particles[skeleton_id].resize(size_x * size_y * 3);
	particles_prev[skeleton_id].resize(size_x * size_y * 3);
	accel[skeleton_id].resize(size_x * size_y * 3);
	for (i = 0; i < size_y; i++) {
		int j;
		for (j = 0; j < size_x; j++) {
			List<int> chain = bone_chains[j];
			if (chain.size() <= i)
				continue;
			int bone_id = bone_chains[j][i];
			//			int root_bone_id = bone_chains[j][0];
			printf("skel: %d i = %d j = %d bone_id = %d\n", skeleton_id, i, j, bone_id);
			Transform pose = skel->get_bone_global_pose(bone_id);
			//			Transform root_pose = skel->get_bone_global_pose(root_bone_id);
			//			pose = pose * root_pose.affine_inverse();
			Vector3 pos = pose.origin;
			particles[skeleton_id].write[(i * size_x + j) * 3 + 0] = pos.x;
			particles[skeleton_id].write[(i * size_x + j) * 3 + 1] = pos.y;
			particles[skeleton_id].write[(i * size_x + j) * 3 + 2] = pos.z;
			particles_prev[skeleton_id].write[(i * size_x + j) * 3 + 0] = pos.x;
			particles_prev[skeleton_id].write[(i * size_x + j) * 3 + 1] = pos.y;
			particles_prev[skeleton_id].write[(i * size_x + j) * 3 + 2] = pos.z;
		}
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
	triangles.resize((size_x - 1) * (size_y - 1) * 6);
	nodes.resize(size_x * size_y);
	int triangle_offset = 0;
	printf("size: %d x %d\n", size_x, size_y);
	for (i = 0; i < size_y; i++) {
		int j;
		for (j = 0; j < size_x; j++) {
			List<int> chain = bone_chains[j];
			if (chain.size() <= i)
				continue;
			int bone_id = bone_chains[j][i];
			if (i < size_y - 1 && j < size_x - 1) {
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

void Skirt::verlet_step(int skeleton_id, float delta) {
	Vector<float> *p = &particles[skeleton_id];
	Vector<float> *pp = &particles_prev[skeleton_id];
	Vector<float> *acc = &accel[skeleton_id];
	for (int i = 0; i < (*p).size(); i++) {
		float x = (*p)[i];
		assert(!isinf(x));
		float temp = x;
		float oldx = (*pp)[i];
		float a = (*acc)[i];
		assert(!isnan(a));
		assert(!isnan(oldx));
		assert(!isnan(temp));
		x += x - oldx + a * delta * delta;
		assert(!isnan(x));
		assert(!isinf(x));
		oldx = temp;
		(*p).write[i] = x;
		(*pp).write[i] = oldx;
	}
}

void Skirt::forces_step(int skeleton_id, float delta) {
	const float gravity[] = { 0.0f, -9.8f, 0.0f };
	int i;
	for (i = 0; i < (int)accel[skeleton_id].size(); i += 3) {
		accel[skeleton_id].write[i] = gravity[0];
		accel[skeleton_id].write[i + 1] = gravity[1];
		accel[skeleton_id].write[i + 2] = gravity[2];
		assert(!isnan(accel[skeleton_id][i + 0]));
		assert(!isnan(accel[skeleton_id][i + 1]));
		assert(!isnan(accel[skeleton_id][i + 2]));
	}
}
void Skirt::constraints_step(int skeleton_id, float delta) {
	int i, j, k;
	Skeleton *skel =
			Object::cast_to<Skeleton>(ObjectDB::get_instance(skeleton_id));
	for (k = 0; k < 2; k++) {
		for (i = 0; i < constraints.size();i++) {
			float x0[3], x1[3];
			float dx1[3], l1, d1, diff1;
			int p0, p1;
			p0 = constraints[i].p1;
			p1 = constraints[i].p2;
//			printf("p0 %d p1 %d\n", p0, p1);
			for (j = 0; j < 3; j++) {
				x0[j] = particles[skeleton_id][p0 * 3 + j];
				x1[j] = particles[skeleton_id][p1 * 3 + j];
				dx1[j] = x1[j] - x0[j];
#ifdef DEBUG_NANS
				assert(!isnan(x0[j]));
				assert(!isnan(x1[j]));
				assert(!isnan(dx1[j]));
				assert(!isinf(x0[j]));
				assert(!isinf(x1[j]));
				assert(!isinf(dx1[j]));
#endif
			}
#ifdef DEBUG_NANS
			printf("x0 = %f %f %f\n", x0[0], x0[1], x0[2]);
			printf("x1 = %f %f %f\n", x1[0], x1[1], x1[2]);
			printf("dx1 = %f %f %f\n", dx1[0], dx1[1], dx1[2]);
#endif
			d1 = constraints[i].distance;
//			printf("distance %f\n", d1);
			if (d1 == 0.0f)
				continue;
			l1 = Vector3(dx1[0], dx1[1], dx1[2]).length();
			l1 = CLAMP(l1, 0.0f, 100.0f);
			diff1 = (l1 - d1) / l1;
//			printf("distance d1 %f l1 %f diff1 %f\n", d1, l1, diff1);
			for (j = 0; j < 3; j++) {
				x0[j] += dx1[j] * 0.5f * diff1;
				x1[j] -= dx1[j] * 0.5f * diff1;
			}
			for (j = 0; j < 3; j++) {
				particles[skeleton_id].write[p0 * 3 + j] = x0[j];
				particles[skeleton_id].write[p1 * 3 + j] = x1[j];
			}
		}
		/* pin top row */
		for (i = 0; i < size_x; i++) {
			int bone = bone_chains[i][0];
			Transform pose = skel->get_bone_global_pose(bone);
			for (j = 0; j < 3; j++)
				particles[skeleton_id].write[i * 3 + j] =
						pose.origin.coord[j];
		}
	}
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
				build_facing_data(skel_id);
				printf("node: %ls\n", String(skel->get_name()).c_str());
				printf("root bones: %d\n", bones_start.size());
				printf("bones: %d\n", bones.size());
				printf("chains: %d\n", bone_chains.size());
			}
			mutex->lock();
			if (!particles.has(skel_id)) {
				verlet_init(skel_id);
				if (constraints.size() == 0) {
					create_constraints(skel_id);
					printf("constraints: %d\n", constraints.size());
				}
			}
			forces_step(skel_id, skel->get_physics_process_delta_time());
			verlet_step(skel_id, skel->get_physics_process_delta_time());
			constraints_step(skel_id, skel->get_physics_process_delta_time());
			mutex->unlock();
#if 0
			if (particles.size() == 0) {
				particles.resize(bones.size() * 3);
				int offset = 0;
				for (i = 0; i < bone_chains.size(); i++) {
					List<int> chain = bone_chains[i];
					for (List<int>::Element *b = chain.front(); b; b = b->next()) {
						int bone_id = b->get();
						Transform pose = skel->get_bone_global_pose(bone_id);
						Vector3 particle = pose.origin;
						particles.write[offset * 3] = particle.x;
						particles.write[offset * 3 + 1] = particle.y;
						particles.write[offset * 3 + 2] = particle.z;
						offset++;
					}
				}

			}
#endif
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
void Skirt::update_bones() {
	List<int> skeletons;
	mutex->lock();
	if (particles.empty())
		return;
	particles.get_key_list(&skeletons);
	for (List<int>::Element *e = skeletons.front(); e; e = e->next()) {
		int i, j;
		int skel_id = e->get();
		Skeleton *skel = Object::cast_to<Skeleton>(ObjectDB::get_instance(skel_id));
		if (!skel)
			continue;
		assert(size_x == bone_chains.size());
		for (i = 0; i < size_x; i++) {
			j = 0;
			for (const List<int>::Element *b = bone_chains[i].front(); b; b = b->next()) {
				Vector3 pos;
				int bone = b->get();
				int parent = skel->get_bone_parent(bone);
				Transform parent_pose = skel->get_bone_global_pose(parent);
				Transform pose = skel->get_bone_pose(bone);
				Transform rest_pose = skel->get_bone_rest(bone);
				Transform custom_pose = skel->get_bone_custom_pose(bone);
				pos.x = particles[skel_id][(j * size_x + i) * 3 + 0];
				pos.y = particles[skel_id][(j * size_x + i) * 3 + 1];
				pos.z = particles[skel_id][(j * size_x + i) * 3 + 2];
				Transform parent_inv = parent_pose.affine_inverse();
				Transform rest_inv = rest_pose.affine_inverse();
				Transform custom_inv = custom_pose.affine_inverse();
				Transform localize = custom_inv * rest_inv * parent_inv;
				Vector3 local_pos = localize.xform(pos);
				pose.origin = local_pos;
				if (j < bone_chains[i].size() - 1) {
					Vector3 target;
					target.x = particles[skel_id][((j + 1) * size_x + i) * 3 + 0];
					target.y = particles[skel_id][((j + 1) * size_x + i) * 3 + 1];
					target.z = particles[skel_id][((j + 1) * size_x + i) * 3 + 2];
					Vector3 local_target = localize.xform(target);
					//					printf("local target: %ls\n", String(local_target).c_str());
					Vector3 local_up = localize.xform(Vector3(0.0f, 1.0f, 0.0f));
					Transform rot;
//					if (facing.has(bone))
//						rot.basis = facing[bone].basis;
					pose = pose.looking_at(local_target, local_up) * rot.affine_inverse();
				}
				skel->set_bone_pose(bone, pose);
				j++;
			}
		}
	}
	mutex->unlock();
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

Skirt::Skirt() {
	instance = this;
	mutex = Mutex::create();
	call_deferred("connect_signals");
}

Skirt::~Skirt() {
	instance = NULL;
	memdelete(mutex);
}

void SkirtUpdate::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			skirt = Skirt::get_instance();
			printf("instance %p\n", skirt);
			set_process_internal(true);
			set_process_priority(16);
			break;
		case NOTIFICATION_INTERNAL_PROCESS:
			if (skirt)
				skirt->update_bones();
			break;
		case NOTIFICATION_EXIT_TREE:
			set_process_internal(false);
			break;
	}
}
SkirtUpdate::SkirtUpdate() :
		Node(), skirt(NULL) {
}
void SkirtDebug::_notification(int p_what) {
	List<int> skeletons;
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			skirt = Skirt::get_instance();
			printf("debug instance %p\n", skirt);
			set_process(true);
			set_process_priority(24);
			Ref<SpatialMaterial> mat = new SpatialMaterial();
			//			mat->set_albedo(Color(1.0f, 0.0f, 0.0f, 1.0f));
			mat->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
			mat->set_flag(SpatialMaterial::FLAG_USE_POINT_SIZE, true);
			mat->set_flag(SpatialMaterial::FLAG_DISABLE_DEPTH_TEST, true);
			mat->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);

			set_material_override(mat);
		} break;
		case NOTIFICATION_PROCESS:
			clear();
			skirt->mutex->lock();
			if (skirt->particles.empty())
				return;
			skirt->particles.get_key_list(&skeletons);
			for (List<int>::Element *e = skeletons.front(); e; e = e->next()) {
				int skel_id = e->get();
				draw_debug(skel_id);
			}
			break;
		case NOTIFICATION_EXIT_TREE:
			set_process(false);
			break;
	}
}
SkirtDebug::SkirtDebug() :
		ImmediateGeometry(), skirt(NULL) {
}
void SkirtDebug::draw_debug(int skeleton_id) {
	int i, j;
	Skeleton *skel =
			Object::cast_to<Skeleton>(ObjectDB::get_instance(skeleton_id));
	Transform skel_transform = skel->get_global_transform();
	for (i = 0; i < skel->get_bone_count(); i++) {
		Transform pose = skel->get_bone_global_pose(i);
		int parent = skel->get_bone_parent(i);
		if (parent >= 0) {
			Transform parent_pose = skel->get_bone_global_pose(parent);
			Vector3 p1 = (skel_transform * parent_pose).origin;
			Vector3 p2 = (skel_transform * pose).origin;
			begin(Mesh::PRIMITIVE_LINES, NULL);
			set_color(Color(1.0f, 1.0f, 0.0f, 1.0f));
			add_vertex(p1);
			add_vertex(p2);
			end();
		} else {
			Vector3 p1 = (skel_transform * pose).origin;
			begin(Mesh::PRIMITIVE_POINTS, NULL);
			set_color(Color(1.0f, 0.0f, 1.0f, 1.0f));
			add_vertex(p1);
			end();
		}
	}
	begin(Mesh::PRIMITIVE_POINTS, NULL);
	set_color(Color(1.0f, 0.0f, 0.0f, 1.0f));
	add_vertex(skel_transform.origin);
	end();
	begin(Mesh::PRIMITIVE_LINES, NULL);
	set_color(Color(0.3f, 1.0f, 0.3f, 1.0f));
	for (i = 0; i < skirt->size_x; i++) {
		for (j = 0; j < skirt->size_y; j++) {
			int pbase = j * skirt->size_x + i;
			int pnext = j * skirt->size_x + (i + 1) % skirt->size_x;
			int pnext2 = (j + 1) * skirt->size_x + i;
			//			int root_bone = skirt->bone_chains[i][0];
			//			int parent = skel->get_bone_parent(root_bone);
			//			Transform root_xform = skel->get_bone_global_pose(parent);

			Vector3 p1, p2, p3;
			p1.x = skirt->particles[skeleton_id][pbase * 3];
			p1.y = skirt->particles[skeleton_id][pbase * 3 + 1];
			p1.z = skirt->particles[skeleton_id][pbase * 3 + 2];
			p2.x = skirt->particles[skeleton_id][pnext * 3];
			p2.y = skirt->particles[skeleton_id][pnext * 3 + 1];
			p2.z = skirt->particles[skeleton_id][pnext * 3 + 2];
			add_vertex((skel_transform).xform(p1));
			add_vertex((skel_transform).xform(p2));
			if (j < skirt->size_y - 1) {
				p3.x = skirt->particles[skeleton_id][pnext2 * 3];
				p3.y = skirt->particles[skeleton_id][pnext2 * 3 + 1];
				p3.z = skirt->particles[skeleton_id][pnext2 * 3 + 2];
				add_vertex((skel_transform).xform(p1));
				add_vertex((skel_transform).xform(p3));
			}
		}
	}
	end();
}
