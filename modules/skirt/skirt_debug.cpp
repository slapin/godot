#include <cstdio>
#include <cassert>
#include <scene/3d/skeleton.h>
#include "skirt.h"

void SkirtDebug::_notification(int p_what) {
	List<int> skeletons;
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			skirt = Skirt::get_instance();
			printf("debug instance %p\n", skirt);
			set_process(true);
			set_process_priority(24);
			Ref<SpatialMaterial> mat = new SpatialMaterial();
			mat->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
			mat->set_flag(SpatialMaterial::FLAG_USE_POINT_SIZE, true);
			mat->set_flag(SpatialMaterial::FLAG_DISABLE_DEPTH_TEST, true);
			mat->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);

			set_material_override(mat);
		} break;
		case NOTIFICATION_PROCESS:
			clear();
			skirt->mutex->lock();
			if (skirt->sim_hash.empty())
				return;
			skirt->sim_hash.get_key_list(&skeletons);
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
	List<int> col_bones;
	skirt->sim_hash[skeleton_id].colliders.get_key_list(&col_bones);
	assert(col_bones.size() > 0);
	Vector<Vector3> circle;
	for (i = 0; i < 8; i++) {
		float angle = M_PI * 2.0f * (float)i / 8.0f;
		circle.push_back(Vector3(sinf(angle), 0.0f, cosf(angle)));
	}
    /* draw collider */
	for (List<int>::Element *s = col_bones.front(); s; s = s->next()) {
		int bone = s->get();
		assert(bone >= 0);
		struct collider &col = skirt->sim_hash[skeleton_id].colliders[bone];
		Vector3 p1 = col.p1;
		Vector3 p2 = col.p2;
		/* drawing colliders */
		begin(Mesh::PRIMITIVE_LINES, NULL);
		set_color(Color(0.8f, 0.8f, 1.0f, 1.0f));
		add_vertex(skel_transform.xform(p1));
		add_vertex(skel_transform.xform(p2));
		for (i = 0; i < circle.size(); i++) {
			add_vertex(skel_transform.xform(p1));
			add_vertex(skel_transform.xform(p1 + circle[i] * col.radius));
			add_vertex(skel_transform.xform(p2));
			add_vertex(skel_transform.xform(p2 + circle[i] * col.radius));

   			add_vertex(skel_transform.xform(p1 + circle[i] * col.radius));
   			add_vertex(skel_transform.xform(p1 + circle[(i + 1) % circle.size()] * col.radius));
   			add_vertex(skel_transform.xform(p2 + circle[i] * col.radius));
   			add_vertex(skel_transform.xform(p2 + circle[(i + 1) % circle.size()] * col.radius));
   			add_vertex(skel_transform.xform(p1 + circle[i] * col.radius));
   			add_vertex(skel_transform.xform(p2 + circle[i] * col.radius));
		}
		end();
    }
	begin(Mesh::PRIMITIVE_LINES, NULL);
	set_color(Color(1.0f, 0.1f, 0.1f, 1.0f));
	bool colliders = false;
	int col_count = skirt->sim_hash[skeleton_id].debug_penetration_list.size();
    printf("skeleton_id: %d col_count: %d\n", skeleton_id, col_count);
	if (skirt->sim_hash.has(skeleton_id)) {
		List<int>::Element *e = skirt->sim_hash[skeleton_id].debug_penetration_list.front();
		while (e) {
			int pt_id = e->get();
			if (!stale_collisions.has(skeleton_id)) {
				Vector<int> stale;
				stale_collisions[skeleton_id] = stale;
			}
			if (stale_collisions[skeleton_id].find(pt_id) < 0)
				stale_collisions[skeleton_id].push_back(pt_id);
			Vector3 pt = skirt->sim_hash[skeleton_id].get_particle(pt_id);
			Vector3 modpt = skel_transform.xform(pt);
			add_vertex(modpt + Vector3(0.1, 0.1, 0));
			add_vertex(modpt + Vector3(-0.1, -0.1, 0));
			add_vertex(modpt + Vector3(0.1, -0.1, 0));
			add_vertex(modpt + Vector3(-0.1, 0.1, 0));
			skirt->sim_hash[skeleton_id].debug_penetration_list.pop_front();
			e = skirt->sim_hash[skeleton_id].debug_penetration_list.front();
			colliders = true;
		}
	}
	if (stale_collisions.has(skeleton_id)) {
		while (stale_collisions[skeleton_id].size() > 20)
			stale_collisions[skeleton_id].remove(0);
		for (i = 0; i < stale_collisions[skeleton_id].size(); i++) {
			int pt_id = stale_collisions[skeleton_id][i];
			Vector3 pt = skirt->sim_hash[skeleton_id].get_particle(pt_id);
			Vector3 modpt = skel_transform.xform(pt);
			add_vertex(modpt + Vector3(0.1, 0.1, 0));
			add_vertex(modpt + Vector3(-0.1, -0.1, 0));
			add_vertex(modpt + Vector3(0.1, -0.1, 0));
			add_vertex(modpt + Vector3(-0.1, 0.1, 0));
		}
		if (!colliders && stale_collisions[skeleton_id].size() > 0)
			stale_collisions[skeleton_id].remove(0);
	}
	end();
	/* Draw skirt simulation */
	begin(Mesh::PRIMITIVE_LINES, NULL);
	set_color(Color(0.3f, 1.0f, 0.3f, 1.0f));
    SkirtSimulation *ss = &skirt->sim_hash[skeleton_id];
    int size_x = ss->get_size_x();
    int size_y = ss->get_size_y();

	for (i = 0; i < size_x; i++) {
		for (j = 0; j < size_y; j++) {
			int pbase = j * size_x + i;
			int pnext = j * size_x + (i + 1) % size_x;
			int pnext2 = (j + 1) * size_x + i;
			Vector3 pt1, pt2, pt3;
			pt1 = skirt->sim_hash[skeleton_id].get_particle(pbase);
			pt2 = skirt->sim_hash[skeleton_id].get_particle(pnext);
			add_vertex((skel_transform).xform(pt1));
			add_vertex((skel_transform).xform(pt2));
			if (j < skirt->size_y - 1) {
					pt3 = skirt->sim_hash[skeleton_id].get_particle(pnext2);
			        add_vertex((skel_transform).xform(pt1));
			        add_vertex((skel_transform).xform(pt3));
			}
		}
	}
	end();
}
