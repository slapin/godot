#include <cassert>
#include <scene/3d/skeleton.h>
#include "skirt.h"
void SkirtSimulation::process_collisions()
{
    int i, j;
    Vector3 p, penetration;
    List<int> collider_list;

	colliders.get_key_list(&collider_list);
	for (List<int>::Element *e = collider_list.front(); e; e = e->next()) {
		int bone = e->get();
        struct collider *col = &colliders[bone];
		for (i = 1; i < size_y; i++) {
            for (j = 0; j < size_x; j++) {
                p = get_particle(i * size_x + j);
                if (col->is_colliding(p, &penetration)) {
                    p = p + penetration;
                    set_particle(i * size_x + j, p);
                    debug_penetration_list.push_back(i * size_x + j);
                }
            }
		}
	}
}

void collider::create_from_bone(const Skeleton *skel, const String &name,
    const String &end_name,
    float height, float r, const Vector3 &cv, const Vector3 &offt)
{
	bone = skel->find_bone(name);
	end_bone = skel->find_bone(end_name);

	parent = skel->get_bone_parent(bone);
	xform = skel->get_bone_pose(bone);
	xform_parent = skel->get_bone_global_pose(parent);
	xform_rest = skel->get_bone_rest(bone);
	xform_custom = skel->get_bone_custom_pose(bone);
	end_offset = skel->get_bone_global_pose(end_bone).origin;
	h = height;
	radius  = r;
	p1 = (xform_parent * xform_rest * xform_custom * xform).origin;
	offset = (end_offset - p1).normalized() * h;
	p2 = p1 + offset;
	this->name = name;
    change = cv;
    toffset = offt;
    Transform xcheck;
    xcheck.origin = p1;
    xcheck.looking_at(p2, Vector3(0, 1, 0));
    xcheck.orthonormalize();
    toffset_mod = xcheck.xform_inv(toffset);
}
void collider::update(const Skeleton *skel)
{
	xform = skel->get_bone_pose(bone);
	xform_parent = skel->get_bone_global_pose(parent);
	p1 = (xform_parent * xform_rest * xform_custom * xform).origin;
	end_offset = skel->get_bone_global_pose(end_bone).origin;
	offset = (end_offset - p1).normalized() * h;
	p2 = p1 + offset;
    Transform xcheck;
    xcheck.origin = p1;
    xcheck.looking_at(p2, Vector3(0, 1, 0));
    xcheck.orthonormalize();
    toffset_mod = xcheck.xform_inv(toffset);
}
bool collider::is_colliding(Vector3 p, Vector3 *penetration)
{
	Vector3 coldir = p - p1;
	Vector3 v = p2 - p1;
    float dx = v.length();
	float dot = coldir.dot(v.normalized());
	if (dot < -radius)
		return false;
	if (dot > v.length() + radius)
		return false;
    dot = CLAMP(dot, 0.0f, v.length());
	Vector3 projp = p1 + v.normalized() * dot;
    Vector3 px = (p - projp);
	if (px.length_squared() > radius * radius)
		return false;
	Vector3 pdir = (p - projp);
	float plength = radius - pdir.length() + 0.0001f;

    assert(plength >= 0.0f);
	*penetration = pdir.normalized() * plength;
	return true;
}
