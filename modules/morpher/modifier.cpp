#include "modifier.h"

void ModifierBlend::modify(float *data, int vertex_count, float value)
{
	for (int i = 0; i < mod_indices.size(); i++) {
		int index = mod_indices[i];
		float vx = mod_data[i * 6 + 0];
		float vy = mod_data[i * 6 + 2];
		float vz = mod_data[i * 6 + 4];
		float nx = mod_data[i * 6 + 1];
		float ny = mod_data[i * 6 + 3];
		float nz = mod_data[i * 6 + 5];
		data[index * 14 + 2] -= vx * value;
		data[index * 14 + 3] -= vy * value;
		data[index * 14 + 4] -= vz * value;
		data[index * 14 + 5] -= nx * value;
		data[index * 14 + 6] -= ny * value;
		data[index * 14 + 7] -= nz * value;
	}
#ifdef MORPH_DEBUG
	printf("mod: %ls val %f\n", mod_name.c_str(), value);
#endif
}

void ModifierBone::modify(Skeleton *skel, float value)
{
	if (bone_id >= 0)
		skel->set_bone_custom_pose(bone_id,
			skel->get_bone_custom_pose(bone_id) *
				Transform().interpolate_with(xform,	value));
}

void ModifierSymmetry::modify(Skeleton *skel)
{
	const Transform &xform = skel->get_bone_custom_pose(bone_from_id);
	Transform gpose = bf_parent_xform * xform;
	gpose = gpose.scaled(Vector3(-1, 1, 1));
	gpose = bt_parent_xform.affine_inverse() * gpose;
	skel->set_bone_custom_pose(bone_to_id, gpose);
}
void ModifierPair::modify(Skeleton *skel, float value)
{
	const Transform &xform_left = skel->get_bone_custom_pose(bone_left_id);
	const Transform &xform_right = skel->get_bone_custom_pose(bone_right_id);
	skel->set_bone_custom_pose(bone_left_id, xform_left * Transform().interpolate_with(bone_left_xform, value));
	skel->set_bone_custom_pose(bone_right_id, xform_right * Transform().interpolate_with(bone_right_xform, value));
}
void ModifierBoneGroup::modify(Skeleton *skel, float value)
{
	int i;
	for (i = 0; i < bones.size(); i++) {
		Transform bone_xform = skel->get_bone_custom_pose(bones[i]);
		bone_xform *= Transform().interpolate_with(xforms[i], value);
		skel->set_bone_custom_pose(bones[i], bone_xform);
	}
}
void ModifierBone::create_from_bone(const Skeleton *skel, const String &bone, const Transform &xform)
{
	assert(skel);
	type = TYPE_BONE;
	bone_id = skel->find_bone(bone);
	this->xform = xform;
	empty = false;
}
void ModifierBlend::create_from_images(const String &name,
		const float *meshdata, int count, 
		Image *vdata, Image *ndata, const Vector3 &vmin,
		const Vector3 &vmax, const Vector3 &nmin, const Vector3 &nmax)
{
	int i, j;
	for (i = 0; i < 3; i++) {
		minp[i] = vmin[i];
		maxp[i] = vmax[i];
		minn[i] = nmin[i];
		maxn[i] = nmax[i];
	}
	vdata->lock();
	ndata->lock();
	for (i = 0; i < count; i++) {
		int vx = (int)(meshdata[i * 14 + 0] * (float)vdata->get_width());
		int vy = (int)(meshdata[i * 14 + 1] * (float)vdata->get_height());
		Color c = vdata->get_pixel(vx, vy);
		Color nc = ndata->get_pixel(vx, vy);
		float pdelta[3], ndelta[3];
		for (j = 0; j < 3; j++) {
			pdelta[j] = minp[j] + (maxp[j] - minp[j]) * c[j];
			ndelta[j] = minn[j] + (maxn[j] - minn[j]) * nc[j];
		}
		const float eps = 0.001f;
		if (pdelta[0] * pdelta[0] + pdelta[1] * pdelta[1] + pdelta[2] * pdelta[2] > eps * eps) {
			mod_indices.push_back(i);
			for (j = 0; j < 3; j++) {
				mod_data.push_back(pdelta[j]);
				mod_data.push_back(ndelta[j]);
			}
		}
	}
	ndata->unlock();
	vdata->unlock();
	empty = false;
	mod_name = name;
	type = TYPE_BLEND;
#ifdef MORPH_DEBUG
	printf("Created %ls\n", mod_name.c_str());
#endif
}
void ModifierSymmetry::create_from_symmetry(const Skeleton *skel,
		const String &bone_left,
		const String &bone_right)
{
	bone_from_id = skel->find_bone(bone_left);
	int bf_parent_id = skel->get_bone_parent(bone_from_id);
	bone_to_id = skel->find_bone(bone_right);
	int bt_parent_id = skel->get_bone_parent(bone_to_id);
	assert(bone_from_id >= 0);
	assert(bone_to_id >= 0);
	if (bf_parent_id >= 0)
		bf_parent_xform = skel->get_bone_global_pose(bf_parent_id);
	if (bt_parent_id >= 0)
		bt_parent_xform = skel->get_bone_global_pose(bt_parent_id);
}
void ModifierPair::create_from_pair(const Skeleton *skel,
		const String &bone_left,
		const Transform &xform_left,
		const String &bone_right,
		const Transform &xform_right)
{
	bone_left_id = skel->find_bone(bone_left);
	bone_right_id = skel->find_bone(bone_right);
	bone_left_xform = xform_left;
	bone_right_xform = xform_right;
}
void ModifierBoneGroup::create_from_group(const Skeleton * skel,
		const PoolVector<String> &bone_names,
		const PoolVector<Transform> &bone_transforms)
{
	int i;
	bones.resize(bone_names.size());
	PoolVector<int>::Write bw = bones.write();

	for (i = 0; i < bone_names.size(); i++) {
		int id = skel->find_bone(bone_names[i]);
		assert(id >= 0);
		bw[i] = id;
	}
	xforms = bone_transforms;
}
