#include "modifierset.h"

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
void ModifierSet::add_modifier(const String &name, Ref<Image> vimage, Ref<Image> nimage, const PoolVector<float> &minmax)
{
	printf("adding modifier %ls\n", name.c_str());
	_add_modifier(name, vimage.ptr(), nimage.ptr(), minmax.read().ptr());
}
void ModifierSet::_add_modifier(const String &name, const Skeleton *skel,
		const String &bone, const Transform &xform)
{
	if (name2mod.has(name))
		return;
	assert(skel);
	modifiers[mod_count].create_from_bone(name, skel, bone, xform);
	name2mod[name] = mod_count;
	mod_count++;
}
void ModifierSet::_add_modifier(const String &name, const Skeleton *skel,
		const String &bone_from, const String &bone_to)
{
	if (name2mod.has(name))
		return;
	assert(skel);
	modifiers[mod_count].create_from_symmetry(name, skel, bone_from, bone_to);
	name2mod[name] = mod_count;
	mod_count++;
}
void ModifierSet::_add_modifier(const String &name, const Skeleton *skel,
		const String &bone_left, const Transform &xform_left,
		const String &bone_right, const Transform &xform_right)
{
	if (name2mod.has(name))
		return;
	assert(skel);
	modifiers[mod_count].create_from_pair(name, skel, bone_left, xform_left, bone_right, xform_right);
	name2mod[name] = mod_count;
	mod_count++;
}
void ModifierSet::_add_modifier(const String &name, const Skeleton *skel,
		const PoolVector<String> &bone_names,
		const PoolVector<Transform> bone_transforms)
{
	if (name2mod.has(name))
		return;
	assert(skel);
	modifiers[mod_count].create_from_group(name, skel, bone_names, bone_transforms);
	name2mod[name] = mod_count;
	mod_count++;
}
void ModifierSet::_add_modifier(const String &name, Image *vimage, Image *nimage, const float *minmax)
{
	Vector3 vmin(minmax[0], minmax[1], minmax[2]);
	Vector3 vmax(minmax[3], minmax[4], minmax[5]);
	Vector3 nmin(minmax[6], minmax[7], minmax[8]);
	Vector3 nmax(minmax[9], minmax[10], minmax[11]);
	if (name.ends_with("_plus") || name.ends_with("_minus")) {
		String gname = name.replace("_plus", "").replace("_minus", "");
		if (name2mod.has(gname)) {
			modifiers[name2mod[gname]].create_from_images(name,
					meshdata, vertex_count, vimage,
					nimage, vmin, vmax, nmin, nmax);
			return;
		} else {
			modifiers[mod_count].create_from_images(name, meshdata, vertex_count, vimage, nimage, vmin, vmax, nmin, nmax);
			name2mod[gname] = mod_count;
			mod_count++;
			return;
		}
	}
	modifiers[mod_count].create_from_images(name, meshdata, vertex_count, vimage, nimage, vmin, vmax, nmin, nmax);
	name2mod[name] = mod_count;
	mod_count++;
}
int ModifierSet::add_work_mesh(Ref<ArrayMesh> mesh, const NodePath &skel)
{
	struct work_mesh wm;
	int i;
	int ret = work_meshes.size();
	printf("pre add %d\n", work_meshes.size());
	wm.mat = mesh->surface_get_material(0);
	wm.work_mesh = mesh;
	wm.skeleton = skel;
	for (i = 0; i < mod_count; i++)
		wm.mod_values[i] = 0.0f;
	work_meshes.push_back(wm);
	printf("post add %d\n", work_meshes.size());
	return ret;
}
void ModifierSet::remove_work_mesh(int id)
{
	if (id < 0 || id >= work_meshes.size())
		return;
	printf("pre remove %d\n", work_meshes.size());
	work_meshes.remove(id);
	printf("post remove %d\n", work_meshes.size());
}
void ModifierSet::add_mesh(const String &name, const Ref<ArrayMesh> mesh)
{
	int i, j;
	surface = mesh->surface_get_arrays(0);
	const PoolVector<Vector2> &uvdata = surface[uv_index];
	const PoolVector<Vector3> &vdata = surface[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector3> &normal = surface[Mesh::ARRAY_NORMAL];
	meshdata = memnew_arr(float, vdata.size() * 14);
	vertex_count = vdata.size();
	const Vector2 *uvs = uvdata.read().ptr();
	const Vector3 *n = normal.read().ptr();
	const Vector3 *v = vdata.read().ptr();
	for (i = 0; i < uvdata.size(); i++) {
		meshdata[i * 14 + 0] = uvs[i][0];
		meshdata[i * 14 + 1] = uvs[i][1];
		meshdata[i * 14 + 2] = v[i][0];
		meshdata[i * 14 + 3] = v[i][1];
		meshdata[i * 14 + 4] = v[i][2];
		meshdata[i * 14 + 5] = n[i][0];
		meshdata[i * 14 + 6] = n[i][1];
		meshdata[i * 14 + 7] = n[i][2];
		meshdata[i * 14 + 8] = v[i][0];
		meshdata[i * 14 + 9] = v[i][1];
		meshdata[i * 14 + 10] = v[i][2];
		meshdata[i * 14 + 11] = n[i][0];
		meshdata[i * 14 + 12] = n[i][1];
		meshdata[i * 14 + 13] = n[i][2];
	}
	float eps_dist = 0.0001f;
	for (i = 0; i < vdata.size(); i++) {
		for (j = 0; j < vdata.size(); j++) {
			if (i == j)
				continue;
			if (v[i].distance_squared_to(v[j]) < eps_dist * eps_dist) {
				if (!same_verts.has(i))
					same_verts[i] = PoolVector<int>();
				same_verts[i].push_back(j);
			}
		}

	}
}

void ModifierSet::modify(Node *scene)
{
	int i, j, k;
	if (!dirty)
		return;
	if (work_meshes.size() == 0)
		return;
	Skeleton *skel = find_node<Skeleton>(scene);
	for (k = 0; k < work_meshes.size(); k++) {
#if 0
		NodePath skel_path = work_meshes[k].skeleton;
		Skeleton *skel = (Skeleton *)scene->get_node(skel_path);
#endif
		for (i = 0; i < skel->get_bone_count(); i++) {
			skel->set_bone_custom_pose(i, Transform());
			skel->set_bone_pose(i, Transform());
		}
		int left_foot = skel->find_bone("foot_L");
		int pelvis = skel->find_bone("pelvis");
		assert(left_foot >= 0 && pelvis >= 0);
		Vector3 lf_orig_pos = skel->get_bone_global_pose(left_foot).origin;

		for (i = 0; i < vertex_count; i++) {
			meshdata[i * 14 + 2] =  meshdata[i * 14 + 8];
			meshdata[i * 14 + 3] =  meshdata[i * 14 + 9];
			meshdata[i * 14 + 4] =  meshdata[i * 14 + 10];
			meshdata[i * 14 + 5] =  meshdata[i * 14 + 11];
			meshdata[i * 14 + 6] =  meshdata[i * 14 + 12];
			meshdata[i * 14 + 7] =  meshdata[i * 14 + 13];
		}
		for (i = 0; i < mod_count; i++)
			if (work_meshes[k].mod_values.has(i))
				if (fabs(work_meshes[k].mod_values[i]) >= 0.001f) {
					switch(modifiers[i].type) {
					case ModifierBase::TYPE_BLEND:
						modifiers[i].modify(meshdata, vertex_count, work_meshes[k].mod_values[i]);
						break;
					case ModifierBase::TYPE_BONE:
					case ModifierBase::TYPE_PAIR:
					case ModifierBase::TYPE_GROUP:
						modifiers[i].modify(skel, work_meshes[k].mod_values[i]);
						break;
					case ModifierBase::TYPE_SYMMETRY:
						modifiers[i].modify(skel);
					}
				}
		for (i = 0; i < vertex_count; i++) {
			if (same_verts.has(i)) {
				float vx = meshdata[i * 14 + 2];
				float vy = meshdata[i * 14 + 3];
				float vz = meshdata[i * 14 + 4];
				float nx = meshdata[i * 14 + 5];
				float ny = meshdata[i * 14 + 6];
				float nz = meshdata[i * 14 + 7];
				for (j = 0; j < same_verts[i].size(); j++) {
					vx = Math::lerp(vx, meshdata[same_verts[i][j] * 14 + 2], 0.5f);
					vy = Math::lerp(vy, meshdata[same_verts[i][j] * 14 + 3], 0.5f);
					vz = Math::lerp(vz, meshdata[same_verts[i][j] * 14 + 4], 0.5f);
					nx = Math::lerp(nx, meshdata[same_verts[i][j] * 14 + 5], 0.5f);
					ny = Math::lerp(ny, meshdata[same_verts[i][j] * 14 + 6], 0.5f);
					nz = Math::lerp(nz, meshdata[same_verts[i][j] * 14 + 7], 0.5f);
				}
				meshdata[i * 14 + 2] = vx;
				meshdata[i * 14 + 3] = vy;
				meshdata[i * 14 + 4] = vz;
				meshdata[i * 14 + 5] = nx;
				meshdata[i * 14 + 6] = ny;
				meshdata[i * 14 + 7] = nz;
				for (j = 0; j < same_verts[i].size(); j++) {
					meshdata[same_verts[i][j] * 14 + 2] = vx;
					meshdata[same_verts[i][j] * 14 + 3] = vy;
					meshdata[same_verts[i][j] * 14 + 4] = vz;
					meshdata[same_verts[i][j] * 14 + 5] = nx;
					meshdata[same_verts[i][j] * 14 + 6] = ny;
					meshdata[same_verts[i][j] * 14 + 7] = nz;
				}
			}
		}
		Vector3 lf_pos = skel->get_bone_global_pose(left_foot).origin;
		Transform pelvis_xform = skel->get_bone_custom_pose(pelvis);
		pelvis_xform.origin += lf_pos - lf_orig_pos;
		skel->set_bone_custom_pose(pelvis, pelvis_xform);
		PoolVector<Vector3> vertices = surface[Mesh::ARRAY_VERTEX];
		PoolVector<Vector3> normals = surface[Mesh::ARRAY_NORMAL];
		PoolVector<Vector3>::Write vertex_w = vertices.write();
		PoolVector<Vector3>::Write normal_w = normals.write();
		for (i = 0; i < vertex_count; i++) {
			vertex_w[i].x = meshdata[i * 14 + 2];
			vertex_w[i].y = meshdata[i * 14 + 3];
			vertex_w[i].z = meshdata[i * 14 + 4];
			normal_w[i].x = meshdata[i * 14 + 5];
			normal_w[i].y = meshdata[i * 14 + 6];
			normal_w[i].z = meshdata[i * 14 + 7];
		}
		vertex_w.release();
		normal_w.release();
		surface[Mesh::ARRAY_VERTEX] = vertices;
		surface[Mesh::ARRAY_NORMAL] = normals;
		work_meshes[k].work_mesh->surface_remove(0);
		work_meshes[k].work_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, surface);
		work_meshes[k].work_mesh->surface_set_material(0, work_meshes[k].mat);
	}
	dirty = false;
}

void ModifierSet::create_uv(int id, Ref<ArrayMesh> mesh, int src_id, Ref<ArrayMesh> src_mesh)
{
}
void CharacterModifierSet::load(Ref<_File> fd)
{
	int i;
	float minp[3], maxp[3], min_normal[3], max_normal[3];
	int width, height, format;
	int nwidth, nheight, nformat;
	int dec_size, comp_size;

	String shape_name = fd->get_pascal_string();
	printf("shape: %ls\n", shape_name.c_str());
	for (i = 0; i < 3; i++)
		minp[i] = fd->get_float();
	for (i = 0; i < 3; i++)
		maxp[i] = fd->get_float();
	width = fd->get_32();
	height = fd->get_32();
	format = fd->get_32();
	dec_size = fd->get_32();
	comp_size = fd->get_32();
	PoolVector<uint8_t> imgbuf = fd->get_buffer(comp_size);
	PoolVector<uint8_t> imgdecbuf;
	imgdecbuf.resize(dec_size);
	Compression::decompress(imgdecbuf.write().ptr(), dec_size, imgbuf.read().ptr(), comp_size, Compression::MODE_DEFLATE);
	Image img = Image();
	img.create(width, height, false, (Image::Format)format, imgdecbuf);
	for (i = 0; i < 3; i++)
		min_normal[i] = fd->get_float();
	for (i = 0; i < 3; i++)
		max_normal[i] = fd->get_float();
	nwidth = fd->get_32();
	nheight = fd->get_32();
	nformat = fd->get_32();
	dec_size = fd->get_32();
	comp_size = fd->get_32();
	PoolVector<uint8_t> imgbufn = fd->get_buffer(comp_size);
	PoolVector<uint8_t> imgdecbufn;
	imgdecbufn.resize(dec_size);
	Compression::decompress(imgdecbufn.write().ptr(), dec_size, imgbufn.read().ptr(), comp_size, Compression::MODE_DEFLATE);
	Image imgn = Image();
	imgn.create(nwidth, nheight, false, (Image::Format)nformat, imgdecbufn);
	Vector<String> splitname = shape_name.split(":");
	PoolVector<float> minmax;
	minmax.resize(12);
	for (i = 0; i < 3; i++)
		minmax.write()[i] = minp[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 3] = maxp[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 6] = min_normal[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 9] = max_normal[i];
	for (const String *key = mods.next(NULL);
			key;
			key = mods.next(key)) {
		if (helpers[*key] == splitname[0])
			mods[*key]->_add_modifier(splitname[1],
				&img, &imgn, minmax.read().ptr());
	}
}
void CharacterModifierSet::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_base_name", "name"),
			&CharacterModifierSet::set_base_name);
	ClassDB::bind_method(D_METHOD("add_mesh_scene", "node"),
			&CharacterModifierSet::add_mesh_scene);
	ClassDB::bind_method(D_METHOD("add_work_mesh_scene", "ps"),
			&CharacterModifierSet::add_work_mesh_scene);
	ClassDB::bind_method(D_METHOD("remove_work_meshes", "id"),
			&CharacterModifierSet::remove_work_meshes);
	ClassDB::bind_method(D_METHOD("set_modifier_value", "id", "name", "value"),
			&CharacterModifierSet::set_modifier_value);
	ClassDB::bind_method(D_METHOD("get_modifier_value", "id", "name"),
			&CharacterModifierSet::get_modifier_value);
	ClassDB::bind_method(D_METHOD("modify", "scene"), &CharacterModifierSet::modify);
	ClassDB::bind_method(D_METHOD("get_modifier_list"),
			&CharacterModifierSet::get_modifier_list);
	ClassDB::bind_method(D_METHOD("load", "fd"),
			&CharacterModifierSet::load);
	ClassDB::bind_method(D_METHOD("set_helper", "mod_name", "name"),
			&CharacterModifierSet::set_helper);
	ClassDB::bind_method(D_METHOD("add_bone_modifier", "name", "scene", "bone_name", "xform"),
			&CharacterModifierSet::add_bone_modifier);
	ClassDB::bind_method(D_METHOD("add_bone_modifier_symmetry", "name", "scene",
				"bone_left",
				"bone_right"),
			&CharacterModifierSet::add_bone_modifier_symmetry);
	ClassDB::bind_method(D_METHOD("add_bone_modifier_pair", "name", "scene",
				"left",
				"right"),
			&CharacterModifierSet::add_bone_modifier_pair);
	ClassDB::bind_method(D_METHOD("add_bone_modifier_group", "name", "scene",
				"bone_names",
				"bone_transforms"),
			&CharacterModifierSet::add_bone_modifier_group);
	ClassDB::bind_method(D_METHOD("add_slot", "name",
								  "helper", "uv_index"),
			&CharacterModifierSet::add_slot);
	ClassDB::bind_method(D_METHOD("spawn"),
			&CharacterModifierSet::spawn);
	ClassDB::bind_method(D_METHOD("remove", "node"),
			&CharacterModifierSet::remove);
	ClassDB::bind_method(D_METHOD("get_slot", "node", "name"),
			&CharacterModifierSet::remove);
}
