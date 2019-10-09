#include "modifierset.h"

PoolVector<float> MapStorage::get_minmax(const String &shape_name)
{
	int i;
	PoolVector<float> minmax;
	minmax.resize(12);
	struct datablock d = data[shape_name];
	for (i = 0; i < 3; i++)
		minmax.write()[i] = d.minp[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 3] = d.maxp[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 6] = d.min_normal[i];
	for (i = 0; i < 3; i++)
		minmax.write()[i + 9] = d.max_normal[i];
	return minmax;
}
void MapStorage::load()
{
	int i, j;
	float minp[3], maxp[3], min_normal[3], max_normal[3];
	int width, height, format;
	int nwidth, nheight, nformat;
	int dec_size, comp_size;
	assert(config.has("map_path"));
	const String &map_path = config["map_path"];
	FileAccess *fd = FileAccess::open(map_path, FileAccess::READ);
	int count = fd->get_32();
	for (j = 0; j < count; j ++) {
		struct datablock d;
		String shape_name = fd->get_pascal_string();
		printf("loading shape: %ls\n", shape_name.c_str());
		for (i = 0; i < 3; i++)
			d.minp[i] = fd->get_float();
		for (i = 0; i < 3; i++)
			d.maxp[i] = fd->get_float();
		d.width = fd->get_32();
		d.height = fd->get_32();
		d.format = fd->get_32();
		d.dec_size = fd->get_32();
		comp_size = fd->get_32();
		d.buf.resize(comp_size);
		fd->get_buffer(d.buf.write().ptr(), comp_size);
		for (i = 0; i < 3; i++)
			d.min_normal[i] = fd->get_float();
		for (i = 0; i < 3; i++)
			d.max_normal[i] = fd->get_float();
		d.width_normal = fd->get_32();
		d.height_normal = fd->get_32();
		d.format_normal = fd->get_32();
		d.dec_size_normal = fd->get_32();
		comp_size = fd->get_32();
		d.buf_normal.resize(comp_size);
		fd->get_buffer(d.buf_normal.write().ptr(), comp_size);
		data[shape_name] = d;
	}
}
Ref<Image> MapStorage::get_image(const String &name) const
{
	const struct datablock &d = data[name];
	PoolVector<uint8_t> imgdecbuf;
	imgdecbuf.resize(d.dec_size);
	Compression::decompress(imgdecbuf.write().ptr(), d.dec_size, d.buf.read().ptr(), d.buf.size(), Compression::MODE_DEFLATE);
	Ref<Image> img = memnew(Image);
	img->create(d.width, d.height, false, (Image::Format)d.format, imgdecbuf);
	return img;
}
Ref<Image> MapStorage::get_normal_image(const String &name) const
{
	const struct datablock &d = data[name];
	PoolVector<uint8_t> imgdecbuf;
	imgdecbuf.resize(d.dec_size_normal);
	Compression::decompress(imgdecbuf.write().ptr(),
		d.dec_size_normal, d.buf_normal.read().ptr(),
		d.buf_normal.size(), Compression::MODE_DEFLATE);
	Ref<Image> img = memnew(Image);
	img->create(d.width_normal, d.height_normal, false, (Image::Format)d.format_normal, imgdecbuf);
	return img;
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
	if (!name2mod.has(name)) {
		modifiers[mod_count].create_from_images(name, meshdata, vertex_count, vimage, nimage, vmin, vmax, nmin, nmax);
		name2mod[name] = mod_count;
		mod_count++;
	} else
		modifiers[name2mod[name]].create_from_images(name,
			meshdata, vertex_count,
			vimage, nimage, vmin, vmax, nmin, nmax);
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
void CharacterModifierSet::process()
{
	MapStorage *ms = MapStorage::get_singleton();
	PoolVector<String> mlist = ms->get_list();
	int i;
	for (i = 0; i < mlist.size(); i++) {
		const String &shape_name = mlist[i];
		Vector<String> splitname = shape_name.split(":");
		PoolVector<float> minmax = ms->get_minmax(shape_name);
		for (const String *key = mods.next(NULL);
				key;
				key = mods.next(key)) {
			if (helpers[*key] == splitname[0]) {
				Ref<Image> img = ms->get_image(shape_name);
				Ref<Image> imgn = ms->get_normal_image(shape_name);
				mods[*key]->_add_modifier(splitname[1],
					img.ptr(), imgn.ptr(), minmax.read().ptr());
			}
		}
	}
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

void CharacterModifierSet::set_accessory(Node *node, const String &slot_name, const String &gender,
				const String &atype, const String &aname)
{
	int i;
	MeshInstance *slot = Object::cast_to<MeshInstance>(get_slot(node, slot_name));
	Skeleton *skel = ModifierSet::find_node<Skeleton>(node);
	assert(skel);
	if (!slot) {
		slot = memnew(MeshInstance);
		slot->set_name(slot_name);
		skel->add_child(slot);
		slot->set_transform(Transform());
		slot->set_skeleton_path(slot->get_path_to(skel));
	} else
		slot->set_skeleton_path(slot->get_path_to(skel));
	if (!accessories.has(gender))
		return;
	const Dictionary &items = accessories[gender];
	if (!items.has(atype))
		return;
	const Dictionary &category = items[atype];
	if (!category.has(aname))
		return;
	const Dictionary &data = category[aname];
	const String &mesh_path = data["path"];
	const Array &materials = data["materials"];
	Error err = OK;
	Ref<ArrayMesh> mesh = ResourceLoader::load(mesh_path, "", &err);
	if (err != OK) {
		printf("Could not read resource %ls\n", mesh_path.c_str());
		return;
	}
	for (i = 0; i < mesh->get_surface_count(); i++) {
		const Dictionary &mat_data = materials[i];
		const String &mat_path  = mat_data["path"];
		Ref<Material> mat = ResourceLoader::load(mat_path, "", &err);
		if (err != OK) {
			printf("Could not read resource %ls\n", mat_path.c_str());
			return;
		}
		mesh->surface_set_material(i, mat);
	}
	slot->hide();
	slot->set_mesh(mesh);
	slot->show();
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
			&CharacterModifierSet::get_slot);
	ClassDB::bind_method(D_METHOD("hide_slot", "node", "name"),
			&CharacterModifierSet::hide_slot);
	ClassDB::bind_method(D_METHOD("show_slot", "node", "name"),
			&CharacterModifierSet::show_slot);
	ClassDB::bind_method(D_METHOD("set_accessory", "node", "slot_name",
			"gender", "atype", "aname"),
			&CharacterModifierSet::set_accessory);
	ClassDB::bind_method(D_METHOD("get_accessory_list",	"gender",
			"atype", "name_start"),
			&CharacterModifierSet::get_acessory_list);
	ClassDB::bind_method(D_METHOD("process"),
			&CharacterModifierSet::process);
}
