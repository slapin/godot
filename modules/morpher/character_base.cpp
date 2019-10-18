#include <sys/time.h>
#include <core/os/os.h>
#include <scene/3d/mesh_instance.h>
#include <scene/3d/spatial.h>
#include <scene/3d/skeleton.h>
#include <scene/main/scene_tree.h>
#include <scene/main/viewport.h>
#include "character_base.h"
#include "character_slot.h"
#include "map_storage.h"
template <class T>
static inline T *find_node(Node *node, const String &name = "") {
	int i;
	T *ret = NULL;
	assert(node);
	List<Node *> queue;
	queue.push_back(node);
	while (!queue.empty()) {
		Node *item = queue.front()->get();
		queue.pop_front();
		ret = Object::cast_to<T>(item);
		if (ret && (name.length() == 0 || ret->get_name() == name))
			break;
		for (i = 0; i < item->get_child_count(); i++)
			queue.push_back(item->get_child(i));
	}
	assert(ret);
	return ret;
}

void CharacterGenderList::config() {
	const Dictionary &config = ConfigData::get_singleton()->get();
	const Array &gdata = config["genders"];
	int i;
	for (i = 0; i < gdata.size(); i++) {
		const Dictionary &g = gdata[i];
		const String &gname = g["name"];
		const String &scene = g["scene"];
		const String &left_foot = g["left_foot"];
		const String &right_foot = g["right_foot"];
		const String &pelvis = g["pelvis"];
		Error err = OK;
		Ref<PackedScene> pscene = ResourceLoader::load(scene, "", &err);
		if (err != OK) {
			printf("Could not read resource %ls\n", scene.c_str());
			continue;
		}
		create_gender(gname, pscene);
		genders[gname].left_foot = left_foot;
		genders[gname].right_foot = right_foot;
		genders[gname].pelvis = pelvis;
	}
}
CharacterGenderList::CharacterGenderList() {
}

void CharacterGenderList::_bind_methods() {
	ClassDB::bind_method(D_METHOD("config"),
			&CharacterGenderList::config);
}
void CharacterGenderList::create_gender(const String &name, Ref<PackedScene> base) {
	int i;
	const Dictionary &config = ConfigData::get_singleton()->get();
	CharacterGender g;
	g.name = name;
	g.scene = base;
	const Array &slot_data = config["slot_data"];
	for (i = 0; i < slot_data.size(); i++) {
		CharacterSlot slot;
		const Dictionary &item = slot_data[i];
		slot.name = item["name"];
		slot.match = item["match"];
		slot.category = item["category"];
		slot.helper = item["helper"];
		slot.mandatory = item["mandatory"];
		if (item.has("blend_skip"))
			slot.blend_skip = item["blend_skip"];
		else
			slot.blend_skip = false;
		g.slot_list[slot.name] = slot;
	}
	genders[name] = g;
}
void CharacterGenderList::remove_gender(const String &name) {
	genders.erase(name);
}

void CharacterInstanceList::_bind_methods() {
	ClassDB::bind_method(D_METHOD("create", "gender", "xform", "slot_conf"),
			&CharacterInstanceList::create);
	ClassDB::bind_method(D_METHOD("update"),
			&CharacterInstanceList::update);
	ClassDB::bind_method(D_METHOD("remove", "scene"),
			&CharacterInstanceList::remove);
	ClassDB::bind_method(D_METHOD("set_mod_value", "scene", "mod", "value"),
			&CharacterInstanceList::set_mod_value);
	ClassDB::bind_method(D_METHOD("get_mod_value", "scene", "mod"),
			&CharacterInstanceList::get_mod_value);
	ClassDB::bind_method(D_METHOD("get_modifier_list"),
			&CharacterInstanceList::get_modifier_list);
	ClassDB::bind_method(D_METHOD("get_base_modifier_list"),
			&CharacterInstanceList::get_base_modifier_list);
	ClassDB::bind_method(D_METHOD("get_instance_modifier_values"),
			&CharacterInstanceList::get_instance_modifier_values);
}
void CharacterInstanceList::remove(Node *scene)
{
	Ref<CharacterInstance> char_instance = scene->get_meta("instance_data");
	instance_list.erase(char_instance);
	char_instance.unref();
	scene->queue_delete();
}
Node *CharacterInstanceList::create(const String &gender, const Transform &xform, const Dictionary &slot_conf) {
	Node *root = SceneTree::get_singleton()->get_root();
	CharacterGenderList *gl = CharacterGenderList::get_singleton();
	AccessoryData *ad = AccessoryData::get_singleton();
	CharacterModifiers *cm = CharacterModifiers::get_singleton();
	const CharacterGender &gdata = gl->genders[gender];
	Node *sc = gl->instance(gender);
	root->add_child(sc);
	Spatial *s = Object::cast_to<Spatial>(sc);
	Skeleton *skel = find_node<Skeleton>(sc);
	s->set_transform(xform);
	/* TODO: custom allocator */
	Ref<CharacterInstance> char_instance = memnew(CharacterInstance);
	char_instance->scene_root = root->get_path_to(sc);
	char_instance->gender = &gl->genders[gender];
	char_instance->left_foot_id = skel->find_bone(char_instance->gender->left_foot);
	char_instance->right_foot_id = skel->find_bone(char_instance->gender->right_foot);
	char_instance->pelvis_id = skel->find_bone(char_instance->gender->pelvis);
	assert(char_instance->left_foot_id >= 0);
	assert(char_instance->right_foot_id >= 0);
	assert(char_instance->pelvis_id >= 0);
	cm->create_modifiers();

	for (const String *key = gdata.slot_list.next(NULL);
			key; key = gdata.slot_list.next(key)) {
		const CharacterSlot &slot = gdata.slot_list[*key];
		if (!slot.mandatory)
			continue;
		PoolVector<Dictionary> entries = ad->get_matching_entries(gender, slot.category, slot.match);
		if (entries.size() == 0)
			continue;
		CharacterSlotInstance si;
		si.slot = &slot;
		if (slot_conf.has(*key))
			si.mesh_no = slot_conf[*key];
		else
			si.mesh_no = 0;
		si.char_instance = char_instance.ptr();
		MeshInstance *mi = memnew(MeshInstance);
		mi->set_name(slot.name);
		skel->add_child(mi);
		Ref<ArrayMesh> mesh = ad->get_mesh(entries[si.mesh_no]);
		mi->hide();
		mi->set_mesh(mesh);
		mi->show();
		si.slot_path = sc->get_path_to(mi);
		mi->set_skeleton_path(mi->get_path_to(skel));
		si.dirty = true;
		si.mesh = mesh;
		si.meshdata = NULL;
		si.uv_index = Mesh::ARRAY_TEX_UV2;
		char_instance->slots[slot.name] = si;
	}
	sc->set_meta("instance_data", char_instance);
	instance_list.push_back(char_instance);
	return sc;
}
void CharacterInstanceList::update() {
	for (List<Ref<CharacterInstance> >::Element *e = instance_list.front();
			e;
			e = e->next()) {
		Ref<CharacterInstance> &ci = e->get();
		CharacterModifiers *cm = CharacterModifiers::get_singleton();
		cm->modify_bones(ci.ptr());
		for (const String *key = ci->slots.next(NULL);
				key;
				key = ci->slots.next(key)) {
			CharacterSlotInstance &si = ci->slots[*key];
			if (si.dirty || ci->mod_updated) {
				update_slot(ci.ptr(), &si);
				si.dirty = false;
			}
		}
		ci->mod_updated = false;
	}
}

void CharacterInstanceList::init_slot(CharacterInstance *ci,
		CharacterSlotInstance *si) {
	int i, j;
	Array surface = si->mesh->surface_get_arrays(0);
	const String &orig_path = si->mesh->get_meta("orig_path");
	const PoolVector<Vector2> &uvdata = surface[si->uv_index];
	if (uvdata.size() == 0)
		return;
	const PoolVector<Vector3> &vdata = surface[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector3> &normal = surface[Mesh::ARRAY_NORMAL];
	si->meshdata = memnew_arr(float, vdata.size() * 14);
	si->vertex_count = vdata.size();
	assert(uvdata.size() > 0);
	assert(vdata.size() == uvdata.size());
	assert(si->vertex_count > 0);
	const Vector2 *uvs = uvdata.read().ptr();
	const Vector3 *n = normal.read().ptr();
	const Vector3 *v = vdata.read().ptr();
	for (i = 0; i < uvdata.size(); i++) {
		si->meshdata[i * 14 + 0] = uvs[i][0];
		si->meshdata[i * 14 + 1] = uvs[i][1];
		assert(si->meshdata[i * 14 + 0] >= 0);
		assert(si->meshdata[i * 14 + 1] >= 0);
		assert(si->meshdata[i * 14 + 0] <= 1.0f);
		assert(si->meshdata[i * 14 + 1] <= 1.0f);
		si->meshdata[i * 14 + 2] = v[i][0];
		si->meshdata[i * 14 + 3] = v[i][1];
		si->meshdata[i * 14 + 4] = v[i][2];
		si->meshdata[i * 14 + 5] = n[i][0];
		si->meshdata[i * 14 + 6] = n[i][1];
		si->meshdata[i * 14 + 7] = n[i][2];
		si->meshdata[i * 14 + 8] = v[i][0];
		si->meshdata[i * 14 + 9] = v[i][1];
		si->meshdata[i * 14 + 10] = v[i][2];
		si->meshdata[i * 14 + 11] = n[i][0];
		si->meshdata[i * 14 + 12] = n[i][1];
		si->meshdata[i * 14 + 13] = n[i][2];
	}
	if (!same_verts.has(orig_path)) {
		float eps_dist = 0.0001f;
		for (i = 0; i < vdata.size(); i++) {
			for (j = 0; j < vdata.size(); j++) {
				if (i == j)
					continue;
				if (v[i].distance_squared_to(v[j]) < eps_dist * eps_dist) {
					if (!si->same_verts.has(i)) {
						si->same_verts[i] = Vector<int>();
					}
					si->same_verts[i].push_back(j);
				}
			}
		}
		same_verts[orig_path] = si->same_verts;
	} else
		si->same_verts = same_verts[orig_path];
}

void CharacterInstanceList::update_slot(CharacterInstance *ci,
		CharacterSlotInstance *si) {
	if (!si->mesh.ptr())
		return;

	CharacterModifiers *cm = CharacterModifiers::get_singleton();
	if (si->dirty) {
		init_slot(ci, si);
		si->dirty = false;
	}
	cm->modify(ci, si, ci->mod_values);
}
CharacterInstanceList *CharacterInstanceList::get_singleton() {
	static CharacterInstanceList *cil = NULL;
	if (!cil)
		cil = memnew(CharacterInstanceList);
	return cil;
}
Ref<CharacterInstance> CharacterInstanceList::get_instance(Node *scene) {
	Ref<CharacterInstance> ret = scene->get_meta("instance_data");
	return ret;
}
void CharacterInstanceList::set_mod_value(Node *scene,
		const String &mod, float value) {
	Ref<CharacterInstance> ci = get_instance(scene);
	ci->mod_values[mod] = value;
	ci->mod_updated = true;
}
float CharacterInstanceList::get_mod_value(Node *scene,
		const String &mod) {
	Ref<CharacterInstance> ci = get_instance(scene);
	return ci->mod_values[mod];
}
Node *CharacterInstanceList::get_scene(CharacterInstance *ci) {
	Node *scene = SceneTree::get_singleton()->get_root()->get_node(ci->scene_root);
	return scene;
}
Node *CharacterInstance::get_scene_root() const
{
	Node *root = SceneTree::get_singleton()->get_root();
	Node *scene = root->get_node(scene_root);
	return scene;
}
Skeleton *CharacterInstance::get_skeleton() const
{
	Node *root = SceneTree::get_singleton()->get_root();
	Node *scene = root->get_node(scene_root);
	Skeleton *skel = find_node<Skeleton>(scene);
	return skel;
}
Skeleton *CharacterInstanceList::get_skeleton(Node *scene) const
{
	Skeleton *skel = find_node<Skeleton>(scene);
	return skel;
}
PoolVector<String> CharacterModifiers::get_modifier_list() const {
	PoolVector<String> ret;
	ret.resize(modifiers.size());
	int count = 0;
	for (const String *key = modifiers.next(NULL);
			key;
			key = modifiers.next(key)) {
		ret[count++] = *key;
	}
	ret.resize(count);
	return ret;
}
PoolVector<String> CharacterModifiers::get_base_modifier_list() const {
	PoolVector<String> ret;
	for (const String *key = modifiers.next(NULL);
			key;
			key = modifiers.next(key)) {
		if ((*key).begins_with("base:"))
			ret.push_back((*key).replace("base:", ""));
		if ((*key).begins_with("bone:"))
			ret.push_back((*key).replace("bone:", ""));
	}
	return ret;
}
void CharacterModifiers::init_blend_modifier(const String &name,
		BlendModifierData *bm) {
	MapStorage *ms = MapStorage::get_singleton();
	PoolVector<float> minmax = ms->get_minmax(name);
	if (bm->mod_name.length() == 0)
		bm->mod_name = name;
	int index = 0, i, j;
	for (i = 0; i < 3; i++)
		bm->minp[i] = minmax[index++];
	for (i = 0; i < 3; i++)
		bm->maxp[i] = minmax[index++];
	for (i = 0; i < 3; i++)
		bm->minn[i] = minmax[index++];
	for (i = 0; i < 3; i++)
		bm->maxn[i] = minmax[index++];
	bm->empty = false;
	assert(bm->mod_name.length() > 0);
	assert(bm->mod_name == name);
}
Transform CharacterModifiers::parse_transform(const Dictionary &xformdata)
{
	Transform xform;
	if (xformdata.has("uniform-scale")) {
		float scale = xformdata["uniform-scale"];
		Basis basisd = xform.basis.scaled(Vector3(1, 1, 1) * scale);
		xform.basis = basisd;
	}
	if (xformdata.has("translate")) {
		Array xlate = xformdata["translate"];
		xform.origin = Vector3(xlate[0], xlate[1], xlate[2]);
	}
	return xform;
}
void CharacterModifiers::init_bone_modifier(const String &name,
		BoneModifierData *bm, const Array &parameters) {
	bm->bone_name = parameters[2];
	const Dictionary &xformdata = parameters[3];
	bm->xform = parse_transform(xformdata);
}
void CharacterModifiers::init_bone_group_modifier(const String &name,
		BoneGroupModifierData *bm, const Array &parameters) {
		Array bones = parameters[2];
		Array xformdata = parameters[3];
		int bone_count = bones.size(), i;
		bm->bone_count = bone_count;
		assert(bone_count <= BoneGroupModifierData::MAX_BONES);
		for (i = 0; i < bone_count; i++) {
			bm->bones[i] = -1;
			bm->xforms[i] = parse_transform(xformdata[i]);
			bm->bone_names[i] = bones[i];
		}
}
void CharacterModifiers::create_mod(int type, const String &name, const String &gender, const Array &parameters) {
	switch (type) {
	case ModifierDataBase::TYPE_BLEND:
		if (name.ends_with("_plus") || name.ends_with("_minus")) {
			String group_name = name.replace("_plus", "").replace("_minus", "");
			if (!modifiers.has(group_name))
				create<BlendModifierSymData>(group_name, gender);
			Ref<BlendModifierSymData> mod = modifiers[group_name];
			if (name.ends_with("_plus"))
				init_blend_modifier(name, &mod->plus);
			if (name.ends_with("_minus"))
				init_blend_modifier(name, &mod->minus);
		} else {
			if (modifiers.has(name))
				break;
			create<BlendModifierData>(name, gender);
			Ref<BlendModifierData> mod = modifiers[name];
			init_blend_modifier(name, mod.ptr());
		}
		break;
	case ModifierDataBase::TYPE_BONE:
		create<BoneModifierData>(name, gender);
		{
			Ref<BoneModifierData> mod = modifiers[name];
			init_bone_modifier(name, mod.ptr(), parameters);
		}
		break;
	case ModifierDataBase::TYPE_GROUP:
		create<BoneGroupModifierData>(name, gender);
		{
			Ref<BoneGroupModifierData> mod = modifiers[name];
			init_bone_group_modifier(name, mod.ptr(), parameters);
		}
		break;
	}
}
void CharacterModifiers::create_modifiers()
{
	if (mods_created)
		return;
	MapStorage *ms = MapStorage::get_singleton();
	PoolVector<String> map_list = ms->get_list();
	int i;
	for (i = 0; i < map_list.size(); i++)
		create_mod(ModifierDataBase::TYPE_BLEND, map_list[i], "common");
	mods_created = true;
	ConfigData * cd = ConfigData::get_singleton();
	Dictionary conf = cd->get();
	if (conf.has("bone_modifiers")) {
		Dictionary bone_modifiers = conf["bone_modifiers"];
		for (const Variant *key = bone_modifiers.next(NULL);
				key;
				key = bone_modifiers.next(key)) {
			Array modifier_list = bone_modifiers[*key];
			String gender = *key;
			printf("gender: %ls\n", gender.c_str());
			for (i = 0; i < modifier_list.size(); i++) {
				Array mod = modifier_list[i];
				String mod_type = mod[0];
				String mod_name = mod[1];
				printf("%ls %ls\n",
						mod_type.c_str(),
						mod_name.c_str());
				if (mod_type == "bone")
					create_mod(ModifierDataBase::TYPE_BONE, "bone:" + mod_name, gender, mod);
				else if (mod_type == "bone-group")
					create_mod(ModifierDataBase::TYPE_GROUP, "bone:" + mod_name, gender, mod);
			}
		}
	}
}
void CharacterModifiers::modify(CharacterSlotInstance *si,
		ModifierDataBase *mod,
		float value) {
	MapStorage *ms = MapStorage::get_singleton();
	if (mod->type == ModifierDataBase::TYPE_BLEND) {
		int i, j;
		PoolVector<int> mod_indices;
		mod_indices.resize(si->vertex_count);
		PoolVector<float> mod_data;
		mod_data.resize(si->vertex_count * 6);
		int index_count = 0, data_count = 0;
		PoolVector<int>::Write mod_indices_w = mod_indices.write();
		PoolVector<float>::Write mod_data_w = mod_data.write();
		BlendModifierData *_mod = Object::cast_to<BlendModifierData>(mod);
		assert(_mod->mod_name.length() > 0);
		Ref<Image> img = ms->get_image(_mod->mod_name);
		Ref<Image> nimg = ms->get_normal_image(_mod->mod_name);
		img->lock();
		nimg->lock();
		int width = img->get_width();
		int height = img->get_height();
		for (i = 0; i < si->vertex_count; i++) {
			int vx = (int)(si->meshdata[i * 14 + 0] * (float)(width - 1));
			int vy = (int)(si->meshdata[i * 14 + 1] * (float)(height - 1));
			assert(vx >= 0 && vy >= 0);
			assert(vx < width && vy < height);
			Color c = img->get_pixel(vx, vy);
			Color nc = nimg->get_pixel(vx, vy);
			float pdelta[3], ndelta[3];
			for (j = 0; j < 3; j++) {
				pdelta[j] = _mod->minp[j] + (_mod->maxp[j] - _mod->minp[j]) * c[j];
				ndelta[j] = _mod->minn[j] + (_mod->maxn[j] - _mod->minn[j]) * nc[j];
			}
			const float eps = 0.001f;
			if (pdelta[0] * pdelta[0] + pdelta[1] * pdelta[1] + pdelta[2] * pdelta[2] > eps * eps) {
				mod_indices_w[index_count++] = i;
//				mod_indices.push_back(i);
				for (j = 0; j < 3; j++) {
					mod_data_w[data_count++] = pdelta[j];
					mod_data_w[data_count++] = ndelta[j];
//					mod_data.push_back(pdelta[j]);
//					mod_data.push_back(ndelta[j]);
				}
			}
		}
		img->unlock();
		nimg->unlock();
		for (i = 0; i < index_count; i++) {
			int index = mod_indices[i];
			float vx = mod_data[i * 6 + 0];
			float vy = mod_data[i * 6 + 2];
			float vz = mod_data[i * 6 + 4];
			float nx = mod_data[i * 6 + 1];
			float ny = mod_data[i * 6 + 3];
			float nz = mod_data[i * 6 + 5];
			si->meshdata[index * 14 + 2] -= vx * value;
			si->meshdata[index * 14 + 3] -= vy * value;
			si->meshdata[index * 14 + 4] -= vz * value;
			si->meshdata[index * 14 + 5] -= nx * value;
			si->meshdata[index * 14 + 6] -= ny * value;
			si->meshdata[index * 14 + 7] -= nz * value;
		}
	} else if (mod->type == ModifierDataBase::TYPE_BLEND_SYM) {
		BlendModifierSymData *_mod = Object::cast_to<BlendModifierSymData>(mod);
		if (value >= 0.0f)
			modify(si, &_mod->plus, value);
		else
			modify(si, &_mod->minus, -value);
	}
}
void CharacterModifiers::modify(Skeleton *skel,
		ModifierDataBase *mod,
		float value) {
	if (mod->type == ModifierDataBase::TYPE_BONE) {
		BoneModifierData *_mod = Object::cast_to<BoneModifierData>(mod);
		assert(skel);
		if (_mod->bone_id < 0)
			_mod->bone_id = skel->find_bone(_mod->bone_name);
		assert(_mod->bone_id >= 0);
		skel->set_bone_custom_pose(_mod->bone_id,
				skel->get_bone_custom_pose(_mod->bone_id) *
						Transform().interpolate_with(_mod->xform, value));
	} else if (mod->type == ModifierDataBase::TYPE_GROUP) {
		int i;
		BoneGroupModifierData *_mod = Object::cast_to<BoneGroupModifierData>(mod);
		assert(skel);
		if (_mod->bone_count > 0 && _mod->bones[0] < 0) {
			for (i = 0; i < _mod->bone_count; i++)
				_mod->bones[i] = skel->find_bone(_mod->bone_names[i]);
		}
		for (i = 0; i < _mod->bone_count; i++) {
			/* TODO: gender-specific mods */
			int bone_id = skel->find_bone(_mod->bone_names[i]);
			if (bone_id >= 0) {
				/* Transform bone_xform = skel->get_bone_custom_pose(_mod->bones[i]); */
				Transform bone_xform = skel->get_bone_custom_pose(bone_id);
				bone_xform *= Transform().interpolate_with(_mod->xforms[i], value);
				/* skel->set_bone_custom_pose(_mod->bones[i], bone_xform); */
				skel->set_bone_custom_pose(bone_id, bone_xform);
			}
		}
	}
}
void CharacterModifiers::modify_bones(CharacterInstance *ci)
{
	int i;
	Skeleton *skel = ci->get_skeleton();
	for (i = 0; i < skel->get_bone_count(); i++)
		skel->set_bone_custom_pose(i, Transform());
	Vector3 lf_orig_pos = skel->get_bone_global_pose(ci->left_foot_id).origin;
	Vector3 rf_orig_pos = skel->get_bone_global_pose(ci->right_foot_id).origin;
	for (const String *key = modifiers.next(NULL);
		key;
		key = modifiers.next(key)) {
		Vector<String> splitname = (*key).split(":");
		if (modifiers[*key]->gender == "common" ||
				modifiers[*key]->gender == ci->get_gender_name()) {
			if (splitname[0] == "bone")
				modify(skel, modifiers[*key].ptr(), ci->mod_values[splitname[1]]);
		}
	}
	Vector3 lf_pos = skel->get_bone_global_pose(ci->left_foot_id).origin;
	Transform pelvis_xform = skel->get_bone_custom_pose(ci->pelvis_id);
	pelvis_xform.origin += lf_pos - lf_orig_pos;
	skel->set_bone_custom_pose(ci->pelvis_id, pelvis_xform);
}
void CharacterModifiers::modify(CharacterInstance *ci, CharacterSlotInstance *si,
		const HashMap<String, float> &values) {
	int i, j;
	if (si->slot->blend_skip)
		return;
	Array surface = si->mesh->surface_get_arrays(0);
	assert(si->meshdata);

	for (i = 0; i < si->vertex_count; i++) {
		si->meshdata[i * 14 + 2] = si->meshdata[i * 14 + 8];
		si->meshdata[i * 14 + 3] = si->meshdata[i * 14 + 9];
		si->meshdata[i * 14 + 4] = si->meshdata[i * 14 + 10];
		si->meshdata[i * 14 + 5] = si->meshdata[i * 14 + 11];
		si->meshdata[i * 14 + 6] = si->meshdata[i * 14 + 12];
		si->meshdata[i * 14 + 7] = si->meshdata[i * 14 + 13];
	}
	for (const String *key = modifiers.next(NULL);
			key;
			key = modifiers.next(key)) {
		Vector<String> splitname = (*key).split(":");
		if (values.has(splitname[1]) && fabs(values[splitname[1]]) > 0.001) {
			if (si->slot->helper == splitname[0]) {
				modify(si, modifiers[*key].ptr(), values[splitname[1]]);
			}
		}
	}
	for (i = 0; i < si->vertex_count; i++) {
		if (si->same_verts.has(i)) {
			float vx = si->meshdata[i * 14 + 2];
			float vy = si->meshdata[i * 14 + 3];
			float vz = si->meshdata[i * 14 + 4];
			float nx = si->meshdata[i * 14 + 5];
			float ny = si->meshdata[i * 14 + 6];
			float nz = si->meshdata[i * 14 + 7];
			for (j = 0; j < si->same_verts[i].size(); j++) {
				vx = Math::lerp(vx, si->meshdata[si->same_verts[i][j] * 14 + 2], 0.5f);
				vy = Math::lerp(vy, si->meshdata[si->same_verts[i][j] * 14 + 3], 0.5f);
				vz = Math::lerp(vz, si->meshdata[si->same_verts[i][j] * 14 + 4], 0.5f);
				nx = Math::lerp(nx, si->meshdata[si->same_verts[i][j] * 14 + 5], 0.5f);
				ny = Math::lerp(ny, si->meshdata[si->same_verts[i][j] * 14 + 6], 0.5f);
				nz = Math::lerp(nz, si->meshdata[si->same_verts[i][j] * 14 + 7], 0.5f);
			}
			si->meshdata[i * 14 + 2] = vx;
			si->meshdata[i * 14 + 3] = vy;
			si->meshdata[i * 14 + 4] = vz;
			si->meshdata[i * 14 + 5] = nx;
			si->meshdata[i * 14 + 6] = ny;
			si->meshdata[i * 14 + 7] = nz;
			for (j = 0; j < si->same_verts[i].size(); j++) {
				si->meshdata[si->same_verts[i][j] * 14 + 2] = vx;
				si->meshdata[si->same_verts[i][j] * 14 + 3] = vy;
				si->meshdata[si->same_verts[i][j] * 14 + 4] = vz;
				si->meshdata[si->same_verts[i][j] * 14 + 5] = nx;
				si->meshdata[si->same_verts[i][j] * 14 + 6] = ny;
				si->meshdata[si->same_verts[i][j] * 14 + 7] = nz;
			}
		}
	}
	PoolVector<Vector3> vertices = surface[Mesh::ARRAY_VERTEX];
	PoolVector<Vector3> normals = surface[Mesh::ARRAY_NORMAL];
	PoolVector<Vector3>::Write vertex_w = vertices.write();
	PoolVector<Vector3>::Write normal_w = normals.write();
	for (i = 0; i < si->vertex_count; i++) {
		vertex_w[i].x = si->meshdata[i * 14 + 2];
		vertex_w[i].y = si->meshdata[i * 14 + 3];
		vertex_w[i].z = si->meshdata[i * 14 + 4];
		normal_w[i].x = si->meshdata[i * 14 + 5];
		normal_w[i].y = si->meshdata[i * 14 + 6];
		normal_w[i].z = si->meshdata[i * 14 + 7];
	}
	vertex_w.release();
	normal_w.release();
	surface[Mesh::ARRAY_VERTEX] = vertices;
	surface[Mesh::ARRAY_NORMAL] = normals;
	Ref<Material> mat = si->mesh->surface_get_material(0);
	si->mesh->surface_remove(0);
	si->mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, surface);
	si->mesh->surface_set_material(0, mat);
}

CharacterModifiers *CharacterModifiers::get_singleton() {
	static CharacterModifiers *cm = NULL;
	if (!cm)
		cm = memnew(CharacterModifiers);
	return cm;
}
