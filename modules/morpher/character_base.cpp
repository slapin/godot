#include <scene/3d/mesh_instance.h>
#include <scene/3d/spatial.h>
#include <scene/main/scene_tree.h>
#include <scene/main/viewport.h>
#include "map_storage.h"
#include "character_slot.h"
#include "character_base.h"
void CharacterGenderList::config() {
	const Dictionary &config = ConfigData::get_singleton()->get();
	const Array &gdata = config["genders"];
	int i;
	for (i = 0; i < gdata.size(); i++) {
		const Dictionary &g = gdata[i];
		const String &gname = g["name"];
		const String &scene = g["scene"];
		Error err = OK;
		printf("creating gender %ls\n", gname.c_str());
		Ref<PackedScene> pscene = ResourceLoader::load(scene, "", &err);
		if (err != OK) {
			printf("Could not read resource %ls\n", scene.c_str());
			continue;
		}
		create_gender(gname, pscene);
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
		printf("creating slot %ls\n", slot.name.c_str());
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
	ClassDB::bind_method(D_METHOD("set_mod_value", "scene", "mod", "value"),
			&CharacterInstanceList::set_mod_value);
}
Node *CharacterInstanceList::create(const String &gender, const Transform &xform, const Dictionary &slot_conf) {
    int i;
    Node *root = SceneTree::get_singleton()->get_root();
	CharacterGenderList *gl = CharacterGenderList::get_singleton();
	AccessoryData *ad = AccessoryData::get_singleton();
    CharacterModifiers *cm = CharacterModifiers::get_singleton();
    MapStorage *ms = MapStorage::get_singleton();
	const CharacterGender &gdata = gl->genders[gender];
	Node *sc = gl->instance(gender);
    root->add_child(sc);
	Spatial *s = Object::cast_to<Spatial>(sc);
	Skeleton *skel = find_node<Skeleton>(sc);
	s->set_transform(xform);
	/* TODO: custom allocator */
	Ref<CharacterInstance> char_instance = memnew(CharacterInstance);
	char_instance->scene_root = root->get_path_to(sc);

    PoolVector<String> map_list = ms->get_list();
    for (i = 0; i < map_list.size(); i++) {
        cm->create_mod(ModifierDataBase::TYPE_BLEND, map_list[i]);
        printf("mod: %ls\n", map_list[i].c_str());
    }

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
            for (const String *key = ci->slots.next(NULL);
                key;
                key = ci->slots.next(key)) {
                printf("update slot %ls\n", ci->slots[*key].slot->name.c_str());
                CharacterSlotInstance &si = ci->slots[*key];
                update_slot(ci.ptr(), &si);
            }
    }
}
void CharacterInstanceList::update_slot(CharacterInstance *ci,
        CharacterSlotInstance *si) {
	if (!si->mesh.ptr())
		return;
    
    int i, j;
    CharacterModifiers *cm = CharacterModifiers::get_singleton();

	Array surface = si->mesh->surface_get_arrays(0);
	const PoolVector<Vector2> &uvdata = surface[si->uv_index];
	const PoolVector<Vector3> &vdata = surface[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector3> &normal = surface[Mesh::ARRAY_NORMAL];

	si->meshdata = memnew_arr(float, vdata.size() * 14);
    si->vertex_count = vdata.size();
	const Vector2 *uvs = uvdata.read().ptr();
	const Vector3 *n = normal.read().ptr();
	const Vector3 *v = vdata.read().ptr();
	for (i = 0; i < uvdata.size(); i++) {
		si->meshdata[i * 14 + 0] = uvs[i][0];
		si->meshdata[i * 14 + 1] = uvs[i][1];
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
	float eps_dist = 0.0001f;
	for (i = 0; i < vdata.size(); i++) {
		for (j = 0; j < vdata.size(); j++) {
			if (i == j)
				continue;
			if (v[i].distance_squared_to(v[j]) < eps_dist * eps_dist) {
				if (!si->same_verts.has(i))
					si->same_verts[i] = PoolVector<int>();
				si->same_verts[i].push_back(j);
			}
		}
	}
    cm->modify(ci, si, ci->mod_values);
}
CharacterInstanceList *CharacterInstanceList::get_singleton()
{
    static CharacterInstanceList cil;
    return &cil;
}
Ref<CharacterInstance> CharacterInstanceList::get_instance(Node *scene)
{
    Ref<CharacterInstance> ret = scene->get_meta("instance_data");
    return ret;
}
void CharacterInstanceList::set_mod_value(Node *scene,
        const String &mod, float value)
{
    Ref<CharacterInstance> ci = get_instance(scene);
    ci->mod_values[mod] = value;
}
Node *CharacterInstanceList::get_scene(CharacterInstance *ci)
{
    Node *scene = SceneTree::get_singleton()->get_root()->get_node(ci->scene_root);
    return scene;
}
PoolVector<String> CharacterModifiers::get_modifier_list() const
{
    PoolVector<String> ret;
    for (const String *key = modifiers.next(NULL);
        key;
        key = modifiers.next(key)) {
        ret.push_back(*key);
    }
    return ret;
}
void CharacterModifiers::init_blend_modifier(const String &name,
    BlendModifierData *bm)
{
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
void CharacterModifiers::create_mod(int type, const String &name)
{
    switch(type) {
    case ModifierDataBase::TYPE_BLEND:
        if (name.ends_with("_plus") || name.ends_with("_minus")) {
            String group_name = name.replace("_plus", "").replace("_minus", "");
            if (!modifiers.has(group_name))
                create<BlendModifierSymData>(group_name);
            Ref<BlendModifierSymData> mod = modifiers[group_name];
            if (name.ends_with("_plus"))
                init_blend_modifier(name, &mod->plus);
            if (name.ends_with("_minus"))
                init_blend_modifier(name, &mod->minus);
        } else {
            create<BlendModifierData>(name);
            Ref<BlendModifierData> mod = modifiers[name];
            init_blend_modifier(name, mod.ptr());
        }
        break;
    }
}
void CharacterModifiers::modify(CharacterSlotInstance *si,
    ModifierDataBase *mod, 
    float value)
{
    MapStorage *ms = MapStorage::get_singleton();
    if (mod->type == ModifierDataBase::TYPE_BLEND) {
        int i, j;
       	PoolVector<int> mod_indices;
    	PoolVector<float> mod_data;
        BlendModifierData *_mod = Object::cast_to<BlendModifierData>(mod);
        assert(_mod->mod_name.length() > 0);
        Ref<Image> img = ms->get_image(_mod->mod_name);
        Ref<Image> nimg = ms->get_normal_image(_mod->mod_name);
    	img->lock();
	    nimg->lock();
    	for (i = 0; i < si->vertex_count; i++) {
	    	int vx = (int)(si->meshdata[i * 14 + 0] * (float)img->get_width());
		    int vy = (int)(si->meshdata[i * 14 + 1] * (float)img->get_height());
    		Color c = img->get_pixel(vx, vy);
	    	Color nc = nimg->get_pixel(vx, vy);
		    float pdelta[3], ndelta[3];
		    for (j = 0; j < 3; j++) {
			    pdelta[j] = _mod->minp[j] + (_mod->maxp[j] - _mod->minp[j]) * c[j];
			    ndelta[j] = _mod->minn[j] + (_mod->maxn[j] - _mod->minn[j]) * nc[j];
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
	    img->unlock();
	    nimg->unlock();
       	for (i = 0; i < mod_indices.size(); i++) {
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
void CharacterModifiers::modify(CharacterInstance *ci, CharacterSlotInstance *si,
        const HashMap<String, float> &values)
{
    Array surface = si->mesh->surface_get_arrays(0);
    int i, j;
    for (i = 0; i < si->vertex_count; i++) {
        si->meshdata[i * 14 + 2] =  si->meshdata[i * 14 + 8];
        si->meshdata[i * 14 + 3] =  si->meshdata[i * 14 + 9];
        si->meshdata[i * 14 + 4] =  si->meshdata[i * 14 + 10];
        si->meshdata[i * 14 + 5] =  si->meshdata[i * 14 + 11];
        si->meshdata[i * 14 + 6] =  si->meshdata[i * 14 + 12];
        si->meshdata[i * 14 + 7] =  si->meshdata[i * 14 + 13];
    }
    for (const String *key = modifiers.next(NULL);
        key;
        key = modifiers.next(key)) {
            Vector<String> splitname = (*key).split(":");
            printf("helper: %ls %ls mod %ls\n", si->slot->helper.c_str(), splitname[0].c_str(), splitname[1].c_str());
            if (si->slot->helper == splitname[0])
                if (values.has(splitname[1]) && fabs(values[splitname[1]]) > 0.001) {
                    printf("applying mod: %ls\n", (*key).c_str());
                    modify(si, modifiers[*key].ptr(), values[splitname[1]]);
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

CharacterModifiers *CharacterModifiers::get_singleton()
{
    static CharacterModifiers cm;
    return &cm;
}
