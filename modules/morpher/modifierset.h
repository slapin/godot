#ifndef MODIFIERSET_H
#define MODIFIERSET_H

#include <cassert>
#include <core/reference.h>
#include <core/resource.h>
#include <core/bind/core_bind.h>
#include <core/os/file_access.h>
#include <core/io/json.h>
#include <core/io/resource_loader.h>
#include <scene/resources/mesh.h>
#include <scene/resources/packed_scene.h>
#include <scene/3d/mesh_instance.h>
#include <scene/3d/skeleton.h>
#include "modifier.h"

#undef MORPH_DEBUG

class MapStorage {
	struct datablock {
		String name;
		float minp[3], maxp[3], min_normal[3], max_normal[3];
		String helper;
		int width;
		int height;
		int format;
		int dec_size;
		PoolVector<uint8_t> buf;
		int width_normal;
		int height_normal;
		int format_normal;
		int dec_size_normal;
		PoolVector<uint8_t> buf_normal;
	};
	HashMap<String, struct datablock> data;
	Dictionary config;
	void load();
	MapStorage()
	{
		FileAccess *fd = FileAccess::open("res://characters/config.json", FileAccess::READ);
		assert(fd);
		String confdata = fd->get_as_utf8_string();
		fd->close();
		String err;
		int err_line;
		Variant adata;
		JSON::parse(confdata, adata, err, err_line);
		config = adata;
		load();
	}
public:
	PoolVector<String> get_list() const
	{
		PoolVector<String> ret;
		for (const String *key = data.next(NULL);
			key; key = data.next(key)) {
				ret.push_back(*key);
		}
		return ret;
	}
	Ref<Image> get_image(const String &name) const;
	Ref<Image> get_normal_image(const String &name) const;
	PoolVector<float> get_minmax(const String &shape_name);
	void remove_map(const String &name)
	{
		data.erase(name);
	}
	static MapStorage *get_singleton()
	{
		static MapStorage ms;
		return &ms;
	}
};

class ModifierSet {
protected:
	ModifierGroup modifiers[256];
	int mod_count;
	float *meshdata;
	HashMap<String, int> name2mod;
	int uv_index;
	HashMap<int, PoolVector<int> > same_verts;
	int vertex_count;
	bool dirty;
	Array surface;
	static void _bind_methods();
	struct work_mesh {
		NodePath skeleton;
		Ref<Material> mat;
		Ref<ArrayMesh> work_mesh;
		HashMap<int, float> mod_values;
	};
	PoolVector<struct work_mesh> work_meshes;
public:
	const int UV_OFFT = 0;
	const int VERTEX_OFFT = 2;
	const int NORMAL_OFFT = 5;
	const int O_VERTEX_OFFT = 8;
	const int O_NORMAL_OFFT = 11;
	void set_uv_index(int index)
	{
		switch(index) {
		case 0:
			uv_index = Mesh::ARRAY_TEX_UV;
			break;
		case 1:
			uv_index = Mesh::ARRAY_TEX_UV2;
			break;
		default:
			uv_index = Mesh::ARRAY_TEX_UV;
			break;
		}
	}
	ModifierSet(): mod_count(0),
		meshdata(0),
		uv_index(Mesh::ARRAY_TEX_UV),
		vertex_count(0), dirty(false)
	{
	}
	void add_modifier(const String &name, Ref<Image> vimage,
			Ref<Image> nimage, const PoolVector<float> &minmax);
	void _add_modifier(const String &name, Image *vimage,
			Image *nimage, const float *minmax);
	void _add_modifier(const String &name, const Skeleton *skel,
			const String &bone, const Transform &xform);
	void _add_modifier(const String &name, const Skeleton *skel,
		const String &bone_from, const String &bone_to);
	void _add_modifier(const String &name, const Skeleton *skel,
		const String &bone_left,
		const Transform &left_xform,
		const String &bone_right,
		const Transform &right_xform);
	void _add_modifier(const String &name, const Skeleton *skel,
		const PoolVector<String> &bone_names,
		const PoolVector<Transform> bone_transforms);
	void add_mesh(const String &name, Ref<ArrayMesh> mesh);
	bool add_mesh_scene(const Node *node, const String &name)
	{
		int i;
		bool ret = true;
		List<const Node *> queue;
		queue.push_back(node);
		Ref<ArrayMesh> mesh;
		bool found = false;
		while(!queue.empty()) {
			const Node *item = queue.front()->get();
			queue.pop_front();
			const MeshInstance *mi = Object::cast_to<MeshInstance>(item);
			if (mi && item->get_name() == name) {
				mesh = mi->get_mesh();
				found = true;
				break;
			}
			for (i = 0; i < item->get_child_count(); i++)
				queue.push_back(item->get_child(i));
		}
		if (found)
			add_mesh(name, mesh);
		else
			ret = false;
		
		return ret;
	}
	void set_modifier_value(int id, const String &name, float value)
	{
#ifdef MORPH_DEBUG
		printf("try mod %d %ls %f\n", id, name.c_str(), value);
#endif
		if (!name2mod.has(name))
			return;
#ifdef MORPH_DEBUG
		printf("has mod %d %ls %f %d\n", id, name.c_str(), value, work_meshes.size());
#endif
		if (id < 0 || id >= work_meshes.size())
			return;
#ifdef MORPH_DEBUG
		printf("has mesh %d %ls %f\n", id, name.c_str(), value);
#endif
		dirty = true;
#ifdef MORPH_DEBUG
		printf("mod %d %ls %f\n", id, name.c_str(), value);
#endif
		work_meshes.write()[id].mod_values[name2mod[name]] = value;
	}
	float get_modifier_value(int id, const String &name)
	{
		if (id < 0 || id >= work_meshes.size())
			return 0.0f;
		if (!name2mod.has(name))
			return 0.0f;
		return work_meshes[id].mod_values[name2mod[name]];
	}
	int add_work_mesh(Ref<ArrayMesh> mesh, const NodePath &skel);
	template <class T>
	static inline T * find_node(Node * node, const String &name = "")
	{
		int i;
		T *ret = NULL;
		List<Node *> queue;
		queue.push_back(node);
		while(!queue.empty()) {
			Node *item = queue.front()->get();
			queue.pop_front();
			ret = Object::cast_to<T>(item);
			if (ret && (name.length() == 0 || ret->get_name() == name))
				break;
			for (i = 0; i < item->get_child_count(); i++)
				queue.push_back(item->get_child(i));
		}
		return ret;
	}
	int add_work_mesh_scene(Node *node, const String &name)
	{
		int i;
		List<Node *> queue;
		queue.push_back(node);
		Ref<ArrayMesh> mesh;
		NodePath skel_path;
		bool found = false;
		while(!queue.empty()) {
			Node *item = queue.front()->get();
			queue.pop_front();
			MeshInstance *mi = Object::cast_to<MeshInstance>(item);
			if (mi && item->get_name() == name) {
				mesh = mi->get_mesh()->duplicate();
				mi->set_mesh(mesh);
				skel_path = mi->get_skeleton_path();
				Node *skel_node = mi->get_node(skel_path);
				skel_path = skel_node->get_owner()->get_path_to(skel_node);
				found = true;
				break;
			}
			for (i = 0; i < item->get_child_count(); i++)
				queue.push_back(item->get_child(i));
		}
		if (found)
			return add_work_mesh(mesh, skel_path);
		else
			return -1;
	}
	void remove_work_mesh(int id);
	void modify(Node *scene);
	void create_uv(int id, Ref<ArrayMesh> mesh, int src_id, Ref<ArrayMesh> src_mesh);
	PoolVector<String> get_modifier_list()
	{
		PoolVector<String> ret;
		for (const String *key = name2mod.next(NULL); key; key = name2mod.next(key)) {
			if (modifiers[name2mod[*key]].type != ModifierBase::TYPE_SYMMETRY)
				ret.push_back(*key);
		}
		return ret;
	}
	~ModifierSet()
	{
		if (meshdata)
			memdelete_arr(meshdata);
	}
};

class CharacterModifierSet: public Reference {
	GDCLASS(CharacterModifierSet, Reference)
protected:
	HashMap<String, ModifierSet *> mods;
	String base_name;
	static void _bind_methods();
	bool dirty;
	HashMap<String, String> helpers;
	HashMap<String, String> slots;
	Ref<PackedScene> ch;
	Dictionary accessories;
public:
	CharacterModifierSet(): base_name("base"), dirty(false)
	{
		FileAccess *fd = FileAccess::open("res://characters/accessory.json", FileAccess::READ);
		String confdata = fd->get_as_utf8_string();
		fd->close();
		String err;
		int err_line;
		Variant adata;
		JSON::parse(confdata, adata, err, err_line);
		accessories = adata;
	}
	void set_base_name(const String &name)
	{
		base_name = name;
	}
protected:
	ModifierSet *create(const String &name)
	{
		if (mods.has(name))
			return NULL;
		mods[name] = memnew(ModifierSet);
		ModifierSet *ret = mods[name];
		return ret;
	}
public:
	void add_mesh_scene(Ref<PackedScene> ps)
	{
		ch = ps;
		Node *node = ps->instance();
		List<String> bad_slots;
		for (const String *key = mods.next(NULL);
				key;
				key = mods.next(key)) {
			if (!mods[*key]->add_mesh_scene(node, *key))
				bad_slots.push_back(*key);
		}
		for (List<String>::Element *e = bad_slots.front();
				e;
				e = e->next()) {
					String d = e->get();
					printf("bad slot: %ls\n", d.c_str());
					memfree(mods[d]);
					mods.erase(d);
		}
		node->queue_delete();
	}
	int add_work_mesh_scene(Node *node)
	{
		int ret = -1;
		for (const String *key = mods.next(NULL);
				key;
				key = mods.next(key)) {
			int nret = mods[*key]->add_work_mesh_scene(node, *key);
			if (ret >= 0)
				assert(nret == ret);
			else
				ret = nret;
		}
		return ret;
	}
	void remove_work_meshes(int id)
	{
		for (const String *key = mods.next(NULL);
				key;
				key = mods.next(key)) {
			printf("%ls\n", key->c_str());
			mods[*key]->remove_work_mesh(id);
		}
	}
	void set_modifier_value(int id, const String &name, float value)
	{
		for (const String *key = mods.next(NULL);
				key;
				key = mods.next(key)) {
			if (key)
				mods[*key]->set_modifier_value(id, name, value);
		}
		dirty = true;
	}
	float get_modifier_value(int id, const String &name)
	{
		assert(mods.has(base_name));
		return mods[base_name]->get_modifier_value(id, name);
	}
	PoolVector<String> get_modifier_list()
	{
		assert(mods.has(base_name));
		return mods[base_name]->get_modifier_list();
	}
	void modify(Node *scene)
	{
		if (!dirty)
			return;
		for (const String *key = mods.next(NULL);
				key;
				key = mods.next(key)) {
			mods[*key]->modify(scene);
		}
		dirty = false;
	}
	void load(Ref<_File> fd);
	void set_helper(const String &mod_name, const String &name)
	{
		helpers[mod_name] = name;
	}
	void add_bone_modifier(const String &name, Node *scene, const String &bone_name, const Transform &xform)
	{
		List<const Node *> queue;
		queue.push_back(scene);
		Skeleton *skel = ModifierSet::find_node<Skeleton>(scene);
		if (skel)
			mods[base_name]->_add_modifier(name, skel, bone_name, xform);
	}
	void add_bone_modifier_symmetry(const String &name,
			Node *scene,
			const String &bone_left,
			const String &bone_right)
	{
		Skeleton *skel = ModifierSet::find_node<Skeleton>(scene);
		if (skel)
			mods[base_name]->_add_modifier(name, skel, bone_left, bone_right);
	}
	void add_bone_modifier_pair(const String &name,
			Node *scene,
			Array left,
			Array right)
	{
		Skeleton *skel = ModifierSet::find_node<Skeleton>(scene);
		assert(skel && left.size() == 2 && right.size() == 2);
		const String &bone_left = left[0];
		const Transform &xform_left = left[1];
		const String &bone_right = right[0];
		const Transform &xform_right = right[1];
		if (skel)
			mods[base_name]->_add_modifier(name, skel, bone_left, xform_left, bone_right, xform_right);
	}
	void add_bone_modifier_group(const String &name,
			Node *scene,
			const PoolVector<String> &bone_names,
			const Array &bone_transforms)
	{
		Skeleton *skel = ModifierSet::find_node<Skeleton>(scene);
		PoolVector<Transform> xforms;
		assert(skel && bone_names.size() == bone_transforms.size());
		xforms.resize(bone_transforms.size());
		PoolVector<Transform>::Write xformsw = xforms.write();

		if (skel) {
			int i;
			for (i = 0; i < bone_transforms.size(); i++)
				xformsw[i] = bone_transforms[i];
			xformsw.release();
			mods[base_name]->_add_modifier(name, skel, bone_names, xforms);
		}
	}
	void add_slot(const String &name, const String &helper, int uv_index)
	{
		slots[name] = helper;
		ModifierSet *data = create(name);
		assert(data);
		data->set_uv_index(uv_index);
		set_helper(name, helper);
	}
	Node *spawn()
	{
		Node *ret = ch->instance();
		ret->set_meta("mesh_id", add_work_mesh_scene(ret));
		ret->set_meta("mod", this);
		return ret;
	}
	void remove(Node *node)
	{
		int mesh_id = node->get_meta("mesh_id");
		remove_work_meshes(mesh_id);
		node->queue_delete();
	}
	Node *get_slot(Node *node, const String &name)
	{
		if (!slots.has(name))
			return NULL;
		MeshInstance *mi = ModifierSet::find_node<MeshInstance>(node, name);
		return mi;
	}
	void hide_slot(Node *node, const String &name)
	{
		MeshInstance *mi = ModifierSet::find_node<MeshInstance>(node, name);
		if (mi)
			mi->hide();
	}
	void show_slot(Node *node, const String &name)
	{
		MeshInstance *mi = ModifierSet::find_node<MeshInstance>(node, name);
		if (mi)
			mi->show();
	}
	void set_accessory(Node *node, const String &slot_name, const String &gender,
					const String &atype, const String &aname);
	PoolVector<String> get_acessory_list(const String &gender,
					const String &atype, const String &name_start)
	{
		int i;
		PoolVector<String> ret;
		if (!accessories.has(gender))
			return ret;
		const Dictionary &items = accessories[gender];
		if (!items.has(atype))
			return ret;
		const Dictionary &category = items[atype];
		Array key_list = category.keys();
		for (i = 0; i < key_list.size(); i++) {
			const String &name = key_list[i];
			if (name_start.length() == 0 || name.begins_with(name_start))
				ret.push_back(name);
		}
		return ret;
	}
	void process();
};

#endif
