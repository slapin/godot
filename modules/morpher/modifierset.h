#ifndef MODIFIERSET_H
#define MODIFIERSET_H

#include <cassert>
#include <core/reference.h>
#include <core/resource.h>
#include <core/bind/core_bind.h>
#include <core/os/file_access.h>
#include <scene/resources/mesh.h>
#include <scene/3d/mesh_instance.h>
#include <scene/3d/skeleton.h>

#undef MORPH_DEBUG

class ModifierBase {
protected:
	int type;
	static const int TYPE_BLEND = 1;
	static const int TYPE_BONE = 2;
	static const int TYPE_SYMMETRY = 3;
	static const int TYPE_PAIR = 4;
	static const int TYPE_GROUP = 5;
	String mod_name;
	bool empty;
	ModifierBase() : empty(true)
	{
	}
};

class ModifierBlend: public ModifierBase {
protected:
	float minp[3];
	float maxp[3];
	float cd[3];
	float minn[3];
	float maxn[3];
	float cdn[3];
	PoolVector<int> mod_indices;
	PoolVector<float> mod_data;
	void modify(float * data, int vertex_count, float value);
	void create_from_images(const String &name,
			const float *meshdata,
			int count,
			Image *vdata,
			Image *ndata,
			const Vector3 &vmin,
			const Vector3 &vmax,
			const Vector3 &nmin,
			const Vector3 &nmax);
};
class ModifierBone: public ModifierBase {
protected:
	int bone_id;
	Transform xform;
	void modify(Skeleton *skel, float value);
	void create_from_bone(const Skeleton *skel, const String &bone, const Transform &xform);
};
class ModifierSymmetry: public ModifierBase  {
protected:
	int bone_from_id;
	Transform bf_parent_xform;
	int bone_to_id;
	Transform bt_parent_xform;
	void modify(Skeleton *skel);
	void create_from_symmetry(const Skeleton * skel,
			const String &bone_left,
			const String &bone_right);
};
class ModifierPair: public ModifierBase  {
protected:
	int bone_left_id;
	int bone_right_id;
	Transform bone_left_xform, bone_right_xform;
	void modify(Skeleton *skel, float value);
	void create_from_pair(const Skeleton * skel,
			const String &bone_left,
			const Transform &xform_left,
			const String &bone_right,
			const Transform &xform_right);
};
class ModifierBoneGroup: public ModifierBase  {
protected:
	PoolVector<int> bones;
	PoolVector<Transform> xforms;
	void modify(Skeleton *skel, float value);
	void create_from_group(const Skeleton * skel,
			const PoolVector<String> &bone_names,
			const PoolVector<Transform> &bone_transforms);
};


template <class M>
class Modifier: public M {
protected:
	friend class ModifierSet;
	friend class ModifierGroup;
	String mod_name;
};

class ModifierGroup {
	friend class ModifierSet;
	String group_name;
	Modifier<ModifierBlend> mod_minus;
	Modifier<ModifierBlend> mod_plus;
	Modifier<ModifierBone> bone;
	Modifier<ModifierSymmetry> symmetry;
	Modifier<ModifierPair> pair;
	Modifier<ModifierBoneGroup> group;
	bool empty;
	int type;
	inline void modify(float * data, int vertex_count, float value)
	{
		if (value > 0.0f) {
			if (!mod_plus.empty)
				mod_plus.modify(data, vertex_count, value);
		} else if (value < 0.0f ){
			if (!mod_minus.empty)
				mod_minus.modify(data, vertex_count, -value);
		}

	}
	inline void create_from_images(const String &name,
			const float *meshdata,
			int count,
			Image *vdata,
			Image *ndata,
			const Vector3 &vmin,
			const Vector3 &vmax,
			const Vector3 &nmin,
			const Vector3 &nmax)
	{
		if (name.ends_with("_plus")) {
				if (empty)
					group_name = name.replace("_plus", "");
				mod_plus.create_from_images(name,
					meshdata,
					count,
					vdata,
					ndata,
					vmin,
					vmax,
					nmin,
					nmax);
		} else if (name.ends_with("_minus")) {
				if (empty)
					group_name = name.replace("_minus", "");
				mod_minus.create_from_images(name,
					meshdata,
					count,
					vdata,
					ndata,
					vmin,
					vmax,
					nmin,
					nmax);
		} else {
				if (empty)
					group_name = name.replace("_plus", "");
				mod_plus.create_from_images(name,
					meshdata,
					count,
					vdata,
					ndata,
					vmin,
					vmax,
					nmin,
					nmax);
		}
		empty = false;
		type = ModifierBase::TYPE_BLEND;
	}
	void create_from_bone(const String &name, const Skeleton *skel, const String &bone_name, const Transform &xform)
	{
		assert(skel);
		bone.create_from_bone(skel, bone_name, xform);
		empty = false;
		type = ModifierBase::TYPE_BONE;
	}
	void create_from_symmetry(const String &name, const Skeleton * skel,
			const String &bone_left,
			const String &bone_right) {
		symmetry.create_from_symmetry(skel, bone_left, bone_right);
		type = ModifierBase::TYPE_SYMMETRY;
	}
	void create_from_pair(const String &name, const Skeleton * skel,
			const String &bone_left,
			const Transform &xform_left,
			const String &bone_right,
			const Transform &xform_right) {
		pair.create_from_pair(skel, bone_left, xform_left, bone_right, xform_right);
		type = ModifierBase::TYPE_PAIR;
	}
	void create_from_group(const String &name, const Skeleton * skel,
				const PoolVector<String> &bone_names,
				const PoolVector<Transform> &bone_transforms)
	{
		group.create_from_group(skel, bone_names, bone_transforms);
		type = ModifierBase::TYPE_GROUP;
	}
	inline void modify(Skeleton *skel, float value)
	{
		switch(type) {
		case ModifierBase::TYPE_BONE:
			bone.modify(skel, value);
			break;
		case ModifierBase::TYPE_PAIR:
			pair.modify(skel, value);
			break;
		case ModifierBase::TYPE_GROUP:
			group.modify(skel, value);
			break;
		}
	}
	inline void modify(Skeleton *skel)
	{
		symmetry.modify(skel);
	}
	ModifierGroup(): empty(true)
	{
	}
};

class ModifierSet: public Reference {
	GDCLASS(ModifierSet, Reference)
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
	void add_mesh_scene(const Node *node, const String &name)
	{
		int i;
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
#endif

class CharacterModifierSet: public Reference {
	GDCLASS(CharacterModifierSet, Reference)
protected:
	HashMap<String, ModifierSet *> mods;
	String base_name;
	static void _bind_methods();
	bool dirty;
	HashMap<String, String> helpers;
public:
	CharacterModifierSet(): base_name("base"), dirty(false)
	{
	}
	void set_base_name(const String &name)
	{
		base_name = name;
	}
	Ref<ModifierSet> create(const String &name)
	{
		mods[name] = memnew(ModifierSet);
		Ref<ModifierSet> ret = mods[name];
		return ret;
	}
	void add_mesh_scene(Node *node)
	{
		for (const String *key = mods.next(NULL);
				key;
				key = mods.next(key)) {
			mods[*key]->add_mesh_scene(node, *key);
		}
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
};

