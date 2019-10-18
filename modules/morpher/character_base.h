#ifndef CHARACTER_BASE_H
#define CHARACTER_BASE_H
#include "character_slot.h"
#include "config_data.h"
#include <core/io/json.h>
#include <core/io/resource_loader.h>
#include <core/os/file_access.h>
#include <core/reference.h>
#include <core/resource.h>
#include <scene/resources/packed_scene.h>
#include <cassert>

/* modifier data classes */

class ModifierDataBase : public Reference {
protected:
	friend class CharacterModifiers;
	int type;
	String gender;

public:
	static const int TYPE_BLEND = 1;
	static const int TYPE_BLEND_SYM = 2;
	static const int TYPE_BONE = 3;
	static const int TYPE_SYMMETRY = 4;
	static const int TYPE_PAIR = 5;
	static const int TYPE_GROUP = 6;

protected:
	String mod_name;
	bool empty;
	ModifierDataBase() :
			empty(true),
       			gender("common")
	{
	}
};

class BlendModifierData : public ModifierDataBase {
protected:
	friend class BlendModifierSymData;
	friend class CharacterModifiers;
	float minp[3];
	float maxp[3];
	float cd[3];
	float minn[3];
	float maxn[3];
	float cdn[3];
	PoolVector<int> mod_indices;
	PoolVector<float> mod_data;

public:
	BlendModifierData() {
		type = TYPE_BLEND;
	}
};
class BlendModifierSymData : public ModifierDataBase {
	friend class CharacterModifiers;
	BlendModifierData plus;
	BlendModifierData minus;

public:
	BlendModifierSymData() {
		type = TYPE_BLEND_SYM;
	}
};
class BoneModifierData : public ModifierDataBase {
	friend class CharacterModifiers;
	String bone_name;
	Transform xform;
	int bone_id;
public:
	BoneModifierData(): bone_id(-1) {
		type = TYPE_BONE;
	}
};
class BoneGroupModifierData : public ModifierDataBase {
	friend class CharacterModifiers;
	static const int MAX_BONES = 32;
	int bones[MAX_BONES];
	Transform xforms[MAX_BONES];
	String bone_names[MAX_BONES];
	int bone_count;
public:
	BoneGroupModifierData(): bone_count(0) {
		type = TYPE_GROUP;
	}
	~BoneGroupModifierData()
	{
	}
};
class CharacterSlotInstance;
class Skeleton;
class CharacterModifiers : public Reference {
	HashMap<String, Ref<ModifierDataBase> > modifiers;
	template <class T>
	void create(const String &name, const String &gender) {
		Ref<T> mod = memnew(T);
		mod->mod_name = name;
		mod->gender = gender;
		modifiers[name] = mod;
	}

protected:
	bool mods_created;
	void init_blend_modifier(const String &name,
			BlendModifierData *bm);
	void init_bone_modifier(const String &name,
			BoneModifierData *bm,
			const Array &parameters);
	void init_bone_group_modifier(const String &name,
			BoneGroupModifierData *bm,
			const Array &parameters);
	Transform parse_transform(const Dictionary &xformdata);
	void create_mod(int type, const String &name, const String &gender, const Array &parameters = Array());

public:
	CharacterModifiers() : mods_created(false)
	{
	}
	PoolVector<String> get_modifier_list() const;
	PoolVector<String> get_base_modifier_list() const;
	void create_modifiers();
	void modify_bones(CharacterInstance *ci);
	void modify(CharacterSlotInstance *si, ModifierDataBase *mod, float value);
	void modify(Skeleton *skel, ModifierDataBase *mod, float value);
	void modify(CharacterInstance *ci, CharacterSlotInstance *si,
			const HashMap<String, float> &values);
	static CharacterModifiers *get_singleton();
	~CharacterModifiers()
	{
	}
};

class CharacterGender {
	friend class CharacterGenderList;
	friend class CharacterInstanceList;
	friend class CharacterInstance;
	String name;
	Ref<PackedScene> scene;
	HashMap<String, CharacterSlot> slot_list;
	String left_foot;
	String right_foot;
	String pelvis;
};

class CharacterGenderList : public Reference {
	friend class CharacterInstanceList;
	GDCLASS(CharacterGenderList, Reference)
	CharacterGenderList();

protected:
	HashMap<String, CharacterGender> genders;
	static void _bind_methods();
	Node *instance(const String &gender) {
		const CharacterGender &g = genders[gender];
		Node *ret = g.scene->instance();
		return ret;
	}

public:
	void config();
	void create_gender(const String &name, Ref<PackedScene> base);
	void remove_gender(const String &name);
	static CharacterGenderList *get_singleton() {
		static CharacterGenderList *gl = NULL;
		if (!gl)
			gl = memnew(CharacterGenderList);
		return gl;
	}
};
class CharacterInstance : public Reference {
	GDCLASS(CharacterInstance, Reference)
	friend class CharacterInstanceList;
	friend class CharacterModifiers;
	NodePath scene_root;
	HashMap<String, CharacterSlotInstance> slots;
	HashMap<String, float> mod_values;
	int left_foot_id;
	int right_foot_id;
	int pelvis_id;
	bool mod_updated;
	CharacterGender *gender;
	CharacterInstance(): mod_updated(false), gender(NULL)
	{
	}
	Node *get_scene_root() const;
	Skeleton *get_skeleton() const;
	const String &get_gender_name() const
	{
		assert(gender);
		return gender->name;
	}
};

class CharacterInstanceList : public Reference {
	GDCLASS(CharacterInstanceList, Reference)
	List<Ref<CharacterInstance> > instance_list;

protected:
	HashMap<String, HashMap<int, Vector<int> > > same_verts;
	static void _bind_methods();
protected:
	void init_slot(CharacterInstance *ci,
			CharacterSlotInstance *si);
	void update_slot(CharacterInstance *ci,
			CharacterSlotInstance *si);

public:
	CharacterInstanceList() {
	}
	void remove(Node *scene);
	Node *create(const String &gender, const Transform &xform, const Dictionary &slot_conf);
	Ref<CharacterInstance> get_instance(Node *scene);
	void set_mod_value(Node *scene,
			const String &mod, float value);
	Node *get_scene(CharacterInstance *ci);
	PoolVector<String> get_modifier_list()
	{
		return CharacterModifiers::get_singleton()->get_modifier_list();
	}
	PoolVector<String> get_base_modifier_list()
	{
		return CharacterModifiers::get_singleton()->get_base_modifier_list();
	}
	Dictionary get_instance_modifier_values(Node *scene)
	{
		Dictionary ret;
		Ref<CharacterInstance> ci = get_instance(scene);
		for (const String *key = ci->mod_values.next(NULL);
				key;
				key = ci->mod_values.next(key))
			ret[*key] = ci->mod_values[*key];
		return ret;
	}
	float get_mod_value(Node *scene, const String &mod);
	Skeleton *get_skeleton(Node *scene) const;
	static CharacterInstanceList *get_singleton();
	void update();
};

#endif
