#ifndef CHARACTER_BASE_H
#define CHARACTER_BASE_H
#include <cassert>
#include <core/reference.h>
#include <core/resource.h>
#include <scene/resources/packed_scene.h>
#include <core/os/file_access.h>
#include <core/io/json.h>
#include <core/io/resource_loader.h>
#include "config_data.h"
#include "character_slot.h"

/* modifier data classes */

class ModifierDataBase: public Reference {
protected:
    friend class CharacterModifiers;
	int type;
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
	ModifierDataBase() : empty(true)
	{
	}
};

class BlendModifierData: public ModifierDataBase {
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
	BlendModifierData()
	{
		type = TYPE_BLEND;
	}
};
class BlendModifierSymData: public ModifierDataBase {
    friend class CharacterModifiers;
	BlendModifierData plus;
	BlendModifierData minus;
public:
	BlendModifierSymData()
	{
		type = TYPE_BLEND_SYM;
	}
};
class CharacterSlotInstance;
class CharacterModifiers: public Reference {
    HashMap<String, Ref<ModifierDataBase> > modifiers;
    template <class T>
    void create(const String &name)
    {
        Ref<T> mod = memnew(T);
        mod->mod_name = name;
        modifiers[name] = mod;
    }
protected:
    void init_blend_modifier(const String &name,
        BlendModifierData *bm);
public:
    PoolVector<String> get_modifier_list() const;
    void create_mod(int type, const String &name);
    void modify(CharacterSlotInstance *si, ModifierDataBase *mod, float value);
    void modify(CharacterInstance *ci, CharacterSlotInstance *si,
        const HashMap<String, float> &values);
    static CharacterModifiers *get_singleton();
};

class CharacterGender {
    friend class CharacterGenderList;
    friend class CharacterInstanceList;
    String name;
    Ref<PackedScene> scene;
    HashMap<String, CharacterSlot> slot_list;
};


class CharacterGenderList: public Reference {
    friend class CharacterInstanceList;
    GDCLASS(CharacterGenderList, Reference)
    CharacterGenderList();
protected:
    HashMap<String, CharacterGender> genders;
    static void _bind_methods();
    Node *instance(const String &gender)
    {
        const CharacterGender &g = genders[gender];
        Node *ret = g.scene->instance();
        return ret;
    }

public:
    void config();
    void create_gender(const String &name, Ref<PackedScene> base);
    void remove_gender(const String &name);
    static CharacterGenderList *get_singleton()
    {
        static CharacterGenderList gl;
        return &gl;
    }
};

class CharacterInstance: public Reference {
   	GDCLASS(CharacterInstance, Reference)
    friend class CharacterInstanceList;
    NodePath scene_root;
    HashMap<String, CharacterSlotInstance> slots;
    HashMap<String, float> mod_values;
};

class CharacterInstanceList: public Reference {
   	GDCLASS(CharacterInstanceList, Reference)
    List<Ref<CharacterInstance> > instance_list;
protected:
   	static void _bind_methods();
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
protected:
    void update_slot(CharacterInstance *ci,
        CharacterSlotInstance *si);
public:
    CharacterInstanceList()
    {
    }
    Node *create(const String &gender, const Transform &xform, const Dictionary &slot_conf);
    Ref<CharacterInstance> get_instance(Node *scene);
    void set_mod_value(Node *scene,
        const String &mod, float value);
    Node *get_scene(CharacterInstance *ci);
    static CharacterInstanceList *get_singleton();
    void update();
};

#endif
