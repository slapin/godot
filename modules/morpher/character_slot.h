#ifndef CHARACTER_SLOT_H
#define CHARACTER_SLOT_H
#include <core/reference.h>
#include <scene/resources/mesh.h>
class CharacterSlot {
	friend class CharacterGenderList;
	friend class CharacterInstanceList;
	friend class CharacterModifiers;
	String name;
	String category;
	String match;
	String helper;
	bool mandatory;
};

class CharacterInstance;

class CharacterSlotInstance {
	friend class CharacterInstanceList;
	friend class CharacterModifiers;
	const CharacterSlot *slot;
	NodePath slot_path;
	int mesh_no;
	bool dirty;
	Ref<ArrayMesh> mesh;
	float *meshdata;
	int vertex_count;
	int uv_index;
	HashMap<int, Vector<int> > same_verts;
	CharacterInstance *char_instance;

public:
	CharacterSlotInstance();
	~CharacterSlotInstance();
};
#endif
