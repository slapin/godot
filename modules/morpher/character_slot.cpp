#include "character_slot.h"
CharacterSlotInstance::CharacterSlotInstance() :
		slot(NULL),
		mesh_no(0),
		dirty(false), meshdata(NULL) {
}
CharacterSlotInstance::~CharacterSlotInstance() {
	if (meshdata)
		memdelete_arr(meshdata);
}
