#include "character_slot.h"
CharacterSlotInstance::CharacterSlotInstance(): slot(NULL), mesh_no(0), dirty(false)
{
}
CharacterSlotInstance::~CharacterSlotInstance()
{
    memdelete_arr(meshdata);
}
