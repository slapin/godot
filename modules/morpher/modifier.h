#ifndef MODIFIER_H
#define MODIFIER_H
#include <cassert>
#include <core/reference.h>
#include <core/image.h>
#include <scene/3d/skeleton.h>
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

#endif
