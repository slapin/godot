#ifndef MODIFIERSET_H
#define MODIFIERSET_H

#include <cassert>
#include <core/reference.h>
#include <core/resource.h>
#include <scene/resources/mesh.h>

class Modifier {
protected:
	friend class ModifierSet;
	static void _bind_methods();
	float minp[3];
	float maxp[3];
	float cd[3];
	float minn[3];
	float maxn[3];
	float cdn[3];
	bool empty;
	float value;
	PoolVector<int> mod_indices;
	PoolVector<float> mod_data;
	Modifier() : empty(true)
	{
	}
	void create_from_images(const float *meshdata,
			int count,
			Image *vdata,
			Image *ndata,
			const Vector3 &vmin,
			const Vector3 &vmax,
			const Vector3 &nmin,
			const Vector3 &nmax);
	void modify(float * data, int vertex_count);
};
class ModifierSet: public Reference {
	GDCLASS(ModifierSet, Reference)
protected:
	static void _bind_methods();
	Modifier modifiers[256];
	int mod_count;
	HashMap<String, int> name2mod;
	float *meshdata;
	int uv_index;
	HashMap<int, PoolVector<int> > same_verts;
	Ref<ArrayMesh> work_mesh;
	Ref<Material> mat;
	Array surface;
	int vertex_count;
	bool dirty;
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
	void add_modifier(const String &name, Ref<Image> vimage, Ref<Image> nimage, const PoolVector<float> &minmax);
	void add_mesh(const String &name, Ref<ArrayMesh> mesh);
	void set_modifier_value(const String &name, float value)
	{
		if (!name2mod.has(name))
			return;
		dirty = true;
		modifiers[name2mod[name]].value = value;
	}
	float get_modifier_value(const String &name)
	{
		if (!name2mod.has(name))
			return 0.0f;
		return modifiers[name2mod[name]].value;
	}
	void modify();
	~ModifierSet()
	{
		if (meshdata)
			memdelete_arr(meshdata);
	}
};
#endif
