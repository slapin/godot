#ifndef MODIFIERSET_H
#define MODIFIERSET_H

#include <cassert>
#include <core/reference.h>
#include <core/resource.h>
#include <scene/resources/mesh.h>
#include <scene/3d/mesh_instance.h>

#undef MORPH_DEBUG

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
	void modify(float * data, int vertex_count, float value);
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
	int vertex_count;
	bool dirty;
	Array surface;
	struct work_mesh {
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
	void add_modifier(const String &name, Ref<Image> vimage, Ref<Image> nimage, const PoolVector<float> &minmax);
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
			const MeshInstance *mi = dynamic_cast<const MeshInstance *>(item);
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
	int add_work_mesh(Ref<ArrayMesh> mesh);
	int add_work_mesh_scene(Node *node, const String &name)
	{
		int i;
		List<Node *> queue;
		queue.push_back(node);
		Ref<ArrayMesh> mesh;
		bool found = false;
		while(!queue.empty()) {
			Node *item = queue.front()->get();
			queue.pop_front();
			MeshInstance *mi = dynamic_cast<MeshInstance *>(item);
			if (mi && item->get_name() == name) {
				mesh = mi->get_mesh()->duplicate();
				mi->set_mesh(mesh);
				found = true;
				break;
			}
			for (i = 0; i < item->get_child_count(); i++)
				queue.push_back(item->get_child(i));
		}
		if (found)
			return add_work_mesh(mesh);
		else
			return -1;
	}
	void remove_work_mesh(int id);
	void modify();
	void create_uv(int id, Ref<ArrayMesh> mesh, int src_id, Ref<ArrayMesh> src_mesh);
	~ModifierSet()
	{
		if (meshdata)
			memdelete_arr(meshdata);
	}
};
#endif
