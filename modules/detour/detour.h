/*************************************************************************/
/*  detour.h                                                             */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2018 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2018 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/
#ifndef DETOUR_H
#define DETOUR_H
#include "scene/3d/spatial.h"
#include "scene/resources/mesh.h"
#include "resource.h"
class DetourNavigation : public Spatial {
	GDCLASS(DetourNavigation, Spatial);
	DetourNavigation() : Spatial()
	{
	}
	static void _bind_methods();
};
class DetourNavigationOffmeshConnection : public Spatial {
	GDCLASS(DetourNavigationOffmeshConnection, Spatial);
	static void _bind_methods();
public:
	Vector3 endpoint;
};

class DetourNavigationArea : public Spatial {
	GDCLASS(DetourNavigationArea, Spatial);
	static void _bind_methods();
public:
	AABB bounds;
	int id;
	unsigned int flags;
};

class dtNavMesh;
class dtNavMeshParams;
class dtNavMeshQuery;
class dtQueryFilter;
#define SETGET(x, t) \
	t x; \
	void set_ ## x(t v) { x = v; } \
	t get_ ## x() { return x; }
class DetourNavigationMesh : public Resource {
	GDCLASS(DetourNavigationMesh, Resource);
	dtNavMesh *navmesh;
	String group;
	bool initialized;
	static void _bind_methods();
protected:
	void release_navmesh();
	int num_tiles_x;
	int num_tiles_z;
public:
	enum partition_t {
		PARTITION_WATERSHED,
		PARTITION_MONOTONE,
	};
	void set_num_tiles(int gridW, int gridH)
	{
		num_tiles_x = (gridW + tile_size - 1) / tile_size;
		num_tiles_z = (gridH + tile_size - 1) / tile_size;
	}
	int get_num_tiles_x()
	{
		return num_tiles_x;
	}
	int get_num_tiles_z()
	{
		return num_tiles_z;
	}
	SETGET(partition_type, int)
	SETGET(tile_size, int)
	SETGET(cell_size, real_t)
	SETGET(cell_height, real_t)
	// real_t cell_height;
	SETGET(agent_height, real_t)
	// real_t agent_height;
	SETGET(agent_radius, real_t)
	// real_t agent_radius;
	SETGET(agent_max_climb, real_t)
	// real_t agent_max_climb;
	SETGET(agent_max_slope, real_t)
	// real_t agent_max_slope;
	SETGET(region_min_size, real_t)
	// real_t region_min_size;
	SETGET(region_merge_size, real_t)
	// real_t region_merge_size;
	SETGET(edge_max_length, real_t)
	// real_t edge_max_length;
	SETGET(edge_max_error, real_t)
	// real_t edge_max_error;
	SETGET(detail_sample_distance, real_t)
	// real_t detail_sample_distance;
	SETGET(detail_sample_max_error, real_t)
	// real_t detail_sample_max_error;
	AABB bounding_box;
	SETGET(padding, Vector3)
	// Vector3 padding;
	void set_group(const String& group);
	bool alloc();
	bool init(dtNavMeshParams *params);
	void set_data(const Dictionary &p_value);
	Dictionary get_data();
	const String& get_group() const
	{
		return group;
	}
	dtNavMesh *get_navmesh()
	{
		return navmesh;
	}
	DetourNavigationMesh();
};
#undef SETGET
VARIANT_ENUM_CAST(DetourNavigationMesh::partition_t);

class DetourNavigationMeshInstance : public Spatial {
	class DetourNavigationQueryData;
	GDCLASS(DetourNavigationMeshInstance, Spatial);
	Ref<DetourNavigationMesh> mesh;
	static void _bind_methods();
protected:
	unsigned int build_tiles(int x1, int y1, int x2, int y2);
	unsigned char *build_tile_mesh(int tx, int ty, const float* bmin, const float* bmax, int& dataSize, const Ref<Mesh>& mesh);
	void get_tile_bounding_box(int x, int z, Vector3& bmin, Vector3& bmax);
	static float random();
public:
	void set_navmesh(const Ref<DetourNavigationMesh> &mesh)
	{
		if (this->mesh != mesh)
			this->mesh = mesh;
	}
	Ref<DetourNavigationMesh> get_navmesh()
	{
		return mesh;
	}
	DetourNavigationMeshInstance();
	void add_meshdata(const Ref<Mesh> &p_mesh,
			const Transform &p_xform,
			Vector<float> &p_verticies,
			Vector<int> &p_indices);
	bool build_tile(int x, int z);
	void remove_tile(int x, int z);
	void build();
	void add_mesh(const Ref<Mesh> &mesh, const Transform &transform);
	void set_group(const String& group)
	{
		mesh->set_group(group);
	}
	const String& get_group() const
	{
		return mesh->get_group();
	}
	Vector<Ref<Mesh> > geometries;
	Vector<Transform> xforms;
	Vector<DetourNavigationArea> nav_areas;
	void collect_geometries(bool recursive);
};
#endif
