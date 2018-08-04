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
	void * navmesh_query;
	DetourNavigation() : navmesh_query(NULL)
	{
	}
};

class DetourNavigationMesh : public Resource {
	GDCLASS(DetourNavigationMesh, Resource);
	real_t cell_size;
	real_t cell_height;
	real_t agent_height;
	real_t agent_radius;
	real_t agent_max_climb;
	real_t agent_max_slope;
	real_t region_min_size;
	real_t region_merge_size;
	real_t edge_max_length;
	real_t edge_max_error;
	real_t detail_sample_distance;
	real_t detail_sample_max_error;
	int tile_size;
	Vector3 padding;
	int num_tiles_x;
	int num_tiles_z;
	int partition_type;
	void *navmesh;
	AABB bounding_box;
	String group;
	Vector<Ref<Mesh> > geometries;
protected:
	void get_tile_bounding_box(int x, int z, Vector3& bmin, Vector3& bmax);
	void collect_geometries(Node *root_node, bool recursive);
	void release_navmesh();
	unsigned char *build_tile_mesh(int tx, int ty, const float* bmin, const float* bmax, int& dataSize, const Ref<Mesh>& mesh);
	void add_meshdata(const Ref<Mesh> &p_mesh,
			const Transform &p_xform,
			Vector<float> &p_verticies,
			Vector<int> &p_indices);
	unsigned int build_tiles(int x1, int y1, int x2, int y2);
	bool build_tile(int x, int z);
public:
	void set_group(const String& group);
	void build();
	void add_mesh(const Ref<Mesh>& mesh);
	DetourNavigationMesh();
};
#endif
