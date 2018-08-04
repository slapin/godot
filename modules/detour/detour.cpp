#include "detour.h"
#include "scene/3d/mesh_instance.h"
#include <Recast.h>
#include <DetourNavMesh.h>

static const int DEFAULT_TILE_SIZE = 128;
static const float DEFAULT_CELL_SIZE = 0.3f;
static const float DEFAULT_CELL_HEIGHT = 0.2f;
static const float DEFAULT_AGENT_HEIGHT = 2.0f;
static const float DEFAULT_AGENT_RADIUS = 0.6f;
static const float DEFAULT_AGENT_MAX_CLIMB = 0.9f;
static const float DEFAULT_AGENT_MAX_SLOPE = 45.0f;
static const float DEFAULT_REGION_MIN_SIZE = 8.0f;
static const float DEFAULT_REGION_MERGE_SIZE = 20.0f;
static const float DEFAULT_EDGE_MAX_LENGTH = 12.0f;
static const float DEFAULT_EDGE_MAX_ERROR = 1.3f;
static const float DEFAULT_DETAIL_SAMPLE_DISTANCE = 6.0f;
static const float DEFAULT_DETAIL_SAMPLE_MAX_ERROR = 1.0f;

DetourNavigationMesh::DetourNavigationMesh() : Resource(), navmesh(NULL),
		cell_size(DEFAULT_CELL_SIZE),
		cell_height(DEFAULT_CELL_HEIGHT),
		agent_height(DEFAULT_AGENT_HEIGHT),
		agent_radius(DEFAULT_AGENT_RADIUS),
		agent_max_climb(DEFAULT_AGENT_MAX_CLIMB),
		agent_max_slope(DEFAULT_AGENT_MAX_SLOPE),
		region_min_size(DEFAULT_REGION_MIN_SIZE),
		region_merge_size(DEFAULT_REGION_MERGE_SIZE),
		edge_max_length(DEFAULT_EDGE_MAX_LENGTH),
		edge_max_error(DEFAULT_EDGE_MAX_ERROR),
		detail_sample_distance(DEFAULT_DETAIL_SAMPLE_DISTANCE),
		detail_sample_max_error(DEFAULT_DETAIL_SAMPLE_MAX_ERROR),
		tile_size(DEFAULT_TILE_SIZE)
{
	padding = Vector3(1.0f, 1.0f, 1.0f);
	bounding_box = AABB();
	group = "";
}
void DetourNavigationMesh::collect_geometries(Node *root_node, bool recursive)
{
	List<Node *> groupNodes;
	Set<Node *> processedNodes;
	if (!root_node)
		return;
	List<Node *> node_queue;
	geometries.clear();
	root_node->get_tree()->get_nodes_in_group(group, &groupNodes);
	for (const List<Node *>::Element *E = groupNodes.front(); E; E = E->next()) {
		Node *groupNode = E->get();
		node_queue.push_back(groupNode);
	}
	while (node_queue.size() > 0) {
		Node *groupNode = node_queue.front()->get();
		node_queue.pop_front();
		if (Object::cast_to<MeshInstance>(groupNode)) {
			MeshInstance *mi = Object::cast_to<MeshInstance>(groupNode);
			Ref<Mesh> mesh = mi->get_mesh();
			if (mesh.is_valid())
				add_mesh(mesh);
		}
		if (recursive)
			for (int i = 0; i < groupNode->get_child_count(); i++)
				node_queue.push_back(groupNode->get_child(i));
	}
}
void DetourNavigationMesh::add_mesh(const Ref<Mesh>& mesh)
{
	geometries.push_back(mesh);
}
inline unsigned int nextPow2(unsigned int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline unsigned int ilog2(unsigned int v)
{
	unsigned int r;
	unsigned int shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}

void DetourNavigationMesh::build()
{
	if (geometries.size() == 0)
		return;
	for (int i = 0; i < geometries.size(); i++)
		if (geometries[i].is_valid())
			bounding_box.merge(geometries[i]->get_aabb());
	bounding_box.position -= padding;
	bounding_box.size += padding * 2.0;
	int gridH = 0, gridW = 0;
	float tile_edge_length = (float)tile_size * cell_size;
	Vector3 bmin = bounding_box.position;
	Vector3 bmax = bounding_box.position + bounding_box.size;
	rcCalcGridSize(&bmin.coord[0], &bmax.coord[0], cell_size, &gridW, &gridH);
	num_tiles_x = (gridW + tile_size - 1) / tile_size;
	num_tiles_z = (gridH + tile_size - 1) / tile_size;
	unsigned int tile_bits = (unsigned int)ilog2(nextPow2(num_tiles_x * num_tiles_z));
	if (tile_bits > 14)
		tile_bits = 14;
	unsigned int poly_bits = 22 - tile_bits;
	unsigned int max_tiles = 1u << tile_bits;
	unsigned int max_polys = 1 << poly_bits;
        dtNavMeshParams params;
	rcVcopy(params.orig, &bmin.coord[0]);
	params.tileWidth = tile_edge_length;
	params.tileHeight = tile_edge_length;
	params.maxTiles = max_tiles;
	params.maxPolys = max_polys;
	navmesh = dtAllocNavMesh();
	if (!navmesh)
		return;
        if (dtStatusFailed(((dtNavMesh *)navmesh)->init(&params))) {
		release_navmesh();
		return;
	}
	unsigned int result = build_tiles(0, 0, num_tiles_x - 1, num_tiles_z - 1);
}
void DetourNavigationMesh::add_meshdata(const Ref<Mesh> &p_mesh, const Transform &p_xform, Vector<float> &p_verticies, Vector<int> &p_indices) {
	int current_vertex_count = 0;

	for (int i = 0; i < p_mesh->get_surface_count(); i++) {
		current_vertex_count = p_verticies.size() / 3;

		if (p_mesh->surface_get_primitive_type(i) != Mesh::PRIMITIVE_TRIANGLES)
			continue;

		int index_count = 0;
		if (p_mesh->surface_get_format(i) & Mesh::ARRAY_FORMAT_INDEX) {
			index_count = p_mesh->surface_get_array_index_len(i);
		} else {
			index_count = p_mesh->surface_get_array_len(i);
		}

		ERR_CONTINUE((index_count == 0 || (index_count % 3) != 0));

		int face_count = index_count / 3;

		Array a = p_mesh->surface_get_arrays(i);

		PoolVector<Vector3> mesh_vertices = a[Mesh::ARRAY_VERTEX];
		PoolVector<Vector3>::Read vr = mesh_vertices.read();

		if (p_mesh->surface_get_format(i) & Mesh::ARRAY_FORMAT_INDEX) {

			PoolVector<int> mesh_indices = a[Mesh::ARRAY_INDEX];
			PoolVector<int>::Read ir = mesh_indices.read();

			for (int i = 0; i < mesh_vertices.size(); i++) {
				Vector3 p_vec3 = p_xform.xform(vr[i]);
				p_verticies.push_back(p_vec3.x);
				p_verticies.push_back(p_vec3.y);
				p_verticies.push_back(p_vec3.z);
			}

			for (int i = 0; i < face_count; i++) {
				// CCW
				p_indices.push_back(current_vertex_count + (ir[i * 3 + 0]));
				p_indices.push_back(current_vertex_count + (ir[i * 3 + 2]));
				p_indices.push_back(current_vertex_count + (ir[i * 3 + 1]));
			}
		} else {
			face_count = mesh_vertices.size() / 3;
			for (int i = 0; i < face_count; i++) {
				Vector3 p_vec3 = p_xform.xform(vr[i * 3 + 0]);
				p_verticies.push_back(p_vec3.x);
				p_verticies.push_back(p_vec3.y);
				p_verticies.push_back(p_vec3.z);
				p_vec3 = p_xform.xform(vr[i * 3 + 2]);
				p_verticies.push_back(p_vec3.x);
				p_verticies.push_back(p_vec3.y);
				p_verticies.push_back(p_vec3.z);
				p_vec3 = p_xform.xform(vr[i * 3 + 1]);
				p_verticies.push_back(p_vec3.x);
				p_verticies.push_back(p_vec3.y);
				p_verticies.push_back(p_vec3.z);

				p_indices.push_back(current_vertex_count + (i * 3 + 0));
				p_indices.push_back(current_vertex_count + (i * 3 + 1));
				p_indices.push_back(current_vertex_count + (i * 3 + 2));
			}
		}
	}
}

unsigned char *DetourNavigationMesh::build_tile_mesh(int tx, int ty, const float* bmin, const float* bmax, int& dataSize, const Ref<Mesh>& mesh)
{
	Vector<float> verts;
	Vector<int> indices;
	Transform xform;
	add_meshdata(mesh, xform, verts, indices);
	int nverts = verts.size();
	int ntris = indices.size() / 3;
}
void DetourNavigationMesh::release_navmesh()
{
	dtFreeNavMesh((dtNavMesh*)navmesh);
	navmesh = NULL;
	num_tiles_x = 0;
	num_tiles_z = 0;
	bounding_box = AABB();
}

void DetourNavigationMesh::set_group(const String& group)
{
	this->group = group;
}

unsigned int DetourNavigationMesh::build_tiles(int x1, int z1, int x2, int z2)
{
	unsigned ret = 0;
	for (int z = z1; z <= z2; z++) {
		for (int x = x1; x <= x2; x++)
			if (build_tile(x, z))
				ret++;
	}
	return ret;
}
void DetourNavigationMesh::get_tile_bounding_box(int x, int z, Vector3& bmin, Vector3& bmax)
{
	const float tile_edge_length = (float)tile_size * cell_size;
	bmin = bounding_box.position +
		Vector3(tile_edge_length * (float)x,
				0,
			       	tile_edge_length * (float)z);
	bmax = bmin + Vector3(tile_edge_length, 0, tile_edge_length);
}
bool DetourNavigationMesh::build_tile(int x, int z)
{
	Vector3 bmin, bmax;
	get_tile_bounding_box(x, z, bmin, bmax);
	dtNavMesh *nav = (dtNavMesh *)navmesh;
	nav->removeTile(nav->getTileRefAt(x, z, 0), NULL, NULL);
	rcConfig cfg;
	cfg.cs = cell_size;
	cfg.ch = cell_height;
	cfg.walkableSlopeAngle = agent_max_slope;
	cfg.walkableHeight = (int)ceil(agent_height / cfg.ch);
	cfg.walkableClimb = (int)floor(agent_max_climb / cfg.ch);
	cfg.walkableRadius = (int)ceil(agent_radius / cfg.cs);
	cfg.maxEdgeLen = (int)(edge_max_length / cfg.cs);
	cfg.maxSimplificationError = edge_max_error;
	cfg.minRegionArea = (int)sqrtf(region_min_size);
	cfg.mergeRegionArea = (int)sqrtf(region_merge_size);
	cfg.maxVertsPerPoly = 6;
	cfg.tileSize = tile_size;
	cfg.borderSize = cfg.walkableRadius + 3;
	cfg.width = cfg.tileSize + cfg.borderSize * 2;
	cfg.height = cfg.tileSize + cfg.borderSize * 2;
	cfg.detailSampleDist = detail_sample_distance < 0.9f ? 0.0f : cell_size * detail_sample_distance;
	cfg.detailSampleMaxError = cell_height * detail_sample_max_error;
	rcVcopy(cfg.bmin, &bmin.coord[0]);
	rcVcopy(cfg.bmax, &bmax.coord[0]);
	cfg.bmin[0] -= cfg.borderSize * cfg.cs;
	cfg.bmin[2] -= cfg.borderSize * cfg.cs;
	cfg.bmax[0] += cfg.borderSize * cfg.cs;
	cfg.bmax[2] += cfg.borderSize * cfg.cs;

	return false;
}

