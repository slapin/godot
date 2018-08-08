#include "detour.h"
#include "scene/3d/mesh_instance.h"
#include <Recast.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>

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
void DetourNavigationMeshInstance::collect_geometries(bool recursive)
{
	if (!mesh.is_valid()) {
		ERR_PRINT("No valid navmesh set, please set valid navmesh resource");
		return;
	}
	List<Node *> groupNodes;
	Set<Node *> processedNodes;
	List<Node *> node_queue;
	geometries.clear();
	get_tree()->get_nodes_in_group(mesh->get_group(), &groupNodes);
	for (const List<Node *>::Element *E = groupNodes.front(); E; E = E->next()) {
		Node *groupNode = E->get();
		node_queue.push_back(groupNode);
	}
	ERR_PRINTS(String() + "node_queue size: " + itos(node_queue.size()));
	while (node_queue.size() > 0) {
		Node *groupNode = node_queue.front()->get();
		node_queue.pop_front();
		if (Object::cast_to<MeshInstance>(groupNode)) {
			MeshInstance *mi = Object::cast_to<MeshInstance>(groupNode);
			Ref<Mesh> mesh = mi->get_mesh();
			Transform xform = mi->get_global_transform();
			if (mesh.is_valid())
				add_mesh(mesh, xform);
		}
		if (recursive)
			for (int i = 0; i < groupNode->get_child_count(); i++)
				node_queue.push_back(groupNode->get_child(i));
	}
	ERR_PRINTS(String() + "geometries size: " + itos(geometries.size()));
}
void DetourNavigationMeshInstance::add_mesh(const Ref<Mesh>& mesh, const Transform& xform)
{
	geometries.push_back(mesh);
	xforms.push_back(xform);
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

void DetourNavigationMeshInstance::build()
{
	if (geometries.size() == 0)
		return;
	if (!mesh.is_valid())
		return;
	ERR_PRINT("Building");
	for (int i = 0; i < geometries.size(); i++)
		if (geometries[i].is_valid())
			mesh->bounding_box.merge(geometries[i]->get_aabb());
	mesh->bounding_box.position -= mesh->padding;
	mesh->bounding_box.size += mesh->padding * 2.0;
	int gridH = 0, gridW = 0;
	float tile_edge_length = (float)mesh->tile_size * mesh->cell_size;
	Vector3 bmin = mesh->bounding_box.position;
	Vector3 bmax = mesh->bounding_box.position + mesh->bounding_box.size;
	rcCalcGridSize(&bmin.coord[0], &bmax.coord[0], mesh->cell_size, &gridW, &gridH);
	mesh->set_num_tiles(gridW, gridH);
	ERR_PRINTS(String() + "tiles x: " + itos(mesh->get_num_tiles_x()) + " tiles z: " + itos(mesh->get_num_tiles_z()));
	unsigned int tile_bits = (unsigned int)ilog2(nextPow2(mesh->get_num_tiles_x() * mesh->get_num_tiles_z()));
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
	if (!mesh->alloc())
		return;
	if (!mesh->init(&params))
		return;
	unsigned int result = build_tiles(0, 0, mesh->get_num_tiles_x() - 1, mesh->get_num_tiles_z() - 1);
	ERR_PRINTS(String() + "built tiles: " + itos(result));
}
void DetourNavigationMeshInstance::add_meshdata(const Ref<Mesh> &p_mesh, const Transform &p_xform, Vector<float> &p_verticies, Vector<int> &p_indices) {
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

unsigned char *DetourNavigationMeshInstance::build_tile_mesh(int tx, int ty, const float* bmin, const float* bmax, int& dataSize, const Ref<Mesh>& mesh)
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

unsigned int DetourNavigationMeshInstance::build_tiles(int x1, int z1, int x2, int z2)
{
	unsigned ret = 0;
	for (int z = z1; z <= z2; z++) {
		for (int x = x1; x <= x2; x++)
			if (build_tile(x, z))
				ret++;
	}
	return ret;
}
void DetourNavigationMeshInstance::get_tile_bounding_box(int x, int z, Vector3& bmin, Vector3& bmax)
{
	if (!mesh.is_valid())
		return;
	const float tile_edge_length = (float)mesh->tile_size * mesh->cell_size;
	bmin = mesh->bounding_box.position +
		Vector3(tile_edge_length * (float)x,
				0,
			       	tile_edge_length * (float)z);
	bmax = bmin + Vector3(tile_edge_length, 0, tile_edge_length);
}
bool DetourNavigationMeshInstance::build_tile(int x, int z)
{
	Vector3 bmin, bmax;
	if (!mesh.is_valid())
		return false;
	get_tile_bounding_box(x, z, bmin, bmax);
	dtNavMesh *nav = mesh->get_navmesh();
	nav->removeTile(nav->getTileRefAt(x, z, 0), NULL, NULL);
	rcConfig cfg;
	cfg.cs = mesh->cell_size;
	cfg.ch = mesh->cell_height;
	cfg.walkableSlopeAngle = mesh->agent_max_slope;
	cfg.walkableHeight = (int)ceil(mesh->agent_height / cfg.ch);
	cfg.walkableClimb = (int)floor(mesh->agent_max_climb / cfg.ch);
	cfg.walkableRadius = (int)ceil(mesh->agent_radius / cfg.cs);
	cfg.maxEdgeLen = (int)(mesh->edge_max_length / cfg.cs);
	cfg.maxSimplificationError = mesh->edge_max_error;
	cfg.minRegionArea = (int)sqrtf(mesh->region_min_size);
	cfg.mergeRegionArea = (int)sqrtf(mesh->region_merge_size);
	cfg.maxVertsPerPoly = 6;
	cfg.tileSize = mesh->tile_size;
	cfg.borderSize = cfg.walkableRadius + 3;
	cfg.width = cfg.tileSize + cfg.borderSize * 2;
	cfg.height = cfg.tileSize + cfg.borderSize * 2;
	cfg.detailSampleDist = mesh->detail_sample_distance < 0.9f ? 0.0f : mesh->cell_size * mesh->detail_sample_distance;
	cfg.detailSampleMaxError = mesh->cell_height * mesh->detail_sample_max_error;
	rcVcopy(cfg.bmin, &bmin.coord[0]);
	rcVcopy(cfg.bmax, &bmax.coord[0]);
	cfg.bmin[0] -= cfg.borderSize * cfg.cs;
	cfg.bmin[2] -= cfg.borderSize * cfg.cs;
	cfg.bmax[0] += cfg.borderSize * cfg.cs;
	cfg.bmax[2] += cfg.borderSize * cfg.cs;

	AABB expbox(bmin, bmax - bmin);
	expbox.position.x -= cfg.borderSize * cfg.cs;
	expbox.position.z -= cfg.borderSize * cfg.cs;
	expbox.size.x += 2.0 * cfg.borderSize * cfg.cs;
	expbox.size.z += 2.0 * cfg.borderSize * cfg.cs;
	Vector<float> points;
	Vector<int> indices;
	for (int idx; idx < geometries.size(); idx++) {
		if (!geometries[idx].is_valid())
			continue;
		ERR_PRINT("valid geometry");
		if (!geometries[idx]->get_aabb().intersects(expbox))
			continue;
		// Add offmesh
		// Add NavArea
		// Add PhysicsBodies?
		Ref<Mesh> mdata = geometries[idx];
		// FIXME
		Transform xform = xforms[idx];
		add_meshdata(mdata, xform, points, indices);
	}
	ERR_PRINTS(String() + "points: " + itos(points.size()) + " indices: " + itos(indices.size()));
	if (points.size() == 0 || indices.size() == 0)
		/* Nothing to do */
		return true;
	rcHeightfield *heightfield = rcAllocHeightfield();
	if (!heightfield)
		return false;
	rcContext *ctx = new rcContext(true);
	if (!rcCreateHeightfield(ctx, *heightfield, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch))
		return false;
	int ntris = indices.size() / 3;
	unsigned char tri_areas[ntris];
	memset(tri_areas, 0, sizeof(tri_areas));
	rcMarkWalkableTriangles(ctx, cfg.walkableSlopeAngle, &points[0], points.size() / 3, &indices[0], ntris, tri_areas);
	rcRasterizeTriangles(ctx, &points[0], points.size() / 3, &indices[0], tri_areas, ntris, *heightfield, cfg.walkableClimb);
	rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *heightfield);
	rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *heightfield);
	rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *heightfield);
	rcCompactHeightfield *compact_heightfield = rcAllocCompactHeightfield();
	if (!compact_heightfield)
		return false;
	if (!rcBuildCompactHeightfield(ctx, cfg.walkableHeight, cfg.walkableClimb, *heightfield,
				*compact_heightfield))
		return false;
	if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *compact_heightfield))
		return false;

	for (unsigned int i = 0; i < nav_areas.size(); i++) {
		Vector3 amin = nav_areas[i].bounds.position;
		Vector3 amax = amin + nav_areas[i].bounds.size;
		int id = nav_areas[i].id;
		rcMarkBoxArea(ctx, &amin.coord[0], &amax.coord[0],
			id, *compact_heightfield);
	}
	if (mesh->partition_type == DetourNavigationMesh::PARTITION_WATERSHED) {
		if (!rcBuildDistanceField(ctx, *compact_heightfield))
			return false;
		if (!rcBuildRegions(ctx, *compact_heightfield, cfg.borderSize, cfg.minRegionArea,
					cfg.mergeRegionArea))
			return false;
	} else
		if (!rcBuildRegionsMonotone(ctx, *compact_heightfield, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea))
			return false;
	rcContourSet *contour_set = rcAllocContourSet();
	if (!contour_set)
		return false;
	if (!rcBuildContours(ctx, *compact_heightfield, cfg.maxSimplificationError, cfg.maxEdgeLen,
				*contour_set))
		return false;
	rcPolyMesh *poly_mesh = rcAllocPolyMesh();
	if (!poly_mesh)
		return false;
	if (!rcBuildPolyMesh(ctx, *contour_set, cfg.maxVertsPerPoly, *poly_mesh))
		return false;
	rcPolyMeshDetail *poly_mesh_detail = rcAllocPolyMeshDetail();
	if (!poly_mesh_detail)
		return false;
	if (!rcBuildPolyMeshDetail(ctx, *poly_mesh, *compact_heightfield, cfg.detailSampleDist,
				cfg.detailSampleMaxError, *poly_mesh_detail))
		return false;
	/* Assign area flags TODO: use nav area assignment here */
	for (int i = 0; i < poly_mesh->npolys; i++) {
		if (poly_mesh->areas[i] != RC_NULL_AREA)
			poly_mesh->flags[i] = 0x1;
	}
	unsigned char *nav_data = NULL;
	int nav_data_size = 0;
	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof params);
	params.verts = poly_mesh->verts;
	params.vertCount = poly_mesh->nverts;
	params.polys = poly_mesh->polys;
	params.polyAreas = poly_mesh->areas;
	params.polyFlags = poly_mesh->flags;
	params.polyCount = poly_mesh->npolys;
	params.nvp = poly_mesh->nvp;
	params.detailMeshes = poly_mesh_detail->meshes;
	params.detailVerts = poly_mesh_detail->verts;
	params.detailVertsCount = poly_mesh_detail->nverts;
	params.detailTris = poly_mesh_detail->tris;
	params.detailTriCount = poly_mesh_detail->ntris;
	params.walkableHeight = mesh->agent_height;
	params.walkableRadius = mesh->agent_radius;
	params.walkableClimb = mesh->agent_max_climb;
	params.tileX = x;
	params.tileY = z;
	rcVcopy(params.bmin, poly_mesh->bmin);
	rcVcopy(params.bmax, poly_mesh->bmax);
	params.cs = cfg.cs;
	params.ch = cfg.ch;
	params.buildBvTree = true;
#if 0
	// building offmesh conections
    if (build.offMeshRadii_.Size())
    {
        params.offMeshConCount = build.offMeshRadii_.Size();
        params.offMeshConVerts = &build.offMeshVertices_[0].x_;
        params.offMeshConRad = &build.offMeshRadii_[0];
        params.offMeshConFlags = &build.offMeshFlags_[0];
        params.offMeshConAreas = &build.offMeshAreas_[0];
        params.offMeshConDir = &build.offMeshDir_[0];
    }
#endif

	if (!dtCreateNavMeshData(&params, &nav_data, &nav_data_size))
		return false;
	if (dtStatusFailed(mesh->get_navmesh()->addTile(nav_data, nav_data_size, DT_TILE_FREE_DATA, 0, NULL))) {
		dtFree(nav_data);
		return false;
	}
	return true;
}
bool DetourNavigationMesh::alloc()
{
	navmesh = dtAllocNavMesh();
	return navmesh ? true : false;
}
bool DetourNavigationMesh::init(dtNavMeshParams *params)
{
	if (dtStatusFailed((navmesh)->init(params))) {
		release_navmesh();
		return false;
	}
	return true;
}

void DetourNavigation::_bind_methods()
{
}
void DetourNavigationArea::_bind_methods()
{
}
void DetourNavigationOffmeshConnection::_bind_methods()
{
}
#define SETGET(v, t) \
	ClassDB::bind_method(D_METHOD("set_" #v, #v), &DetourNavigationMesh::set_##v); \
	ClassDB::bind_method(D_METHOD("get_" #v), &DetourNavigationMesh::get_##v); \
	ADD_PROPERTY(PropertyInfo(Variant:: t, #v), "set_" #v, "get_" #v)

void DetourNavigationMesh::_bind_methods()
{
	SETGET(cell_size, REAL);
	SETGET(cell_height, REAL);
	SETGET(agent_height, REAL);
	SETGET(agent_radius, REAL);
	SETGET(agent_max_climb, REAL);
	SETGET(agent_max_slope, REAL);
	SETGET(region_min_size, REAL);
	SETGET(region_merge_size, REAL);
	SETGET(edge_max_length, REAL);
	SETGET(edge_max_error, REAL);
	SETGET(detail_sample_distance, REAL);
	SETGET(detail_sample_max_error, REAL);
	SETGET(group, STRING);
	SETGET(padding, VECTOR3);
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "cell_size"), "set_cell_size", "get_cell_size");
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "cell_height"), "set_cell_height", "get_cell_height");
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "agent_height"), "set_agent_height", "get_agent_height");
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "agent_radius"), "set_agent_radius", "get_agent_radius");
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "agent_max_climb"), "set_agent_max_climb", "get_agent_max_climb");
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "agent_max_slope"), "set_agent_max_slope", "get_agent_max_slope");
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "region_min_size"), "set_region_min_size", "get_region_min_size");
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "region_merge_size"), "set_region_merge_size", "get_region_merge_size");
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "edge_max_length"), "set_edge_max_length", "get_edge_max_length");
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "edge_max_error"), "set_edge_max_error", "get_edge_max_error");
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "detail_sample_distance"), "set_detail_sample_distance", "get_detail_sample_distance");
	// ADD_PROPERTY(PropertyInfo(Variant::REAL, "detail_sample_max_error"), "set_detail_sample_max_error", "get_detail_sample_max_error");
	// ADD_PROPERTY(PropertyInfo(Variant::STRING, "group"), "set_group", "get_group");
}
void DetourNavigationMeshInstance::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("build"), &DetourNavigationMeshInstance::build);
	ClassDB::bind_method(D_METHOD("collect_geometries", "recursive"), &DetourNavigationMeshInstance::collect_geometries);
	ClassDB::bind_method(D_METHOD("set_navmesh", "navmesh"), &DetourNavigationMeshInstance::set_navmesh);
	ClassDB::bind_method(D_METHOD("get_navmesh"), &DetourNavigationMeshInstance::get_navmesh);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "navmesh", PROPERTY_HINT_RESOURCE_TYPE, "DetourNavigationMesh"), "set_navmesh", "get_navmesh");
}
#undef SETGET
