#include "detour.h"
#include "obstacle.h"
#include "scene/3d/mesh_instance.h"
#include <Recast.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <DetourNavMeshQuery.h>
#include <DetourTileCache.h>
#include <DetourTileCacheBuilder.h>

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
	print_line("Building");
	for (int i = 0; i < geometries.size(); i++)
		if (geometries[i].is_valid()) {
			AABB convbox = geometries[i]->get_aabb();
			convbox = xforms[i].xform(convbox);
			mesh->bounding_box.merge_with(convbox);
		}
	print_line("mesh bb: " + String(mesh->bounding_box));
	mesh->bounding_box.position -= mesh->padding;
	mesh->bounding_box.size += mesh->padding * 2.0;
	int gridH = 0, gridW = 0;
	float tile_edge_length = (float)mesh->tile_size * mesh->cell_size;
	Vector3 bmin = mesh->bounding_box.position;
	Vector3 bmax = mesh->bounding_box.position + mesh->bounding_box.size;
	rcCalcGridSize(&bmin.coord[0], &bmax.coord[0], mesh->cell_size, &gridW, &gridH);
	mesh->set_num_tiles(gridW, gridH);
	print_line(String() + "tiles x: " + itos(mesh->get_num_tiles_x()) + " tiles z: " + itos(mesh->get_num_tiles_z()));
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
#ifdef TILE_CACHE
	dtTileCacheParams tile_cache_params;
	memset(&tile_cache_params, 0, sizeof(tile_cache_params));
	rcVcopy(tile_cache_params.orig, &bmin.coord[0]);
	tile_cache_params.ch = mesh->cell_height;
	tile_cache_params.cs = mesh->cell_size;
	tile_cache_params.width = mesh->tile_size;
	tile_cache_params.height = mesh->tile_size;
	tile_cache_params.maxSimplificationError = mesh->edge_max_error;
	tile_cache_params.maxTiles = mesh->get_num_tiles_x() * mesh->get_num_tiles_z() * mesh->max_layers;
	tile_cache_params.maxObstacles = mesh->max_obstacles;
	tile_cache_params.walkableClimb = mesh->agent_max_climb;
	tile_cache_params.walkableHeight = mesh->agent_height;
	tile_cache_params.walkableRadius = mesh->agent_radius;
	if (!mesh->alloc_tile_cache())
		return;
	if (!mesh->init_tile_cache(&tile_cache_params))
		return;
#endif
	unsigned int result = build_tiles(0, 0, mesh->get_num_tiles_x() - 1, mesh->get_num_tiles_z() - 1);
#ifdef TILE_CACHE
	mesh->get_tile_cache()->update(0, mesh->get_navmesh());
#endif
	print_line(String() + "built tiles: " + itos(result));
	print_line("mesh final bb: " + String(mesh->bounding_box));
#ifdef TILE_CACHE
	for (int i = 0; i < obstacles.size(); i++) {
		DetourNavigationObstacle *obstacle = obstacles[i];
		unsigned int id = mesh->add_obstacle(obstacle->get_global_transform().origin, obstacle->get_radius(), obstacle->get_height());
		obstacle->id = id;
	}
#else
	if (debug_view && mesh.is_valid()) {
		print_line("rebuilding debug navmesh");
		mesh->clear_debug_mesh();
		Object::cast_to<MeshInstance>(debug_view)->set_mesh(mesh->get_debug_mesh());
	}
#endif
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
unsigned int DetourNavigationMeshInstance::build_tiles(int x1, int z1, int x2, int z2)
{
	unsigned ret = 0;
	for (int z = z1; z <= z2; z++) {
		for (int x = x1; x <= x2; x++) {
			if (build_tile(x, z))
				ret++;
		}
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
	bmax = bmin + Vector3(tile_edge_length, mesh->bounding_box.size.y, tile_edge_length);
	// print_line("tile bounding box: " +itos(x) + " " + itos(z) + ": " + String(bmin) + "/" + String(bmax));
	// print_line("mesh bounding box: " + String(mesh->bounding_box));
}
bool DetourNavigationMeshInstance::build_tile(int x, int z)
{
	Vector3 bmin, bmax;
	if (!mesh.is_valid())
		return false;
	get_tile_bounding_box(x, z, bmin, bmax);
	dtNavMesh *nav = mesh->get_navmesh();
#ifdef TILE_CACHE
	dtTileCache *tile_cache = mesh->get_tile_cache();
	tile_cache->removeTile(nav->getTileRefAt(x, z, 0), NULL, NULL);
#else
	nav->removeTile(nav->getTileRefAt(x, z, 0), NULL, NULL);
#endif
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
	Transform base = get_global_transform().inverse();
	for (int idx; idx < geometries.size(); idx++) {
		if (!geometries[idx].is_valid())
			continue;
		AABB mesh_aabb = geometries[idx]->get_aabb();
		Transform xform = base * xforms[idx];
		mesh_aabb = xform.xform(mesh_aabb);
		if (!mesh_aabb.intersects_inclusive(expbox) && !expbox.encloses(mesh_aabb)) {
			continue;
		}
		// Add offmesh
		// Add NavArea
		// Add PhysicsBodies?
		Ref<Mesh> mdata = geometries[idx];
		// FIXME
		add_meshdata(mdata, xform, points, indices);
	}
	// print_line(String() + "points: " + itos(points.size()) + " indices: " + itos(indices.size()) + " tile_size: " + itos(mesh->tile_size));
#if 0
	print_line("mesh points:");
	for (int k = 0; k < points.size(); k += 3)
		print_line("point: " + itos(k) + ": " + rtos(points[k]) + ", " + rtos(points[k + 1]) + ", " + rtos(points[k + 2]));
#endif
	if (points.size() == 0 || indices.size() == 0)
		/* Nothing to do */
		return true;
	rcHeightfield *heightfield = rcAllocHeightfield();
	if (!heightfield) {
		ERR_PRINT("Failed to allocate height field");
		return false;
	}
	rcContext *ctx = new rcContext(true);
	if (!rcCreateHeightfield(ctx, *heightfield, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)) {
		ERR_PRINT("Failed to create height field");
		return false;
	}
	int ntris = indices.size() / 3;
	unsigned char tri_areas[ntris];
	memset(tri_areas, 0, sizeof(tri_areas));
	rcMarkWalkableTriangles(ctx, cfg.walkableSlopeAngle, &points[0], points.size() / 3, &indices[0], ntris, tri_areas);
	rcRasterizeTriangles(ctx, &points[0], points.size() / 3, &indices[0], tri_areas, ntris, *heightfield, cfg.walkableClimb);
	rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *heightfield);


	rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *heightfield);
	rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *heightfield);

	rcCompactHeightfield *compact_heightfield = rcAllocCompactHeightfield();
	if (!compact_heightfield) {
		ERR_PRINT("Failed to allocate compact height field");
		return false;
	}
	if (!rcBuildCompactHeightfield(ctx, cfg.walkableHeight, cfg.walkableClimb, *heightfield,
				*compact_heightfield)) {
		ERR_PRINT("Could not build compact height field");
		return false;
	}
	if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *compact_heightfield)) {
		ERR_PRINT("Could not erode walkable area");
		return false;
	}

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
#ifdef TILE_CACHE
	rcHeightfieldLayerSet * heightfield_layer_set = rcAllocHeightfieldLayerSet();
	if (!heightfield_layer_set) {
		ERR_PRINT("Could not allocate height field layer set");
		return false;
	}
	if (!rcBuildHeightfieldLayers(ctx, *compact_heightfield, cfg.borderSize, cfg.walkableHeight,
				*heightfield_layer_set)) {
		ERR_PRINT("Could not build heightfield layers");
		return false;
	}
	for (int i = 0; i < heightfield_layer_set->nlayers; i++) {
		dtTileCacheLayerHeader header;
		header.magic = DT_TILECACHE_MAGIC;
		header.version = DT_TILECACHE_VERSION;
		header.tx = x;
		header.ty = z;
		header.tlayer = i;
		rcHeightfieldLayer* layer = &heightfield_layer_set->layers[i];
		rcVcopy(header.bmin, layer->bmin);
		rcVcopy(header.bmax, layer->bmax);
		header.width = (unsigned char)layer->width;
		header.height = (unsigned char)layer->height;
		header.minx = (unsigned char)layer->minx;
		header.maxx = (unsigned char)layer->maxx;
		header.miny = (unsigned char)layer->miny;
		header.maxy = (unsigned char)layer->maxy;
		header.hmin = (unsigned short)layer->hmin;
		header.hmax = (unsigned short)layer->hmax;
		unsigned char *tile_data;
		int tile_data_size;
		if (dtStatusFailed(dtBuildTileCacheLayer(mesh->get_tile_cache_compressor(), &header, layer->heights, layer->areas, layer->cons,
						&tile_data, &tile_data_size))) {
			ERR_PRINT("Failed to build tile cache layers");
			return false;
		}
		dtCompressedTileRef tileRef;
		int status = tile_cache->addTile(tile_data, tile_data_size, DT_COMPRESSEDTILE_FREE_DATA, &tileRef);
		if (dtStatusFailed((dtStatus)status)) {
			dtFree(tile_data);
			tile_data = NULL;
		}
                tile_cache->buildNavMeshTilesAt(x, z, nav);

	}
#else
	rcContourSet *contour_set = rcAllocContourSet();
	if (!contour_set)
		return false;
	print_line("allocated contour set");
	if (!rcBuildContours(ctx, *compact_heightfield, cfg.maxSimplificationError, cfg.maxEdgeLen,
				*contour_set))
		return false;
	print_line("created contour set");
	rcPolyMesh *poly_mesh = rcAllocPolyMesh();
	if (!poly_mesh)
		return false;
	print_line("allocated polymesh");
	if (!rcBuildPolyMesh(ctx, *contour_set, cfg.maxVertsPerPoly, *poly_mesh))
		return false;
	print_line("created polymesh");
	rcPolyMeshDetail *poly_mesh_detail = rcAllocPolyMeshDetail();
	if (!poly_mesh_detail)
		return false;
	print_line("allocated polymesh detail");
	if (!rcBuildPolyMeshDetail(ctx, *poly_mesh, *compact_heightfield, cfg.detailSampleDist,
				cfg.detailSampleMaxError, *poly_mesh_detail))
		return false;
	print_line("created polymesh detail");
	/* Assign area flags TODO: use nav area assignment here */
	for (int i = 0; i < poly_mesh->npolys; i++) {
		if (poly_mesh->areas[i] != RC_NULL_AREA)
			poly_mesh->flags[i] = 0x1;
	}
	print_line("created area flags");
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
	print_line("setup offmesh connections");

	if (!dtCreateNavMeshData(&params, &nav_data, &nav_data_size))
		return false;
	if (dtStatusFailed(mesh->get_navmesh()->addTile(nav_data, nav_data_size, DT_TILE_FREE_DATA, 0, NULL))) {
		dtFree(nav_data);
		return false;
	}
	print_line("created navmesh data");
#endif
	return true;
}
/* More complicated queries follow */

DetourNavigationMeshInstance::DetourNavigationMeshInstance() :
	Spatial(),
	mesh(0),
	debug_view(0)
{
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
void DetourNavigationMeshInstance::collect_geometries(bool recursive)
{
	if (!mesh.is_valid()) {
		print_line("No valid navmesh set, please set valid navmesh resource");
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
	print_line(String() + "node_queue size: " + itos(node_queue.size()));
	while (node_queue.size() > 0) {
		Node *groupNode = node_queue.front()->get();
		node_queue.pop_front();
		if (Object::cast_to<MeshInstance>(groupNode)) {
			MeshInstance *mi = Object::cast_to<MeshInstance>(groupNode);
			Ref<Mesh> mesh = mi->get_mesh();
			Transform xform = mi->get_global_transform();
			if (mesh.is_valid())
				add_mesh(mesh, xform);
#ifdef TILE_CACHE
		} else if (Object::cast_to<DetourNavigationObstacle>(groupNode)) {
			DetourNavigationObstacle *obstacle = Object::cast_to<DetourNavigationObstacle>(groupNode);
			obstacles.push_back(obstacle);
#endif
		} else if (Object::cast_to<DetourNavigationOffmeshConnection>(groupNode)) {
			DetourNavigationOffmeshConnection *offcon = Object::cast_to<DetourNavigationOffmeshConnection>(groupNode);
			Transform xform = offcon->get_global_transform();
			Transform base = get_global_transform().inverse();
			Vector3 start = (base * xform).xform(Vector3());
			Vector3 end = (base * xform).xform(offcon->end);
			mesh->add_offmesh_connection(start, end, offcon->radius, offcon->flags, offcon->area, offcon->bidirectional);
		}
		if (recursive)
			for (int i = 0; i < groupNode->get_child_count(); i++)
				node_queue.push_back(groupNode->get_child(i));
	}
	print_line(String() + "geometries size: " + itos(geometries.size()));
}
void DetourNavigationMeshInstance::add_mesh(const Ref<Mesh>& mesh, const Transform& xform)
{
	geometries.push_back(mesh);
	xforms.push_back(xform);
}
void DetourNavigationMeshInstance::_notification(int p_what) {

	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			if (get_tree()->is_debugging_navigation_hint()) {
				MeshInstance *dm = memnew(MeshInstance);
				if (mesh.is_valid())
					dm->set_mesh(mesh->get_debug_mesh());
				dm->set_material_override(get_tree()->get_debug_navigation_material());
				add_child(dm);
				debug_view = dm;
			}
#ifdef TILE_CACHE
			set_process(true);
#endif
		} break;
		case NOTIFICATION_EXIT_TREE: {
			if (debug_view) {
				debug_view->queue_delete();
				debug_view = NULL;
			}
#ifdef TILE_CACHE
			set_process(false);
#endif
		} break;
#ifdef TILE_CACHE
		case NOTIFICATION_PROCESS: {
			float delta = get_process_delta_time();
			dtTileCache *tile_cache = mesh->get_tile_cache();
			if (tile_cache) {
				tile_cache->update(delta, mesh->get_navmesh());
				if (debug_view)
					Object::cast_to<MeshInstance>(debug_view)->set_mesh(mesh->get_debug_mesh());
			}
	   	} break;
#endif
	}
}
void DetourNavigationMeshInstance::remove_tile(int x, int z)
{
	if (mesh.is_valid()) {
		mesh->get_navmesh();
	}
}
void DetourNavigationMeshInstance::set_navmesh(const Ref<DetourNavigationMesh> &mesh)
{
	if (this->mesh != mesh) {
		this->mesh = mesh;
		if (debug_view && this->mesh.is_valid())
			Object::cast_to<MeshInstance>(debug_view)->set_mesh(this->mesh->get_debug_mesh());
	}
}
void DetourNavigationMeshInstance::_bind_methods()
{
	/* Navmesh */
	ClassDB::bind_method(D_METHOD("build"), &DetourNavigationMeshInstance::build);
	ClassDB::bind_method(D_METHOD("collect_geometries", "recursive"), &DetourNavigationMeshInstance::collect_geometries);
	ClassDB::bind_method(D_METHOD("set_navmesh", "navmesh"), &DetourNavigationMeshInstance::set_navmesh);
	ClassDB::bind_method(D_METHOD("get_navmesh"), &DetourNavigationMeshInstance::get_navmesh);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "navmesh", PROPERTY_HINT_RESOURCE_TYPE, "DetourNavigationMesh"), "set_navmesh", "get_navmesh");
}
#undef SETGET
