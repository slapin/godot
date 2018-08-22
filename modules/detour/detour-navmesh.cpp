#include "detour-navmesh.h"
#include <Recast.h>
#include <DetourTileCache.h>
#include <DetourTileCacheBuilder.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <fastlz.h>

static const int DEFAULT_TILE_SIZE = 64;
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

#ifdef TILE_CACHE
static const int TILECACHE_MAXLAYERS = 255;
static const int DEFAULT_MAX_OBSTACLES = 1024;
static const int DEFAULT_MAX_LAYERS = 16;
#endif

#ifdef TILE_CACHE
struct FastLZCompressor : public dtTileCacheCompressor
{
	virtual int maxCompressedSize(const int bufferSize)
	{
		return (int)(bufferSize* 1.05f);
	}

	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
							  unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}

	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
								unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};
struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	size_t capacity;
	size_t top;
	size_t high;

	LinearAllocator(const size_t cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}

	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const size_t cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}

	virtual void reset()
	{
		high = MAX(high, top);
		top = 0;
	}

	virtual void* alloc(const size_t size)
	{
		if (!buffer)
			return 0;
		if (top+size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}

	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

struct NavMeshProcess: public dtTileCacheMeshProcess {
	DetourNavigationMesh *nav;
	inline explicit NavMeshProcess(DetourNavigationMesh *mesh) :
		nav(mesh)
	{
	}
	virtual void process(struct dtNavMeshCreateParams* params, unsigned char* polyAreas, unsigned short* polyFlags)
	{
		/* Add proper flags and offmesh connections here */
		for (int i = 0; i < params->polyCount; i++) {
			if (polyAreas[i] != RC_NULL_AREA)
				polyFlags[i] = RC_WALKABLE_AREA;
		}
		params->offMeshConCount = nav->offmesh_radii.size();
		if (params->offMeshConCount > 0) {
			params->offMeshConVerts = reinterpret_cast<const float *>(&nav->offmesh_vertices[0]);
			params->offMeshConRad = &nav->offmesh_radii[0];
			params->offMeshConFlags = &nav->offmesh_flags[0];
			params->offMeshConAreas = &nav->offmesh_areas[0];
			params->offMeshConDir = &nav->offmesh_dir[0];
			print_line("added offmesh connection");
		} else {
			print_line("NO offmesh connection");
			params->offMeshConVerts = NULL;
			params->offMeshConRad = NULL;
			params->offMeshConFlags = NULL;
			params->offMeshConAreas = NULL;
			params->offMeshConDir = NULL;
		}
		nav->clear_debug_mesh();
		nav->get_debug_mesh();
	}
};

#endif

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
		tile_size(DEFAULT_TILE_SIZE),
#ifdef TILE_CACHE
		tile_cache(0),
		tile_cache_alloc(new LinearAllocator(64000)),
		tile_cache_compressor(new FastLZCompressor),
		mesh_process(new NavMeshProcess(this)),
		max_obstacles(DEFAULT_MAX_OBSTACLES),
		max_layers(DEFAULT_MAX_LAYERS),
#endif
		initialized(false),
		bounding_box(AABB()),
		padding(Vector3(1.0f, 1.0f, 1.0f)),
		group("")
{
}

bool DetourNavigationMesh::alloc_tile_cache()
{
	tile_cache = dtAllocTileCache();
	if (!tile_cache) {
		ERR_PRINT("Could not allocate tile cache");
		release_navmesh();
		return false;
	}
	return true;
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

unsigned int DetourNavigationMesh::add_obstacle(const Vector3 &pos, real_t radius, real_t height)
{
	/* Need to test how this works and why this needed at all */
	/* TODO implement navmesh changes queue */
//	while (tile_cache->isObstacleQueueFull())
//		tile_cache->update(1, navMesh_);
	dtObstacleRef ref = 0;
	if (dtStatusFailed(tile_cache->addObstacle(&pos.coord[0], radius, height, &ref))) {
		ERR_PRINT("can't add obstacle");
		return 0;
	}
	return (unsigned int)ref;
}
void DetourNavigationMesh::remove_obstacle(unsigned int id)
{
	/* Need to test how this works and why this needed at all */
	/* TODO implement navmesh changes queue */
//	while (tile_cache->isObstacleQueueFull())
//		tile_cache->update(1, navMesh_);
	if (dtStatusFailed(tile_cache->removeObstacle(id)))
		ERR_PRINT("failed to remove obstacle");
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
	initialized = true;
	return true;
}
#ifdef TILE_CACHE
bool DetourNavigationMesh::init_tile_cache(dtTileCacheParams *params)
{
	if (dtStatusFailed(tile_cache->init(params, tile_cache_alloc, tile_cache_compressor, mesh_process))) {
		ERR_PRINT("Could not initialize tile cache");
		release_navmesh();
		return false;
	}
	return true;
}
#endif
Ref<ArrayMesh> DetourNavigationMesh::get_debug_mesh()
{
	if (debug_mesh.is_valid())
		return debug_mesh;
	if (!navmesh)
		return debug_mesh;
	print_line("building debug navmesh");
	List<Vector3> lines;
	const dtNavMesh *navm = navmesh;
	for (int i = 0; i < navm->getMaxTiles(); i++) {
		const dtMeshTile *tile = navm->getTile(i);
		if (!tile || !tile->header)
			continue;
		for (int j = 0; j <  tile->header->polyCount; j++) {
			dtPoly* poly = tile->polys + j;
			if (poly->getType() != DT_POLYTYPE_OFFMESH_CONNECTION) {
				for (int k = 0; k < poly->vertCount; k++) {
					lines.push_back(*reinterpret_cast<const Vector3*>(&tile->verts[poly->verts[k] * 3]));
					lines.push_back(*reinterpret_cast<const Vector3*>(&tile->verts[poly->verts[(k + 1) % poly->vertCount] * 3]));
				}
			} else if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) {
				const dtOffMeshConnection* con = &tile->offMeshCons[j - tile->header->offMeshBase];
				const float* va = &tile->verts[poly->verts[0]*3];
				const float* vb = &tile->verts[poly->verts[1]*3];
				bool startSet = false;
				bool endSet = false;
				for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next) {
					if (tile->links[k].edge == 0)
						startSet = true;
					if (tile->links[k].edge == 1)
						endSet = true;
				}
				Vector3 p0 = *reinterpret_cast<const Vector3 *>(va);
				Vector3 p1 = *reinterpret_cast<const Vector3 *>(&con[0]);
				Vector3 p2 = *reinterpret_cast<const Vector3 *>(&con[3]);
				Vector3 p3 = *reinterpret_cast<const Vector3 *>(vb);
				lines.push_back(p0);
				lines.push_back(p1);
				lines.push_back(p1);
				lines.push_back(p2);
				lines.push_back(p2);
				lines.push_back(p3);
				print_line("debug offmesh connection");
			}
		}
	}
	print_line("debug mesh lines: " + itos(lines.size()));

	PoolVector<Vector3> varr;
	varr.resize(lines.size());
	PoolVector<Vector3>::Write w = varr.write();
	int idx = 0;
	for (List<Vector3>::Element *E = lines.front(); E; E = E->next()) {
		w[idx++] = E->get();
	}

	debug_mesh = Ref<ArrayMesh>(memnew(ArrayMesh));

	Array arr;
	arr.resize(Mesh::ARRAY_MAX);
	arr[Mesh::ARRAY_VERTEX] = varr;

	debug_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, arr);

	return debug_mesh;
}

void DetourNavigationMesh::set_data(const Dictionary &p_value)
{
        dtNavMeshParams params;
	Vector3 orig = p_value["orig"];
	rcVcopy(params.orig, &orig.coord[0]);
	params.tileWidth = p_value["tile_edge_length"];
	params.tileHeight = p_value["tile_edge_length"];
	params.maxTiles = p_value["max_tiles"];
	params.maxPolys = p_value["max_polys"];
	if (navmesh) {
		if (initialized)
			dtFreeNavMesh(navmesh);
		else
			dtFree(navmesh);
		navmesh = NULL;
	}
	if (!alloc())
		return;
	if (!init(&params))
		return;
}

Dictionary DetourNavigationMesh::get_data()
{
	Dictionary t;
#if 0
	t["initialized"] = initialized;
	if (!initialized) {
		return t;
	}
        const dtNavMeshParams *params = navmesh->getParams();
	Vector3 orig;
	rcVcopy(&orig.coord[0], params->orig);
	t["orig"] = orig;
	t["tile_edge_length"] = params->tileWidth;
	t["max_tiles"] = params->maxTiles;
	t["max_polys"] = params->maxPolys;
	PoolVector<uint8_t> data;
	PoolVector<uint8_t>::Write data_w = data.write();
	const dtNavMesh *nm = navmesh;
	int pos = 0;
	for (int z = 0; z < num_tiles_z; z++)
		for (int x = 0; x < num_tiles_x; x++) {
			const dtMeshTile* tile = nm->getTileAt(x, z, 0);
			if (!tile)
				continue;
			if (pos >= data.size())
				data.resize(data.size() + sizeof(int) * 2 + sizeof(unsigned int) * 2 + tile->dataSize);
			memcpy(&data_w[pos], &x, sizeof(x));
			pos += sizeof(x);
			memcpy(&data_w[pos], &z, sizeof(x));
			pos += sizeof(z);
			uint32_t tile_ref = (uint32_t)nm->getTileRef(tile);
			memcpy(&data_w[pos], &tile_ref, sizeof(tile_ref));
			pos += sizeof(tile_ref);
			uint32_t data_size = (uint32_t)tile->dataSize;
			memcpy(&data_w[pos], &data_size, sizeof(data_size));
			pos += sizeof(data_size);
			memcpy(&data_w[pos], tile->data, data_size);
			pos += data_size;
		}
	print_line("submitted: " + itos(data.size()));
	t["data"] = data;
#endif
	return t;
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
	SETGET(tile_size, INT);
	BIND_ENUM_CONSTANT(PARTITION_WATERSHED);
	BIND_ENUM_CONSTANT(PARTITION_MONOTONE);
	ClassDB::bind_method(D_METHOD("set_partition_type", "type"), &DetourNavigationMesh::set_partition_type);
	ClassDB::bind_method(D_METHOD("get_partition_type"), &DetourNavigationMesh::get_partition_type);
	ClassDB::bind_method(D_METHOD("set_data", "data"), &DetourNavigationMesh::set_data);
	ClassDB::bind_method(D_METHOD("get_data"), &DetourNavigationMesh::get_data);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "partition_type", PROPERTY_HINT_ENUM, "watershed,monotone"), "set_partition_type", "get_partition_type");
	ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "data", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_STORAGE), "set_data", "get_data");
}
#undef SETGET

