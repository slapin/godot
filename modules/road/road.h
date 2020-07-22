#ifndef ROAD_H
#define ROAD_H
#include <core/object.h>
#include <scene/main/node.h>
#include <scene/3d/mesh_instance.h>
#include <scene/3d/immediate_geometry.h>

class RoadElementDebug;
class OpenSimplexNoise;
class RandomNumberGenerator;
class RoadProfile {
public:
	PoolVector<Vector2> profile;
	float minx = HUGE_VAL, maxx = -HUGE_VAL, length;
	RoadProfile(const PoolVector<Vector2> &data)
	{
		int i;
		profile = data;
		for (i = 0; i < data.size(); i++) {
			if (minx > data[i].x)
				minx = data[i].x;
			if (maxx < data[i].x)
				maxx = data[i].x;
		}
		length = maxx - minx;
	}
};
struct vshape;
class RoadElement: public MeshInstance {
	GDCLASS(RoadElement, MeshInstance);
protected:
	List<struct vshape *> vshapes;
	Ref<ArrayMesh> work_mesh;
	PoolVector<Vector3> vertices;
	PoolVector<Vector3> normals;
	PoolVector<int> indices;
	PoolVector<Vector2> uvdata;
	Vector3 up;
	RoadProfile *lane_profile, *sidewalk_profile;
	RoadElementDebug *debug;
	unsigned int debug_flags;
	static void _bind_methods();
	void extrude(struct vshape *vshape);
	void setup_mid(struct vshape *vshape);
	void setup_edge(struct vshape *vshape);
	void match_curves(struct vshape *vshape);
	void split_large_edge(PoolVector<Vector3> *points, int count);
	float get_lane_width() const;
	float get_sidewalk_width() const;
	Vector3 get_ptx(struct vshape *vshape, float offset);
	float center_magnet;
public:
	void triangulate();
	void add_quad(const PoolVector<Vector3> &points,
			const PoolVector<Vector3> &normals,
			const PoolVector<Vector2> &uvs);
	void add_triangle(const PoolVector<Vector3> &points,
			const PoolVector<Vector3> &normals,
			const PoolVector<Vector2> &uvs);
	void commit(Ref<Material> material);
	void clear();
	void set_debug(int flags);
	void triangulate_set(int edges, int edge_size,
			const PoolVector<Vector3> &points,
			const PoolVector<Vector3> &normals,
			const PoolVector<Vector2> &uvs);
	void _triangulate_set(const Dictionary &eset);
	void _extrude(const Dictionary &params);
	void extrude_neighbors(const Array &neighbors);
	void build(const Array &neighbors, Ref<Material> material,
		const PoolVector<Vector2> &lane_prof_data,
		const PoolVector<Vector2> &sidewalk_prof_data, bool debug);
	RoadElement();
	~RoadElement();
	friend class RoadElementDebug;
};

class RoadElementDebug: public ImmediateGeometry {
	GDCLASS(RoadElementDebug, ImmediateGeometry);
protected:
	RoadElement *element;
	void _notification(int p_what);
	void setup();
	void draw_debug();
public:
	RoadElementDebug();
};

class VoronoiCity: public Spatial {
	GDCLASS(VoronoiCity, Spatial);
protected:
	OpenSimplexNoise *noise;
	RandomNumberGenerator *rnd;
	int seed;
public:
	VoronoiCity();
	~VoronoiCity();
};
#endif
