#include <cstdio>
#include "road.h"

RoadElement::RoadElement(): MeshInstance(),
	up(0.0f, 1.0f, 0.0f),
	debug(NULL),
	center_magnet(3.0f),
	work_mesh(memnew(ArrayMesh))
{
}
RoadElement::~RoadElement()
{
	if (debug)
		memfree(debug);
}

struct curve_data {
	float curve_len;
	PoolVector<Vector3> points;
};

struct edge_set {
	int edges;
	int edge_size;
	PoolVector<Vector3> points;
	PoolVector<Vector3> normals;
	PoolVector<Vector2> uvs;
};

struct vshape {
	Vector3 points[3];
	int lanes;
	bool sidewalks;

	Vector3 dir1, dir2, t1, t2;
	PoolVector<struct curve_data> curves;
	PoolVector<struct edge_set> edgesets;
};

void RoadElement::set_debug(int flags)
{
	if (flags) {
		if (!debug) {
			debug = memnew(RoadElementDebug);
			add_child(debug);
		}
	} else {
		if (debug) {
			debug->queue_delete();
			debug = NULL;
		}
	}
	debug_flags = flags;
}
void RoadElement::add_triangle(const PoolVector<Vector3> &points,
		const PoolVector<Vector3> &normals,
		const PoolVector<Vector2> &uvs)
{
	int cur_index = indices.size();
	int i;
	for (i = 0; i < points.size(); i++) {
		vertices.push_back(points[i]);
		this->normals.push_back(normals[i]);
		uvdata.push_back(uvs[i]);
		indices.push_back(cur_index);
		cur_index++;
	}
}
void RoadElement::add_quad(const PoolVector<Vector3> &points,
		const PoolVector<Vector3> &normals,
		const PoolVector<Vector2> &uvs)
{
	PoolVector<Vector3> p1, p2;
	PoolVector<Vector3> n1, n2;
	PoolVector<Vector2> u1, u2;
	int indices1[] = {0, 1, 2};
	int indices2[] = {0, 2, 3};
	int i;
	for (i = 0; i < 3; i++) {
		p1.push_back(points[indices1[i]]);
		n1.push_back(normals[indices1[i]]);
		u1.push_back(uvs[indices1[i]]);
		p2.push_back(points[indices2[i]]);
		n2.push_back(normals[indices2[i]]);
		u2.push_back(uvs[indices2[i]]);
	}
	add_triangle(p1, n1, u1);
	add_triangle(p2, n2, u2);
}
void RoadElement::commit(Ref<Material> material)
{
	Array array;
	int id = work_mesh->get_surface_count();
	hide();
	array.resize(ArrayMesh::ARRAY_MAX);
	array[ArrayMesh::ARRAY_VERTEX] = vertices;
	array[ArrayMesh::ARRAY_NORMAL] = normals;
	array[ArrayMesh::ARRAY_TEX_UV] = uvdata;
	array[ArrayMesh::ARRAY_INDEX] = indices;
	work_mesh->add_surface_from_arrays(ArrayMesh::PRIMITIVE_TRIANGLES, array);
	work_mesh->surface_set_material(id, material);
	set_mesh(work_mesh);
	show();
}
void RoadElement::clear()
{
	hide();
	vertices.resize(0);
	normals.resize(0);
	uvdata.resize(0);
	indices.resize(0);
	work_mesh = Ref<ArrayMesh>(memnew(ArrayMesh));
	set_mesh(NULL);
	show();
}

void RoadElement::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("add_triangle", "points", "normals", "uvs"), &RoadElement::add_triangle);
	ClassDB::bind_method(D_METHOD("add_quad", "points", "normals", "uvs"), &RoadElement::add_quad);
	ClassDB::bind_method(D_METHOD("commit", "material"), &RoadElement::commit);
	ClassDB::bind_method(D_METHOD("clear"), &RoadElement::clear);
	ClassDB::bind_method(D_METHOD("set_debug", "flags"), &RoadElement::set_debug);
	ClassDB::bind_method(D_METHOD("triangulate_set", "eset"), &RoadElement::_triangulate_set);
	ClassDB::bind_method(D_METHOD("extrude_test", "params"), &RoadElement::_extrude);
	ClassDB::bind_method(D_METHOD("triangulate"), &RoadElement::triangulate);
	ClassDB::bind_method(D_METHOD("extrude_neighbors", "neighbors"), &RoadElement::extrude_neighbors);
	ClassDB::bind_method(D_METHOD("build", "neighbrs", "material",
				"lane_prof_data", "sidewalk_prof_data", "debug"), &RoadElement::build);
}

void RoadElementDebug::setup()
{
	Ref<SpatialMaterial> mat = memnew(SpatialMaterial);
	mat->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
	mat->set_flag(SpatialMaterial::FLAG_USE_POINT_SIZE, true);
	mat->set_flag(SpatialMaterial::FLAG_DISABLE_DEPTH_TEST, true);
	mat->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
	set_material_override(mat);
}

RoadElementDebug::RoadElementDebug():
	ImmediateGeometry(),
	element(NULL)
{
}

void RoadElementDebug::_notification(int p_what)
{
	switch(p_what) {
	case NOTIFICATION_ENTER_TREE:
		set_process(true);
		set_process_priority(30);
		setup();
		break;
	case NOTIFICATION_PROCESS:
		clear();
		draw_debug();
		break;
	case NOTIFICATION_EXIT_TREE:
		set_process(false);
		break;
	}
}
void RoadElementDebug::draw_debug()
{
	RoadElement *e = Object::cast_to<RoadElement>(get_parent());
	if (!e)
		return;
	if (e->debug_flags & 1) {
		int k;
		begin(ArrayMesh::PRIMITIVE_LINES);
		set_color(Color(1, 0, 0, 1));
		for (k = 0; k < e->vertices.size(); k++) {
			add_vertex(e->vertices[k] + Vector3(0, -0.5f, 0));
			add_vertex(e->vertices[k] + Vector3(0, 0.5f, 0));
		}
		end();
	}
	if (e->debug_flags & 2) {
		int k;
		begin(ArrayMesh::PRIMITIVE_LINES);
		set_color(Color(0, 1, 0, 1));
		for (k = 0; k < e->indices.size(); k += 3) {
			add_vertex(e->vertices[e->indices[k]] + Vector3(0, 0.1f, 0));
			add_vertex(e->vertices[e->indices[k + 1]] + Vector3(0, 0.1f, 0));
			add_vertex(e->vertices[e->indices[k + 1]] + Vector3(0, 0.1f, 0));
			add_vertex(e->vertices[e->indices[k + 2]] + Vector3(0, 0.1f, 0));
			add_vertex(e->vertices[e->indices[k + 2]] + Vector3(0, 0.1f, 0));
			add_vertex(e->vertices[e->indices[k]] + Vector3(0, 0.1f, 0));
		}
		end();
	}
}

void RoadElement::triangulate_set(int edges, int edge_size,
			const PoolVector<Vector3> &points,
			const PoolVector<Vector3> &normals,
			const PoolVector<Vector2> &uvs)
{
	int i, j;
	int idx = vertices.size();
	for (i = 0; i < edges; i++) {
		for (j = 0; j < edge_size; j++) {
			Vector3 pt = points[i * edge_size + j];
			Vector3 n = normals[i * edge_size + j];
			Vector2 uv = uvs[i * edge_size + j];
			this->vertices.push_back(pt);
			this->normals.push_back(n);
			this->uvdata.push_back(uv);
			if (i < edges - 1 && j < edge_size - 1) {
				indices.push_back(idx);
				indices.push_back(idx + 1);
				indices.push_back(idx + edge_size);
				indices.push_back(idx + 1);
				indices.push_back(idx + edge_size + 1);
				indices.push_back(idx + edge_size);
			}
			idx++;
		}
	}
}
void RoadElement::_triangulate_set(const Dictionary &eset)
{
	const Array &edges = eset["edges"];
	const Array &normals = eset["normals"];
	const Array &uvs = eset["edges_uv"];
	int nedges = edges.size();
	int edge_size = ((Array)edges[0]).size();
	int i, j;
	PoolVector<Vector3> point_data;
	PoolVector<Vector3> normal_data;
	PoolVector<Vector2> uv_data;
	for (i = 0; i < nedges; i++) {
		for (j = 0; j < edge_size; j++) {
			point_data.push_back(((Array)edges[i])[j]);
			normal_data.push_back(((Array)normals[i])[j]);
			uv_data.push_back(((Array)uvs[i])[j]);
		}
	}
	triangulate_set(nedges, edge_size, point_data, normal_data, uv_data);
}
float RoadElement::get_lane_width() const
{
	return lane_profile->length;
}
float RoadElement::get_sidewalk_width() const
{
	return sidewalk_profile->length;
}
void RoadElement::setup_mid(struct vshape *vshape)
{
	Curve3D curve;
	struct curve_data data;
	float offset = lane_profile->profile[0].y;
	curve.add_point(vshape->points[0] + Vector3(0, offset, 0));
	curve.add_point(vshape->points[1] + Vector3(0, offset, 0));
	curve.add_point(vshape->points[2] + Vector3(0, offset, 0));
	curve.set_bake_interval(2.0f);
	data.points = curve.get_baked_points();
	data.curve_len = curve.get_baked_length();
	vshape->curves.push_back(data);
}
Vector3 RoadElement::get_ptx(struct vshape *vshape, float offset)
{
	Vector2 pt1(vshape->points[0].x, vshape->points[0].z);
	Vector2 pt2(vshape->points[1].x, vshape->points[1].z);
	Vector2 pt3(vshape->points[2].x, vshape->points[2].z);
	Vector2 b1(vshape->t1.x, vshape->t1.z);
	Vector2 b2(vshape->t2.x, vshape->t2.z);
	Vector2 d1(vshape->dir1.x, vshape->dir1.z);
	Vector2 d2(vshape->dir2.x, vshape->dir2.z);
	Vector2 px;
	bool r = Geometry::line_intersects_line_2d(pt1 + b1 * offset, d1, pt3 + b2 * offset, -d2, px);
	if (!r)
		px = pt2 + b1 * offset;
	Vector3 ptx(px.x, vshape->points[1].y, px.y);
	return ptx;
}
void RoadElement::setup_edge(struct vshape *vshape)
{
	int lane_count = vshape->lanes;
	bool sidewalks = vshape->sidewalks;
	int lcount = lane_count, i, l;
	RoadProfile *prof;
	if (sidewalks)
		lcount++;
	for (i = 0; i < lcount; i++) {
		prof = (i < lane_count) ? lane_profile : sidewalk_profile;
		for (l = 0; l < prof->profile.size() - 1; l++) {
			struct curve_data data;
			Curve3D curve;
			int lane_index = i + 1;
			float blength1 = vshape->curves[vshape->curves.size() - 1].curve_len;
			Vector2 lp = prof->profile[l + 1];
			Vector3 ptx = get_ptx(vshape, prof->length * (lane_index - 1) + lp.x) + up * lp.y;
			curve.add_point(vshape->points[0]
					+ vshape->t1 * prof->length * (lane_index - 1)
					+ vshape->t1 * lp.x + up * lp.y);
			curve.add_point(ptx - vshape->dir1 * prof->length * lane_index);
			curve.set_point_out(1, vshape->dir1 * (prof->length + 1.0f));
			curve.add_point(ptx + vshape->dir2 * prof->length * lane_index);
			curve.set_point_out(2, -vshape->dir2 * (prof->length + 1.0f));
			curve.add_point(vshape->points[2]
					+ vshape->t2 * prof->length * (lane_index - 1)
					+ vshape->t2 * lp.x + up * lp.y);
			float blength2 = curve.get_baked_length();
			curve.set_bake_interval(2.0 * blength2 / blength1);
			data.curve_len = blength2;
			data.points = curve.get_baked_points();
			vshape->curves.push_back(data);
		}
	}
}
void RoadElement::split_large_edge(PoolVector<Vector3> *points, int count)
{
	if (points->size() == count)
		return;
	while (points->size() < count) {
		int d = -1, i;
		float dst = 0.0f;
		for (i = 0; i < points->size() - 1; i++) {
			PoolVector<Vector3>::Write w = points->write();
			Vector3 p0 = w[i];
			Vector3 p1 = w[i + 1];
			float xd = p0.distance_squared_to(p1);
			if (dst < xd) {
				d = i;
				dst = xd;
			}
		}
		if (d < 0)
			break;
		Vector3 p0 = points->get(d);
		Vector3 p1 = points->get(d + 1);
		Vector3 m = p0.linear_interpolate(p1, 0.5f);
		points->insert(d + 1, m);
	}
}
void RoadElement::match_curves(struct vshape *vshape)
{
	int sz = 0;
	int i, j;
	for (i = 0; i < vshape->curves.size(); i++)
		if (sz < vshape->curves[i].points.size())
			sz = vshape->curves[i].points.size();
	for (i = 0; i < vshape->curves.size(); i++) {
		if (sz > vshape->curves[i].points.size()) {
			split_large_edge(&vshape->curves.write()[i].points, sz);
		}
	}
}
void RoadElement::extrude(struct vshape *vshape)
{
	int i, j;
	Vector3 p1 = vshape->points[0];
	Vector3 p2 = vshape->points[1];
	Vector3 p3 = vshape->points[2];
	Vector3 dir1 = (p2 - p1).normalized();
	Vector3 dir2 = (p3 - p2).normalized();
	Vector3 t1 = -dir1.cross(up).normalized();
	Vector3 t2 = -dir2.cross(up).normalized();
	vshape->dir1 = dir1;
	vshape->dir2 = dir2;
	vshape->t1 = t1;
	vshape->t2 = t2;
	vshape->curves.resize(0);
	setup_mid(vshape);
	setup_edge(vshape);
	match_curves(vshape);
	for (i = 0; i < vshape->curves.size() - 1; i++) {
		struct edge_set eset;
		eset.edge_size = 2;
		eset.edges = 0;
		for (j = 0; j < vshape->curves[i].points.size(); j++) {
			eset.points.push_back(vshape->curves[i].points[j]);
			eset.points.push_back(vshape->curves[i + 1].points[j]);
			eset.normals.push_back(up);
			eset.normals.push_back(up);
			eset.uvs.push_back(Vector2(0.f, 0.1f * (float)j));
			eset.uvs.push_back(Vector2(1.f, 0.1f * (float)j));
			eset.edges++;
		}
		vshape->edgesets.push_back(eset);
	}
}

void RoadElement::triangulate()
{
	int i, j;
	struct vshape *vshape;
	for (List<struct vshape *>::Element *e = vshapes.front(); e; e = e->next()) {
		vshape = e->get();
		for (i = 0; i < vshape->edgesets.size(); i++) {
			const struct edge_set *es = &(vshape->edgesets.read()[i]);
			for (j = 0; j < es->points.size(); j++)
				if (es->points[j].length_squared() < center_magnet) {
					struct edge_set *esw = &(vshape->edgesets.write()[i]);
					esw->points.write()[j] = Vector3();
				}
			triangulate_set(es->edges, es->edge_size, es->points, es->normals, es->uvs);
		}
	}
}

void RoadElement::_extrude(const Dictionary &params)
{
	int i;
	struct vshape *vshape = memnew(struct vshape);
	for (i = 0; i < 3; i++)
		vshape->points[i] = ((Array)params["points"])[i];
	vshape->sidewalks = params["sidewalks"];
	vshape->lanes = params["lanes"];
	lane_profile = memnew(RoadProfile(params["lane_profile"]));
	sidewalk_profile = memnew(RoadProfile(params["sidewalk_profile"]));
	extrude(vshape);
	vshapes.push_back(vshape);
}
void RoadElement::build(const Array &neighbors, Ref<Material> material,
		const PoolVector<Vector2> &lane_prof_data,
		const PoolVector<Vector2> &sidewalk_prof_data, bool debug)
{
	lane_profile = memnew(RoadProfile(lane_prof_data));
	sidewalk_profile = memnew(RoadProfile(sidewalk_prof_data));
	extrude_neighbors(neighbors);
	triangulate();
	if (debug)
		set_debug(3);
	commit(material);
	create_trimesh_collision();
}

void RoadElement::extrude_neighbors(const Array &neighbors)
{
	struct neighbor {
		Vector3 point;
		bool sidewalks;
		int lanes;
	};
	struct comparator {
		bool operator()(const struct neighbor &a, const struct neighbor &b) const {
			Vector3 p1 = a.point;
			Vector3 p2 = b.point;
			Vector2 pt1(p1.x, p1.z);
			Vector2 pt2(p2.x, p2.z);
			return pt1.angle() < pt2.angle();
		}
	};
	Vector<struct neighbor> neighbors_sorted;
	int i;
	for (i = 0; i < neighbors.size(); i++) {
		struct neighbor n;
		n.point = ((Dictionary)neighbors[i])["point"];
		n.sidewalks = ((Dictionary)neighbors[i])["sidewalks"];
		n.lanes = ((Dictionary)neighbors[i])["lanes"];
		neighbors_sorted.push_back(n);
	}
	neighbors_sorted.sort_custom<struct comparator>();
	PoolVector<float> xdistance;
	xdistance.resize(neighbors_sorted.size());
	for (i = 0; i < neighbors_sorted.size(); i++)
		xdistance.write()[i] = 0.0f;
	for (i = 0; i < neighbors_sorted.size(); i++) {
		int i1 = i;
		int i2 = (i + 1) % neighbors_sorted.size();
		struct neighbor n1 = neighbors_sorted[i1];
		struct neighbor n2 = neighbors_sorted[i2];
		bool sidewalks = n1.sidewalks || n2.sidewalks;
		int lanes = MAX(n1.lanes, n2.lanes);
		float xd = (float)(lanes * 2 + 1) * lane_profile->length * 2.0f;
		if (sidewalks)
			xd += sidewalk_profile->length;
		xd = MAX(xd, xdistance[i1]);
		xd = MAX(xd, xdistance[i2]);
		if (xdistance[i1] < xd)
			xdistance.write()[i1] = xd;
		if (xdistance[i2] < xd)
			xdistance.write()[i2] = xd;
	}
	for (i = 0; i < neighbors_sorted.size(); i++) {
		int i1 = i;
		int i2 = (i + 1) % neighbors_sorted.size();
		struct neighbor n1 = neighbors_sorted[i1];
		struct neighbor n2 = neighbors_sorted[i2];
		struct vshape *vshape = memnew(struct vshape);
		bool sidewalks = n1.sidewalks || n2.sidewalks;
		int lanes = MAX(n1.lanes, n2.lanes);
		vshape->points[0] = n1.point.normalized() * xdistance[i1];
		vshape->points[1] = Vector3();
		vshape->points[2] = n2.point.normalized() * xdistance[i2];
		vshape->lanes = lanes;
		vshape->sidewalks = sidewalks;
		extrude(vshape);
		vshapes.push_back(vshape);
	}
}


