#include <core/math/random_number_generator.h>
#include "road.h"

class Voronoi {
	class Triangle {
	public:
		Vector2 p1, p2, p3, center;
		float radius;
		bool has_edge(const Vector2 &a, const Vector2 &b);
		Triangle(const Vector2 &pt1,
				const Vector2 &pt2,
				const Vector2 &pt3);
		Triangle();
	protected:
		float get_signed_area(const Vector2 &pt1,
				const Vector2 &pt2,
				const Vector2 &pt3);
	};
	class Region {
		List<Triangle> vertices;
		Vector2 _seed;
		Vector2 center();
	};
	List<Triangle> triangles;
	Vector<Vector2> points;
	Vector<Vector2> frame;
	bool regions_dirty;
	HashMap<Vector2, Region> regions;
	void build_region(const Vector2 &p);
	HashMap<Vector2, Region> get_regions();
	List<Region> part();
public:
	void triangulate();
	static Vector<Vector2> relax(Voronoi *v, Rect2 bounds);
	void add_point(const Vector2 &p);
	Voronoi(const Rect2 &bounds, const Vector<Vector2> &add_points);
};

Voronoi::Triangle::Triangle(const Vector2 &pt1,
				const Vector2 &pt2,
				const Vector2 &pt3)
{
	float s = get_signed_area(pt1, pt2, pt3);
	if (!(fabs(s) > 0))
		return;
	p1 = pt1;
	p2 = (s > 0.0f) ? pt2 : pt3;
	p3 = (s > 0.0f) ? pt3 : pt2;
	Vector2 m1 = (p1 + p2) / 2.0f;
	Vector2 m2 = (p2 + p3) / 2.0f;
	Vector2 t1 = (m1 - p1).tangent();
	Vector2 t2 = (m2 - p1).tangent();
	if (!Geometry::line_intersects_line_2d(m1, t1, m2, -t2, center))
		return;
	radius = center.distance_to(p1);
}
Voronoi::Triangle::Triangle()
{
}
float Voronoi::Triangle::get_signed_area(const Vector2 &pt1,
		const Vector2 &pt2,
		const Vector2 &pt3)
{
	return (pt2.x - pt1.x) * (pt2.y + pt1.y) +
		(pt3.x - pt2.x) * (pt3.y + pt2.y) +
		(pt1.x - pt3.x) * (pt1.y + pt3.y);
}

Voronoi::Voronoi(const Rect2 &bounds, const Vector<Vector2> &add_points)
{
	Vector2 c1, c2, c3, c4;
	c1 = bounds.position;
	c2 = bounds.position + Vector2(0, bounds.size.y);
	c3 = bounds.position + Vector2(bounds.size.x, 0);
	c4 = bounds.position + Vector2(bounds.size.x, bounds.size.y);
	points.push_back(c1);
	points.push_back(c2);
	points.push_back(c3);
	points.push_back(c4);
	frame.push_back(c1);
	frame.push_back(c2);
	frame.push_back(c3);
	frame.push_back(c4);
	points.append_array(add_points);
}
void Voronoi::triangulate()
{
	int i;
	triangles.clear();
	Vector<int> tris = Geometry::triangulate_delaunay_2d(points);
	for (i = 0; i < tris.size(); i += 3) {
		Vector2 p1 = points[tris[i]];
		Vector2 p2 = points[tris[i + 1]];
		Vector2 p3 = points[tris[i + 2]];
		triangles.push_back(Triangle(p1, p2, p3));
	}
	regions_dirty = true;
}
Vector<Vector2> Voronoi::relax(Voronoi *v, Rect2 bounds)
{
	int i;
	List<Region> regions = v->part();
	Vector<Vector2> points(v->points);
	for (i = 0; i < v->frame.size(); i++)
		if (points.has(v->frame[i]))
			points.erase(v->frame[i]);

	for (i = 0; i < regions.size(); i++)
		if (v->points.has(regions[i]._seed)) {
			points.erase(regions[i]._seed);
			points.push_back(regions[i].center());
		}
	return points;
}
VoronoiCity::VoronoiCity(): Spatial()
{
	Rect2 bounds(-1000, -1000, 2000, 2000);
	Vector<Vector2> random_points;
	int count = 200, i;
	rnd = memnew(RandomNumberGenerator);
	for (i = 0; i < count; i++) {
		Vector2 rp = Vector2(rnd->randf(), rnd->randf())
			* bounds.size + bounds.position;
		if (random_points.find(rp) < 0)
			random_points.push_back(rp);
	}
//	Vector<int> tris = Geometry::triangulate_delaunay_2d(random_points);
//	voronoi.triangles.clear();
//	voronoi.points.clear();
	Voronoi voronoi(bounds, random_points);
	voronoi.triangulate();
	for (i = 0; i < 4; i++) {
		Vector<Vector2> points = Voronoi::relax(&voronoi, bounds);
		voronoi = Voronoi(bounds, points);
		voronoi.triangulate();
	}
	List<Region> regions = Voronoi::part();
}
VoronoiCity::~VoronoiCity()
{
}
