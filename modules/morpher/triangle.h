#ifndef TRIANGLE_H
#define TRIANGLE_H
#include <cassert>
#include <core/reference.h>
#include <core/resource.h>
#include <scene/resources/mesh.h>

class TriangleSet: public Reference {
	GDCLASS(TriangleSet, Reference)
protected:
	PoolVector<Vector3> vertices;
	PoolVector<Vector2> uvs1;
	PoolVector<Vector2> uvs2;
	PoolVector<int> indices;
	static void _bind_methods();
	float minp[3];
	float maxp[3];
	float cd[3];
	static inline void draw_hline(Image *image, const float *v1, const float *v2)
	{
		if (v1[0] < 0 && v2[0] < 0)
			return;
		if (v1[0] >= v2[0])
			return;
		if (v1[0] >= image->get_width())
			return;
		float l = (v2[0] - v1[0]);
		Color c;
		for (int i = MAX(0, (int)v1[0] - 1); i <= MIN(image->get_width() - 1, (int)v2[0] + 1); i++) {
			float t = ((float)i - v1[0] + 1)/ (l + 2.0f);
			t = CLAMP(t, 0.0f, 1.0f);
			c.r = Math::lerp(v1[2], v2[2], t);
			c.g = Math::lerp(v1[3], v2[3], t);
			c.b = Math::lerp(v1[4], v2[4], t);
			image->set_pixel(i, v1[1], c);
		}
	}
#if 0
	static inline void draw_hline(Image *image, float x1, float x2, int y,
			Color c1, Color c2)
	{
		assert(y >= 0 && y < image->get_height());
		if (x1 < 0 && x2 < 0)
			return;
		if (x1 >= x2)
			return;
		assert(x1 <= x2);
		if (x1 >= image->get_width())
			return;
		float l = (x2 - x1);
		for (int i = MAX(0, (int)x1 - 1); i <= MIN(image->get_width() - 1, (int)x2 + 1); i++) {
			float t = ((float)i - x1 + 1)/ (l + 2.0f);
			t = CLAMP(t, 0.0f, 1.0f);
			image->set_pixel(i, y, c1.linear_interpolate(c2, t));
		}

	}
#endif
	static inline void flat_bottom_triangle(Image *image,
			const float *v1, const float *v2, const float *v3)
	{
		if ((v2[1] - v1[1]) < 1.0)
			return;
		double bdiv = (v2[1] - v1[1]);
		for (int scanlineY = v1[1]; scanlineY <= v2[1]; scanlineY++) {
			float t = ((double)((double)scanlineY - v1[1])) / bdiv;
			t = CLAMP(t, 0.0f, 1.0f);
			if (scanlineY < 0 || scanlineY >= image->get_height())
				continue;
			float cx1[5], cx2[5];
			cx1[0] = Math::lerp(v1[0], v2[0], t);
			cx1[1] = scanlineY;
			cx1[2] = Math::lerp(v1[2], v2[2], t);
			cx1[3] = Math::lerp(v1[3], v2[3], t);
			cx1[4] = Math::lerp(v1[4], v2[4], t);
			cx2[0] = Math::lerp(v1[0], v3[0], t);
			cx2[1] = scanlineY;
			cx2[2] = Math::lerp(v1[2], v3[2], t);
			cx2[3] = Math::lerp(v1[3], v3[3], t);
			cx2[4] = Math::lerp(v1[4], v3[4], t);
			draw_hline(image, cx1, cx2);
		}
	}
#if 0
	static inline void flat_bottom_triangle(Image *image,
			const Vector2 &v1, const Vector2 &v2, const Vector2 &v3,
			const Color &c1, const Color &c2, const Color& c3)
	{
		if ((v2.y - v1.y) < 1.0)
			return;
		float invslope1 = (v2.x - v1.x) / (v2.y - v1.y);
		float invslope2 = (v3.x - v1.x) / (v3.y - v1.y);
		double bdiv = (v2.y - v1.y);

		float curx1 = v1.x;
		float curx2 = v1.x;

		for (int scanlineY = v1.y; scanlineY <= v2.y; scanlineY++) {
			float t = ((double)((double)scanlineY - v1.y)) / bdiv;
			t = CLAMP(t, 0.0f, 1.0f);
			if (scanlineY < 0 || scanlineY >= image->get_height())
				continue;
			Color cx1 = c1.linear_interpolate(c2, t);
			Color cx2 = c1.linear_interpolate(c3, t);
			curx1 = Math::lerp(v1.x, v2.x, t);
			curx2 = Math::lerp(v1.x, v3.x, t);
			draw_hline(image, curx1, curx2, scanlineY, cx1, cx2);
			curx1 += invslope1;
			curx2 += invslope2;
		}
	}
#endif
	static inline void flat_top_triangle(Image *image,
			const float *v1, const float *v2, const float *v3)
	{
		if ((v3[1] - v1[1]) < 1.0)
			return;
		double bdiv = (v3[1] - v1[1]);
		for (int scanlineY = v3[1]; scanlineY > v1[1] - 1; scanlineY--) {
			float t = (double)(v3[1] - (double)scanlineY) / bdiv;
			t = CLAMP(t, 0.0f, 1.0f);
			if (scanlineY < 0 || scanlineY >= image->get_height())
				continue;
			float cx1[5], cx2[5];
			cx1[0] = Math::lerp(v3[0], v1[0], t);
			cx1[1] = scanlineY;
			cx1[2] = Math::lerp(v3[2], v1[2], t);
			cx1[3] = Math::lerp(v3[3], v1[3], t);
			cx1[4] = Math::lerp(v3[4], v1[4], t);
			cx2[0] = Math::lerp(v3[0], v2[0], t);
			cx2[1] = scanlineY;
			cx2[2] = Math::lerp(v3[2], v2[2], t);
			cx2[3] = Math::lerp(v3[3], v2[3], t);
			cx2[4] = Math::lerp(v3[4], v2[4], t);
			draw_hline(image, cx1, cx2);
		}
	}
#if 0
	static inline void flat_top_triangle(Image *image,
			const Vector2 &v1, const Vector2 &v2, const Vector2 &v3,
			const Color &c1, const Color &c2, const Color& c3)
	{
		if ((v3.y - v1.y) < 1.0)
			return;
		float invslope1 = (v3.x - v1.x) / (v3.y - v1.y);
		float invslope2 = (v3.x - v2.x) / (v3.y - v2.y);

		double bdiv = (v3.y - v1.y);

		float curx1 = v3.x;
		float curx2 = v3.x;

		for (int scanlineY = v3.y; scanlineY > v1.y - 1; scanlineY--) {
			float t = (double)(v3.y - (double)scanlineY) / bdiv;
			t = CLAMP(t, 0.0f, 1.0f);
			if (scanlineY < 0 || scanlineY >= image->get_height())
				continue;
			Color cx1 = c3.linear_interpolate(c1, t);
			Color cx2 = c3.linear_interpolate(c2, t);
			curx1 = Math::lerp(v3.x, v1.x, t);
			curx2 = Math::lerp(v3.x, v2.x, t);
			assert(curx1 <= curx2);
			draw_hline(image, curx1, curx2, scanlineY, cx1, cx2);
			curx1 -= invslope1;
			curx2 -= invslope2;
		}
	}
#endif
	inline float distance_squared(const float *v1, const float *v2)
	{
		return Vector2(v1[0], v1[1]).distance_squared_to(Vector2(v2[0], v2[1]));
	}
	inline void draw_triangle(Image *image,
			const float *v1,
			const float *v2,
			const float *v3)
	{
		if (v1[1] == v2[1] && v1[1] == v3[1])
			return;
		float d12 = distance_squared(v1, v2);
		float d13 = distance_squared(v1, v3);
		const float *points[] = {v1, v2, v3};
		int i, j;
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++) {
				if (i == j)
					continue;
				if (points[i][1] < points[j][1])
					SWAP(points[i], points[j]);
			}
		if (points[0][1] == points[1][1]) {
			if (points[2][0] - points[0][0] < points[2][0] - points[1][0])
				flat_top_triangle(image, points[1], points[0], points[2]);
			else
				flat_top_triangle(image, points[0], points[1], points[2]);
		} else if (points[1][1] == points[2][1]) {
			if (points[1][0] - points[0][0] > points[2][0] - points[0][0])
				flat_bottom_triangle(image, points[0], points[2], points[1]);
			else
				flat_bottom_triangle(image, points[0], points[1], points[2]);
		} else {
			float y01 = points[1][1] - points[0][1];
			float y02 = points[2][1] - points[0][1];
			float p4[5];
			// Vector2 p4(points[0][0] + (y01 / y02) * (points[2][0] - points[0][0]), points[1][1]);
			float t = y01 / y02;
			assert(t <= 1.0f && t >= 0.0f);
			p4[0] = Math::lerp(points[0][0], points[2][0], t);
			p4[1] = points[1][1];
			p4[2] = Math::lerp(points[0][2], points[2][2], t);
			p4[3] = Math::lerp(points[0][3], points[2][3], t);
			p4[4] = Math::lerp(points[0][4], points[2][4], t);
			if (points[1][0] - points[0][0] > p4[0] - points[0][0])
				flat_bottom_triangle(image, points[0], p4, points[1]);
			else
				flat_bottom_triangle(image, points[0], points[1], p4);
			if (points[2][0] - points[1][0] < points[2][0] - p4[0])
				flat_top_triangle(image, p4, points[1], points[2]);
			else
				flat_top_triangle(image, points[1], p4, points[2]);
		}
	}
#if 0
	inline void draw_triangle(Image *image,
			Vector2 p1,
			Vector2 p2,
			Vector2 p3,
			Color c1,
			Color c2,
			Color c3)
	{
		int i, j;
		Vector2 points[] = {p1, p2, p3};
		Color colors[] = {c1, c2, c3};
		if (p1.y==p2.y && p1.y==p3.y)
			return;
		float min_edge_size = 1.9f;
		float min_edge_sq = min_edge_size * min_edge_size;
		float d12 = p1.distance_squared_to(p2);
		float d13 = p1.distance_squared_to(p3);

		if (d12 < min_edge_sq && d13 < min_edge_sq)
			return;

		Vector2 center = (p1 + p2 + p3) / 3.0f;
#if 0
		float extra_pixels = 0.0f;
		Vector2 d1 = (p1 - center).normalized() * extra_pixels;
		Vector2 d2 = (p2 - center).normalized() * extra_pixels;
		Vector2 d3 = (p3 - center).normalized() * extra_pixels;
		if (p1.y == p2.y) {
			float d = MAX(d1.y, d2.y);
			d1.y = d2.y = d;
		}
		if (p1.y == p3.y) {
			float d = MAX(d1.y, d3.y);
			d1.y = d3.y = d;
		}
		if (p2.y == p3.y) {
			float d = MAX(d2.y, d3.y);
			d2.y = d3.y = d;
		}
		points[0] += d1;
		points[1] += d2;
		points[2] += d3;
#endif
		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++) {
				if (i == j)
					continue;
				if (points[i].y < points[j].y) {
					SWAP(points[i], points[j]);
					SWAP(colors[i], colors[j]);
				}
			}
		if (points[0].y == points[1].y) {
			if (points[2].x - points[0].x < points[2].x - points[1].x) {
				SWAP(points[0], points[1]);
				SWAP(colors[0], colors[1]);
			}
			flat_top_triangle(image, points[0], points[1], points[2], colors[0], colors[1], colors[2]);
		} else if (points[1].y == points[2].y) {
			if (points[1].x - points[0].x > points[2].x - points[0].x) {
				SWAP(points[1], points[2]);
				SWAP(colors[1], colors[2]);
			}
			flat_bottom_triangle(image, points[0], points[1], points[2], colors[0], colors[1], colors[2]);
		} else {
			float y01 = points[1].y - points[0].y;
			float y02 = points[2].y - points[0].y;
			Vector2 p4(points[0].x + (y01 / y02) * (points[2].x - points[0].x), points[1].y);
			float t = y01 / y02;
			assert(t <= 1.0f && t >= 0.0f);
			Color c4 = colors[0].linear_interpolate(colors[2], t);
			if (points[1].x - points[0].x > p4.x - points[0].x)
				flat_bottom_triangle(image, points[0], p4, points[1], colors[0], colors[1], c4);
			else
				flat_bottom_triangle(image, points[0], points[1], p4, colors[0], colors[1], c4);
			if (points[2].x - points[1].x < points[2].x - p4.x)
				flat_top_triangle(image, p4, points[1], points[2], c4, colors[1], colors[2]);
			else
				flat_top_triangle(image, points[1], p4, points[2], colors[1], c4, colors[2]);
		}

#if 0
		printf("sorted: %ls %ls %ls\n",
				String(points[0]).c_str(), 
				String(points[1]).c_str(), 
				String(points[2]).c_str()
		      );
#endif
	}
#endif
public:
	TriangleSet()
	{
	}
	~TriangleSet()
	{
	}
	void normalize_deltas();
	static inline float get_area(Vector3 p1, Vector3 p2, Vector3 p3)
	{
		return (p2 - p1).cross(p3 - p1).length() / 2.0;
	}
	static inline Vector3 get_baricentric(Vector3 pt, Vector3 p1, Vector3 p2, Vector3 p3)
	{
		Vector3 p;
		float area = get_area(p1, p2, p3);
		if (area == 0.0) {
			printf("bad triangle %ls %ls %ls\n", String(p1).c_str(), String(p2).c_str(), String(p3).c_str());
			return Vector3(-1, -1, -1);
		}
		Vector3 n = (p2 - p1).cross(p3 - p1);
		float d = n.dot(p1);
		float denom = n.dot(n);
		if (denom != 0.0) {
			float t = (-n.dot(pt) + d) / denom;
			p = pt + n * t;
		} else 
			p = pt;
		float c = get_area(p2, p3, p);
		float u = c / area;
		float e = get_area(p3, p1, p);
		float v = e / area;
		float w = 1.0f - u - v;
		if (!(u < 0 || v < 0 || w < 0))
			printf("%f %f %f %f %f %f %ls %f %f %f\n", u, v, w, d, denom, n.length(), String(p).c_str(), c, e, area);
		return Vector3(u, v, w);
	}
	/* Same topology */
	void create_from_array_shape(const Array &arrays_base, const Array &arrays_shape);
	/* Close but not the same topology */
	void create_from_array_difference(const Array &arrays_base, int uv_index1, const Array &arrays_shape, int uv_index2);
	void draw(Ref<Image> image, int uv_index);
};
#endif
