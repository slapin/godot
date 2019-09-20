#ifndef TRIANGLE_H
#define TRIANGLE_H
#include <cassert>
#include <core/reference.h>
#include <core/resource.h>
#include <core/bind/core_bind.h>
#include <core/os/file_access.h>
#include <scene/resources/mesh.h>

class TriangleSet: public Reference {
	GDCLASS(TriangleSet, Reference)
protected:
	PoolVector<Vector3> vertices;
	PoolVector<Vector3> normals;
	PoolVector<Vector2> uvs1;
	PoolVector<Vector2> uvs2;
	PoolVector<int> indices;
	static void _bind_methods();
	float minp[3];
	float maxp[3];
	float cd[3];
	float minn[3];
	float maxn[3];
	float cdn[3];
	int minx, miny, maxx, maxy;
	PoolVector<uint8_t> get_data(const Ref<Image> &vimage, const Ref<Image> &nimage)
	{
		int i, j, width = vimage->get_width(), height = vimage->get_height();
		const PoolVector<uint8_t> &vdata = vimage->get_data(), &ndata = nimage->get_data();
		assert(vdata.size() == ndata.size());
		uint32_t *h_table = memnew_arr(uint32_t, height + 1);
		h_table[0] = (uint32_t) height;
		uint64_t *src_buffer = memnew_arr(uint64_t, width);
		uint8_t *dst_buffer = memnew_arr(uint8_t, width * 9);
		PoolVector<uint8_t> ret;
		if (minx == maxx || miny == maxy)
			return ret;
		/* map of height offsets */
		ret.resize(ret.size() + height * 4 + 4);
		const uint32_t *pxdata = (const uint32_t *)vdata.read().ptr();
		const uint32_t *npxdata = (const uint32_t *)vdata.read().ptr();
		int out_count = ret.size();
		for (i = miny; i < maxy + 1; i++) {
			h_table[i + 1] = out_count;
			for (j = minx; j < maxx + 1; j++) {
				uint64_t data_l = (pxdata[i * width + j] & 0xFFFFFFU);
				uint64_t data_h = (npxdata[i * width + j] & 0xFFFFFFU);
				src_buffer[j] = (data_l | (data_h << 24)) & 0xFFFFFFFFFFFFULL;
			}
			uint8_t count = 1;
			uint64_t cur = src_buffer[0];
			int dst_count = 0;
			for (j = minx + 1; j < maxx + 1; j++) {
				if (cur == src_buffer[j] && count < 255 && j < maxx)
					count++;
				else {
					int k;
					for (k = 0; k < 6; k++) {
						dst_buffer[dst_count++] = (uint8_t)(cur & 0xff);
						cur >>= 8;
					}
					dst_buffer[dst_count++] = count;
					cur = src_buffer[j];
					count = 1;
				}
			}
			if (ret.size() < out_count + dst_count)
				ret.resize(out_count + dst_count * 2);
			uint8_t *dptr = ret.write().ptr();
			memcpy(&dptr[out_count], dst_buffer, dst_count);
			out_count += dst_count;
		}
		uint32_t *table_p = (uint32_t *)ret.read().ptr();
		memcpy(table_p, h_table, (height + 1) * 4);
		return ret;
	}
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
	inline void update_bounds(float x, float y)
	{
		if (minx > (int)x)
			minx = (int)x;
		if (miny > (int)y)
			miny = (int)y;
		if (maxx < ceil(x))
			maxx = (int)(ceil(x));
		if (maxy < ceil(y))
			maxy = (int)(ceil(y));
	}
	void draw(Ref<Image> vimage, Ref<Image> nimage, int uv_index);
	Vector3 get_min()
	{
		return Vector3(minp[0], minp[1], minp[2]);
	}
	Vector3 get_max()
	{
		return Vector3(maxp[0], maxp[1], maxp[2]);
	}
	Vector3 get_min_normal()
	{
		return Vector3(minn[0], minn[1], minn[2]);
	}
	Vector3 get_max_normal()
	{
		return Vector3(maxn[0], maxn[1], maxn[2]);
	}
	void save(Ref<_File> fd, const String &shape_name, Ref<Image> vimage, Ref<Image> nimage);
};
#endif
