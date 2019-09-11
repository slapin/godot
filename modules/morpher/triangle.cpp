#include "triangle.h"
void TriangleSet::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("create_from_array_shape", "arrays_base", "shape_arrays"), &TriangleSet::create_from_array_shape);
	ClassDB::bind_method(D_METHOD("create_from_mesh_difference", "arrays_base", "uv_index1", "arrays_shape", "uv_index2"), &TriangleSet::create_from_array_difference);
	ClassDB::bind_method(D_METHOD("draw", "image", "uv_index"), &TriangleSet::draw);
}
void TriangleSet::create_from_array_shape(const Array &arrays_base, const Array &shape_array)
{
	const PoolVector<int> &data_indices = shape_array[Mesh::ARRAY_INDEX];
	const PoolVector<Vector3> &base_vertices = arrays_base[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector3> &shape_vertices = shape_array[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector2> &uvs1 = shape_array[Mesh::ARRAY_TEX_UV];
	const PoolVector<Vector2> &uvs2 = shape_array[Mesh::ARRAY_TEX_UV2];
	int i;
	indices = data_indices;
	vertices.resize(base_vertices.size());
	this->uvs1.resize(base_vertices.size());
	this->uvs2.resize(base_vertices.size());
	PoolVector<Vector3>::Write vertices_write = this->vertices.write();
	PoolVector<Vector2>::Write uvs1_w = this->uvs1.write();
	PoolVector<Vector2>::Write uvs2_w = this->uvs2.write();
	for (i = 0; i < base_vertices.size(); i++) {
		vertices_write[i] = shape_vertices[i] - base_vertices[i];
		uvs1_w[i] = uvs1.read()[i];
		uvs2_w[i] = uvs2.read()[i];
	}
	printf("create done\n");
}
void TriangleSet::create_from_array_difference(const Array &arrays_base, int uv_index1, const Array &arrays_shape, int uv_index2)
{
	Vector<int> missing_vertices;
	const PoolVector<Vector3> &base_vertices = arrays_base[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector3> &shape_vertices = arrays_shape[Mesh::ARRAY_VERTEX];
	switch(uv_index1) {
	case 0:
		uv_index1 = Mesh::ARRAY_TEX_UV;
		break;
	case 1:
		uv_index1 = Mesh::ARRAY_TEX_UV2;
		break;
	default:
		uv_index1 = Mesh::ARRAY_TEX_UV;
	}
	switch(uv_index2) {
	case 0:
		uv_index2 = Mesh::ARRAY_TEX_UV;
		break;
	case 1:
		uv_index2 = Mesh::ARRAY_TEX_UV2;
		break;
	default:
		uv_index2 = Mesh::ARRAY_TEX_UV;
	}
	const PoolVector<Vector2> &base_uvs = arrays_base[uv_index1];
	const PoolVector<Vector2> &shape_uvs = arrays_shape[uv_index1];
	const float okdist = 0.0001f;
	int i;
	for (i = 0; i < base_vertices.size(); i++) {
		if (base_uvs[i].distance_squared_to(shape_uvs[i]) < okdist * okdist)
			this->vertices[i] = shape_vertices[i] - base_vertices[i];
		else
			missing_vertices.push_back(i);
	}
}
void TriangleSet::draw(Ref<Image> image, int uv_index)
{
	int i;
	int width = image->get_width();
	float maxp[3];
	float cd[3];
	int height = image->get_height();
	Vector2 mulv(width, height);
	normalize_deltas();
	image->lock();
	PoolVector<Vector2>::Read uvs_r;
	PoolVector<int>::Read indices_r = indices.read();
	PoolVector<Vector3>::Read vertices_r = vertices.read();
	switch(uv_index) {
	case 0:
		uvs_r = uvs1.read();
		break;
	case 1:
		uvs_r = uvs2.read();
		break;
	}
	printf("draw %d\n", indices.size() / 3);
#if 0
	float w = 64.0f;
	for (i = 0; i < 1000; i++) {
		int y = i % 64;
		int x = i / 64;
		Vector2 p1(w * x, w * y * 2.0f);
		Vector2 p2(w * x + w, w * y * 2.0f);
		Vector2 p3(w * x + w, w * y * 2.0f + w);
		Vector2 p4(w * x, w * y * 2.0f + w);
		Vector2 p5(w * x + w, w * y * 2.0f + w * 2.0f);
		Vector2 p6(w * x, w * y * 2.0f + w * 2.0);
		Color c1(1, 1, 1);
		Color c2(1, 0, 0);
		Color c3(0, 0, 1);
		Color c4(0, 1, 0);
		Color c5(0, 1, 1);
		Color c6(1, 1, 0);
		float v1[] = {w * x + 45.0, w * y * 2.0f, 1, 1, 1};
		float v2[] = {w * x + w + 45.0, w * y * 2.0f, 1, 0, 0};
		float v3[] = {w * x + w, w * y * 2.0f + w, 0, 0, 1};
		float v4[] = {w * x, w * y * 2.0f + w, 0, 1, 0};
		float v5[] = {w * x + w + 60.0, w * y * 2.0f + w * 2.0f, 0, 1, 1};
		float v6[] = {w * x + 60.0, w * y * 2.0f + w * 2.0, 1, 1, 0};
		draw_triangle(image.ptr(), v1, v4, v3);
		draw_triangle(image.ptr(), v1, v2, v3);
		draw_triangle(image.ptr(), v3, v4, v6);
		draw_triangle(image.ptr(), v3, v5, v6);
		// draw_triangle(image.ptr(), Vector2(w * x + w, w * y), Vector2(w * x, w * y + w), Vector2(w * x + w, w * y + w), Color(1, 0, 0), Color(0, 0, 1), Color(1, 1, 1));
	}
#else
	for (i = 0; i < indices.size(); i += 3) {
		const Vector2 &vp1 = uvs_r[indices_r[i + 0]];
		const Vector2 &vp2 = uvs_r[indices_r[i + 1]];
		const Vector2 &vp3 = uvs_r[indices_r[i + 2]];
		const Vector3 &vc1 = vertices_r[indices_r[i + 0]];
		const Vector3 &vc2 = vertices_r[indices_r[i + 1]];
		const Vector3 &vc3 = vertices_r[indices_r[i + 2]];
		float v1[] = {vp1.x * mulv.x, vp1.y * mulv.y, vc1.x, vc1.y, vc1.z};
		float v2[] = {vp2.x * mulv.x, vp2.y * mulv.y, vc2.x, vc2.y, vc2.z};
		float v3[] = {vp3.x * mulv.x, vp3.y * mulv.y, vc3.x, vc3.y, vc3.z};
		draw_triangle(image.ptr(), v1, v2, v3);
	}
#endif
	image->unlock();
}

void TriangleSet::normalize_deltas()
{
	int i, j;
	minp[0] = 100.0f;
	minp[1] = 100.0f;
	minp[2] = 100.0f;
	maxp[0] = -100.0f;
	maxp[1] = -100.0f;
	maxp[2] = -100.0f;
	for (i = 0; i < vertices.size(); i++) {
		for (j = 0; j < 3; j++) {
			if (minp[j] > vertices[i][j])
				minp[j] = vertices[i][j];
			if (maxp[j] < vertices[i][j])
				maxp[j] = vertices[i][j];
		}
	}
	for (i = 0; i < 3; i++)
		cd[i] = maxp[i] - minp[i];
	PoolVector<Vector3>::Write vertices_w = vertices.write();
	for (i = 0; i < vertices.size(); i++)
		for (j = 0; j < 3; j++) {
			if (cd[j] == 0.0f)
				vertices_w[i][j] = 0.0f;
			else
				vertices_w[i][j] = (vertices_w[i][j] - minp[j]) / cd[j];
		}
}
