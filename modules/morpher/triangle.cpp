#include "triangle.h"
void TriangleSet::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("create_from_array_shape", "arrays_base", "shape_arrays"), &TriangleSet::create_from_array_shape);
	ClassDB::bind_method(D_METHOD("create_from_mesh_difference", "arrays_base", "uv_index1", "arrays_shape", "uv_index2"), &TriangleSet::create_from_array_difference);
	ClassDB::bind_method(D_METHOD("draw", "vimage", "nimage", "uv_index"), &TriangleSet::draw);
	ClassDB::bind_method(D_METHOD("get_min"), &TriangleSet::get_min);
	ClassDB::bind_method(D_METHOD("get_max"), &TriangleSet::get_max);
	ClassDB::bind_method(D_METHOD("get_min_normal"), &TriangleSet::get_min_normal);
	ClassDB::bind_method(D_METHOD("get_max_normal"), &TriangleSet::get_max_normal);
}
void TriangleSet::create_from_array_shape(const Array &arrays_base, const Array &shape_array)
{
	const PoolVector<int> &data_indices = shape_array[Mesh::ARRAY_INDEX];
	const PoolVector<Vector3> &base_vertices = arrays_base[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector3> &shape_vertices = shape_array[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector3> &base_normals = arrays_base[Mesh::ARRAY_NORMAL];
	const PoolVector<Vector3> &shape_normals = shape_array[Mesh::ARRAY_NORMAL];
	const PoolVector<Vector2> &uvs1 = shape_array[Mesh::ARRAY_TEX_UV];
	const PoolVector<Vector2> &uvs2 = shape_array[Mesh::ARRAY_TEX_UV2];
	int i;
	indices = data_indices;
	vertices.resize(base_vertices.size());
	normals.resize(base_vertices.size());
	this->uvs1.resize(base_vertices.size());
	this->uvs2.resize(base_vertices.size());
	PoolVector<Vector3>::Write vertices_write = this->vertices.write();
	PoolVector<Vector3>::Write normals_write = this->normals.write();
	PoolVector<Vector2>::Write uvs1_w = this->uvs1.write();
	PoolVector<Vector2>::Write uvs2_w = this->uvs2.write();
	for (i = 0; i < base_vertices.size(); i++) {
		vertices_write[i] = shape_vertices[i] - base_vertices[i];
		normals_write[i] = shape_normals[i] - base_normals[i];
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
	const PoolVector<Vector3> &base_normals = arrays_base[Mesh::ARRAY_NORMAL];
	const PoolVector<Vector3> &shape_normals = arrays_shape[Mesh::ARRAY_NORMAL];
	const PoolVector<int> &shape_index = arrays_shape[Mesh::ARRAY_INDEX];
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
	indices = arrays_base[Mesh::ARRAY_INDEX];
	this->vertices.resize(base_vertices.size());
	this->uvs1 = arrays_base[Mesh::ARRAY_TEX_UV];
	this->uvs2 = arrays_base[Mesh::ARRAY_TEX_UV2];
	normals.resize(base_vertices.size());
	const float okdist = 0.001f;
	int i, j;
	PoolVector<Vector3>::Write vertices_write = this->vertices.write();
	PoolVector<Vector3>::Write normals_write = this->normals.write();
	for (i = 0; i < base_vertices.size(); i++) {
		bool ok = false;
		for (j = 0; j < shape_vertices.size(); j++) {
			if (base_uvs[i].distance_squared_to(shape_uvs[j]) < okdist * okdist) {
				vertices_write[i] = shape_vertices[j] - base_vertices[i];
				normals_write[i] = shape_normals[j] - base_normals[i];
				ok = true;
				break;
			}
		}
		if (!ok)
			missing_vertices.push_back(i);
	}
	for (i = 0; i < missing_vertices.size(); i++) {
		int base_index = missing_vertices[i];
		Vector2 pt2 = base_uvs[base_index];
		Vector3 pt = Vector3(pt2.x, pt2.y, 0.0f);
		for (j = 0; j < shape_index.size();j += 3) {
			Vector3 p1 = Vector3(shape_uvs[shape_index[j + 0]].x,
					shape_uvs[shape_index[j + 0]].y,
					0.0f);
			Vector3 p2 = Vector3(shape_uvs[shape_index[j + 1]].x,
					shape_uvs[shape_index[j + 1]].y,
					0.0f);
			Vector3 p3 = Vector3(shape_uvs[shape_index[j + 2]].x,
					shape_uvs[shape_index[j + 2]].y,
					0.0f);
			Vector3 b = get_baricentric(pt, p1, p2, p3);
			if (b.x < 0 || b.y < 0 || b.z < 0)
				continue;
			Vector3 newpt = shape_vertices[shape_index[j + 0]] * b.x
				+ shape_vertices[shape_index[j + 1]] * b.y
				+ shape_vertices[shape_index[j + 2]] * b.z;
			Vector3 newn = shape_normals[shape_index[j + 0]] * b.x
				+ shape_normals[shape_index[j + 1]] * b.y
				+ shape_normals[shape_index[j + 2]] * b.z;
			vertices_write[base_index] = newpt - base_vertices[base_index];
			normals_write[base_index] = newn - base_normals[base_index];
		}
	}
	vertices_write.release();
	normals_write.release();
}
void TriangleSet::draw(Ref<Image> vimage, Ref<Image> nimage, int uv_index)
{
	int i, j;
	int width = vimage->get_width();
	int height = vimage->get_height();
	int nwidth = nimage->get_width();
	int nheight = nimage->get_height();
	Vector2 mulv(width, height);
	Vector2 muln(nwidth, nheight);
	normalize_deltas();
	vimage->lock();
	nimage->lock();
	PoolVector<Vector2>::Read uvs_r;
	PoolVector<int>::Read indices_r = indices.read();
	PoolVector<Vector3>::Read vertices_r = vertices.read();
	PoolVector<Vector3>::Read normals_r = normals.read();
	switch(uv_index) {
	case 0:
		uvs_r = uvs1.read();
		break;
	case 1:
		uvs_r = uvs2.read();
		break;
	}
	for (i = 0; i < vimage->get_height(); i++)
		for (j = 0; j < vimage->get_width(); j++)
			vimage->set_pixel(j, i, Color(0.5f, 0.5f, 0.5f));
	for (i = 0; i < nimage->get_height(); i++)
		for (j = 0; j < nimage->get_width(); j++)
			nimage->set_pixel(j, i, Color(0.5f, 0.5f, 0.5f));
	for (i = 0; i < indices.size(); i += 3) {
		const Vector2 &vp1 = uvs_r[indices_r[i + 0]];
		const Vector2 &vp2 = uvs_r[indices_r[i + 1]];
		const Vector2 &vp3 = uvs_r[indices_r[i + 2]];
		Vector2 center = (vp1 + vp2 + vp3) / 3.0;
		Vector2 c1 = (vp1 - center).normalized() * 8.0;
		Vector2 c2 = (vp2 - center).normalized() * 8.0;
		Vector2 c3 = (vp3 - center).normalized() * 8.0;
		const Vector3 &vc1 = vertices_r[indices_r[i + 0]];
		const Vector3 &vc2 = vertices_r[indices_r[i + 1]];
		const Vector3 &vc3 = vertices_r[indices_r[i + 2]];
		const Vector3 &nc1 = normals_r[indices_r[i + 0]];
		const Vector3 &nc2 = normals_r[indices_r[i + 1]];
		const Vector3 &nc3 = normals_r[indices_r[i + 2]];
		float v1[] = {vp1.x * mulv.x + c1.x, vp1.y * mulv.y + c1.y, vc1.x, vc1.y, vc1.z};
		float v2[] = {vp2.x * mulv.x + c2.x, vp2.y * mulv.y + c2.y, vc2.x, vc2.y, vc2.z};
		float v3[] = {vp3.x * mulv.x + c3.x, vp3.y * mulv.y + c3.x, vc3.x, vc3.y, vc3.z};
		float n1[] = {vp1.x * muln.x + c1.x, vp1.y * muln.y + c1.y, nc1.x, nc1.y, nc1.z};
		float n2[] = {vp2.x * muln.x + c2.x, vp2.y * muln.y + c2.y, nc2.x, nc2.y, nc2.z};
		float n3[] = {vp3.x * muln.x + c3.x, vp3.y * muln.y + c3.y, nc3.x, nc3.y, nc3.z};
		draw_triangle(vimage.ptr(), v1, v2, v3);
		draw_triangle(nimage.ptr(), n1, n2, n3);
	}
	for (i = 0; i < indices.size(); i += 3) {
		const Vector2 &vp1 = uvs_r[indices_r[i + 0]];
		const Vector2 &vp2 = uvs_r[indices_r[i + 1]];
		const Vector2 &vp3 = uvs_r[indices_r[i + 2]];
		const Vector3 &vc1 = vertices_r[indices_r[i + 0]];
		const Vector3 &vc2 = vertices_r[indices_r[i + 1]];
		const Vector3 &vc3 = vertices_r[indices_r[i + 2]];
		const Vector3 &nc1 = normals_r[indices_r[i + 0]];
		const Vector3 &nc2 = normals_r[indices_r[i + 1]];
		const Vector3 &nc3 = normals_r[indices_r[i + 2]];
		float v1[] = {vp1.x * mulv.x, vp1.y * mulv.y, vc1.x, vc1.y, vc1.z};
		float v2[] = {vp2.x * mulv.x, vp2.y * mulv.y, vc2.x, vc2.y, vc2.z};
		float v3[] = {vp3.x * mulv.x, vp3.y * mulv.y, vc3.x, vc3.y, vc3.z};
		float n1[] = {vp1.x * muln.x, vp1.y * muln.y, nc1.x, nc1.y, nc1.z};
		float n2[] = {vp2.x * muln.x, vp2.y * muln.y, nc2.x, nc2.y, nc2.z};
		float n3[] = {vp3.x * muln.x, vp3.y * muln.y, nc3.x, nc3.y, nc3.z};
		draw_triangle(vimage.ptr(), v1, v2, v3);
		draw_triangle(nimage.ptr(), n1, n2, n3);
	}
	vimage->unlock();
	nimage->unlock();
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
	minn[0] = 100.0f;
	minn[1] = 100.0f;
	minn[2] = 100.0f;
	maxn[0] = -100.0f;
	maxn[1] = -100.0f;
	maxn[2] = -100.0f;
	for (i = 0; i < vertices.size(); i++) {
		for (j = 0; j < 3; j++) {
			if (minp[j] > vertices[i][j])
				minp[j] = vertices[i][j];
			if (maxp[j] < vertices[i][j])
				maxp[j] = vertices[i][j];
			if (minn[j] > normals[i][j])
				minn[j] = normals[i][j];
			if (maxn[j] < normals[i][j])
				maxn[j] = normals[i][j];
		}
	}
	for (i = 0; i < 3; i++) {
		cd[i] = maxp[i] - minp[i];
		cdn[i] = maxn[i] - minn[i];
	}
	PoolVector<Vector3>::Write vertices_w = vertices.write();
	PoolVector<Vector3>::Write normals_w = normals.write();
	for (i = 0; i < vertices.size(); i++)
		for (j = 0; j < 3; j++) {
			if (cd[j] == 0.0f)
				vertices_w[i][j] = 0.0f;
			else
				vertices_w[i][j] = (vertices_w[i][j] - minp[j]) / cd[j];
			if (cdn[j] == 0.0f)
				normals_w[i][j] = 0.0f;
			else
				normals_w[i][j] = (normals_w[i][j] - minn[j]) / cdn[j];
		}
}
