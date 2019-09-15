#include "modifierset.h"

void Modifier::modify(float *data, int vertex_count)
{
	for (int i = 0; i < mod_indices.size(); i++) {
		int index = mod_indices[i];
		float vx = mod_data[i * 6 + 0];
		float vy = mod_data[i * 6 + 2];
		float vz = mod_data[i * 6 + 4];
		float nx = mod_data[i * 6 + 1];
		float ny = mod_data[i * 6 + 3];
		float nz = mod_data[i * 6 + 5];
		data[index * 14 + 2] -= vx * value;
		data[index * 14 + 3] -= vy * value;
		data[index * 14 + 4] -= vz * value;
		data[index * 14 + 5] -= nx * value;
		data[index * 14 + 6] -= ny * value;
		data[index * 14 + 7] -= nz * value;
	}
}
void Modifier::create_from_images(const float *meshdata, int count, 
		Image *vdata, Image *ndata, const Vector3 &vmin,
		const Vector3 &vmax, const Vector3 &nmin, const Vector3 &nmax)
{
	int i, j;
	for (i = 0; i < 3; i++) {
		minp[i] = vmin[i];
		maxp[i] = vmax[i];
		minn[i] = nmin[i];
		maxn[i] = nmax[i];
	}
	vdata->lock();
	ndata->lock();
	for (i = 0; i < count; i++) {
		int vx = (int)(meshdata[i * 14 + 0] * (float)vdata->get_width());
		int vy = (int)(meshdata[i * 14 + 1] * (float)vdata->get_height());
		Color c = vdata->get_pixel(vx, vy);
		Color nc = ndata->get_pixel(vx, vy);
		float pdelta[3], ndelta[3];
		for (j = 0; j < 3; j++) {
			pdelta[j] = minp[j] + (maxp[j] - minp[j]) * c[j];
			ndelta[j] = minn[j] + (maxn[j] - minn[j]) * nc[j];
		}
		const float eps = 0.001f;
		if (pdelta[0] * pdelta[0] + pdelta[1] * pdelta[1] + pdelta[2] * pdelta[2] > eps * eps) {
			mod_indices.push_back(i);
			for (j = 0; j < 3; j++) {
				mod_data.push_back(pdelta[j]);
				mod_data.push_back(ndelta[j]);
			}
		}
	}
	ndata->unlock();
	vdata->unlock();
}
void ModifierSet::add_modifier(const String &name, Ref<Image> vimage, Ref<Image> nimage, const PoolVector<float> &minmax)
{
	Vector3 vmin(minmax[0], minmax[1], minmax[2]);
	Vector3 vmax(minmax[3], minmax[4], minmax[5]);
	Vector3 nmin(minmax[6], minmax[7], minmax[8]);
	Vector3 nmax(minmax[9], minmax[10], minmax[11]);
	modifiers[mod_count].create_from_images(meshdata, vertex_count, vimage.ptr(), nimage.ptr(), vmin, vmax, nmin, nmax);
	name2mod[name] = mod_count;
	mod_count++;
}
void ModifierSet::add_mesh(const String &name, Ref<ArrayMesh> mesh)
{
	int i, j;
	work_mesh = mesh;
	surface = mesh->surface_get_arrays(0);
	mat = mesh->surface_get_material(0);
	const PoolVector<Vector2> &uvdata = surface[uv_index];
	const PoolVector<Vector3> &vdata = surface[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector3> &normal = surface[Mesh::ARRAY_NORMAL];
	meshdata = memnew_arr(float, vdata.size() * 14);
	vertex_count = vdata.size();
	const Vector2 *uvs = uvdata.read().ptr();
	const Vector3 *n = normal.read().ptr();
	const Vector3 *v = vdata.read().ptr();
	for (i = 0; i < uvdata.size(); i++) {
		meshdata[i * 14 + 0] = uvs[i][0];
		meshdata[i * 14 + 1] = uvs[i][1];
		meshdata[i * 14 + 2] = v[i][0];
		meshdata[i * 14 + 3] = v[i][1];
		meshdata[i * 14 + 4] = v[i][2];
		meshdata[i * 14 + 5] = n[i][0];
		meshdata[i * 14 + 6] = n[i][1];
		meshdata[i * 14 + 7] = n[i][2];
		meshdata[i * 14 + 8] = v[i][0];
		meshdata[i * 14 + 9] = v[i][1];
		meshdata[i * 14 + 10] = v[i][2];
		meshdata[i * 14 + 11] = n[i][0];
		meshdata[i * 14 + 12] = n[i][1];
		meshdata[i * 14 + 13] = n[i][2];
	}
	float eps_dist = 0.0001f;
	for (i = 0; i < vdata.size(); i++) {
		for (j = 0; j < vdata.size(); j++) {
			if (i == j)
				continue;
			if (v[i].distance_squared_to(v[j]) < eps_dist * eps_dist) {
				if (!same_verts.has(i))
					same_verts[i] = PoolVector<int>();
				same_verts[i].push_back(j);
			}
		}

	}
}
void ModifierSet::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("add_modifier", "name", "vimage", "nimage", "minmax"), &ModifierSet::add_modifier);
	ClassDB::bind_method(D_METHOD("add_mesh", "name", "mesh"), &ModifierSet::add_mesh);
	ClassDB::bind_method(D_METHOD("set_modifier_value", "name", "value"), &ModifierSet::set_modifier_value);
	ClassDB::bind_method(D_METHOD("get_modifier_value", "name"), &ModifierSet::get_modifier_value);
	ClassDB::bind_method(D_METHOD("set_uv_index", "index"), &ModifierSet::set_uv_index);
	ClassDB::bind_method(D_METHOD("modify"), &ModifierSet::modify);
}

void ModifierSet::modify()
{
	int i, j;
	if (!dirty)
		return;
	for (i = 0; i < vertex_count; i++) {
		meshdata[i * 14 + 2] =  meshdata[i * 14 + 8];
		meshdata[i * 14 + 3] =  meshdata[i * 14 + 9];
		meshdata[i * 14 + 4] =  meshdata[i * 14 + 10];
		meshdata[i * 14 + 5] =  meshdata[i * 14 + 11];
		meshdata[i * 14 + 6] =  meshdata[i * 14 + 12];
		meshdata[i * 14 + 7] =  meshdata[i * 14 + 13];
	}
	for (i = 0; i < mod_count; i++)
		modifiers[i].modify(meshdata, vertex_count);
	for (i = 0; i < vertex_count; i++) {
		if (same_verts.has(i)) {
			float vx = meshdata[i * 14 + 2];
			float vy = meshdata[i * 14 + 3];
			float vz = meshdata[i * 14 + 4];
			float nx = meshdata[i * 14 + 5];
			float ny = meshdata[i * 14 + 6];
			float nz = meshdata[i * 14 + 7];
			for (j = 0; j < same_verts[i].size(); j++) {
				vx = Math::lerp(vx, meshdata[same_verts[i][j] * 14 + 2], 0.5f);
				vy = Math::lerp(vy, meshdata[same_verts[i][j] * 14 + 3], 0.5f);
				vz = Math::lerp(vz, meshdata[same_verts[i][j] * 14 + 4], 0.5f);
				nx = Math::lerp(nx, meshdata[same_verts[i][j] * 14 + 5], 0.5f);
				ny = Math::lerp(ny, meshdata[same_verts[i][j] * 14 + 6], 0.5f);
				nz = Math::lerp(nz, meshdata[same_verts[i][j] * 14 + 7], 0.5f);
			}
			meshdata[i * 14 + 2] = vx;
			meshdata[i * 14 + 3] = vy;
			meshdata[i * 14 + 4] = vz;
			meshdata[i * 14 + 5] = nx;
			meshdata[i * 14 + 6] = ny;
			meshdata[i * 14 + 7] = nz;
			for (j = 0; j < same_verts[i].size(); j++) {
				meshdata[same_verts[i][j] * 14 + 2] = vx;
				meshdata[same_verts[i][j] * 14 + 3] = vy;
				meshdata[same_verts[i][j] * 14 + 4] = vz;
				meshdata[same_verts[i][j] * 14 + 5] = nx;
				meshdata[same_verts[i][j] * 14 + 6] = ny;
				meshdata[same_verts[i][j] * 14 + 7] = nz;
			}
		}
	}
	PoolVector<Vector3> vertices = surface[Mesh::ARRAY_VERTEX];
	PoolVector<Vector3> normals = surface[Mesh::ARRAY_NORMAL];
	PoolVector<Vector3>::Write vertex_w = vertices.write();
	PoolVector<Vector3>::Write normal_w = normals.write();
	for (i = 0; i < vertex_count; i++) {
		vertex_w[i].x = meshdata[i * 14 + 2];
		vertex_w[i].y = meshdata[i * 14 + 3];
		vertex_w[i].z = meshdata[i * 14 + 4];
		normal_w[i].x = meshdata[i * 14 + 5];
		normal_w[i].y = meshdata[i * 14 + 6];
		normal_w[i].z = meshdata[i * 14 + 7];
	}
	vertex_w.release();
	normal_w.release();
	surface[Mesh::ARRAY_VERTEX] = vertices;
	surface[Mesh::ARRAY_NORMAL] = normals;
	work_mesh->surface_remove(0);
	work_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, surface);
	work_mesh->surface_set_material(0, mat);

	dirty = false;
}

