#include "morpher.h"
#include <core/io/compression.h>
#include <core/math/math_funcs.h>

DNA_::DNA_(String &path) {
}
DNA_::DNA_() {
	modify_mesh_mutex = Mutex::create();
}
DNA_::~DNA_() {
}
void DNA_::_bind_methods() {
	ClassDB::bind_method(D_METHOD("add_mesh", "part", "mesh", "same_verts"), &DNA_::add_mesh);
	ClassDB::bind_method(D_METHOD("add_cloth_mesh", "cloth_name", "cloth_helper", "mesh"), &DNA_::add_cloth_mesh);
	ClassDB::bind_method(D_METHOD("add_body_mesh", "body_mesh", "same_verts"), &DNA_::add_body_mesh);
	ClassDB::bind_method(D_METHOD("_prepare_cloth", "body_mesh", "cloth_mesh"), &DNA_::_prepare_cloth);
	ClassDB::bind_method(D_METHOD("find_body_points", "body_mesh", "cloth_mesh"), &DNA_::find_body_points);
	ClassDB::bind_method(D_METHOD("get_modifier_list"), &DNA_::get_modifier_list);
	ClassDB::bind_method(D_METHOD("triangulate_uv", "v0", "vs", "uvs"), &DNA_::triangulate_uv);
	ClassDB::bind_method(D_METHOD("get_replace_point_id", "point_pool", "vc", "vb_id", "selected_points"), &DNA_::get_replace_point_id);
	ClassDB::bind_method(D_METHOD("triangulate_uv_arrays", "point", "uv_index", "arrays", "points"), &DNA_::triangulate_uv_arrays);
	ClassDB::bind_method(D_METHOD("ensure_uv2", "arrays"), &DNA_::ensure_uv2);
	ClassDB::bind_method(D_METHOD("get_closest_points", "point", "pool_vertices", "indices", "count"), &DNA_::get_closest_points);
	ClassDB::bind_method(D_METHOD("load", "path"), &DNA_::load);
	ClassDB::bind_method(D_METHOD("modify_mesh", "orig_mesh", "same_verts"), &DNA_::modify_mesh);
	ClassDB::bind_method(D_METHOD("modify_part", "part"), &DNA_::modify_part);
	ClassDB::bind_method(D_METHOD("set_modifier_value", "mod_name", "value"), &DNA_::set_modifier_value);
}

Dictionary DNA_::find_body_points(const Ref<ArrayMesh> &body_mesh, const Ref<ArrayMesh> &cloth_mesh) {
	Dictionary tmp;
	Array arrays_cloth = cloth_mesh->surface_get_arrays(0);
	Array arrays_body = body_mesh->surface_get_arrays(0);
	const PoolVector<Vector3> &cloth_vertices = arrays_cloth[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector3> &body_vertices = arrays_body[Mesh::ARRAY_VERTEX];
	for (int vcloth = 0; vcloth < cloth_vertices.size(); vcloth++) {
		for (int vbody = 0; vbody < body_vertices.size(); vbody++) {
			Vector3 vc = cloth_vertices[vcloth];
			Vector3 vb = body_vertices[vbody];
			if (vc.distance_to(vb) < 0.02f) {
				if (tmp.has(vcloth)) {
					PoolVector<int> data = tmp[vcloth];
					data.push_back(vbody);
					tmp[vcloth] = data;
				} else {
					PoolVector<int> data;
					data.push_back(vbody);
					tmp[vcloth] = data;
				}
			}
		}
	}
	return tmp;
}

Ref<ArrayMesh> DNA_::_prepare_cloth(const Ref<ArrayMesh> &body_mesh, const Ref<ArrayMesh> &cloth_mesh) {
	Array arrays_cloth = cloth_mesh->surface_get_arrays(0);
	ensure_uv2(arrays_cloth);
	Array arrays_body = body_mesh->surface_get_arrays(0);
	const PoolVector<Vector3> &cloth_vertices = arrays_cloth[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector3> &body_vertices = arrays_body[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector2> &body_uvs = arrays_body[Mesh::ARRAY_TEX_UV];
	Dictionary tmp = find_body_points(body_mesh, cloth_mesh);
	PoolVector<Vector2> cloth_uv2 = arrays_cloth[Mesh::ARRAY_TEX_UV2];
	Array tmp_keys = tmp.keys();
	for (int i = 0; i < tmp_keys.size(); i++) {
		int k = tmp_keys[i];
		Vector3 vc = cloth_vertices[k];
		PoolVector<int> data = tmp[k];
		PoolVector<int> res = get_closest_points(vc, body_vertices, data, 3);
		if (res.size() == 3) {
			Vector2 uv = triangulate_uv_arrays(vc, Mesh::ARRAY_TEX_UV, arrays_body, res);
			cloth_uv2.write()[k] = uv;
		}
	}
	arrays_cloth[Mesh::ARRAY_TEX_UV2] = cloth_uv2;
	for (int s = 0; s < cloth_uv2.size(); s++) {
		PoolVector<Vector2> uv1 = arrays_cloth[Mesh::ARRAY_TEX_UV];
		PoolVector<Vector2> uv2 = arrays_cloth[Mesh::ARRAY_TEX_UV2];
	}

	Ref<ArrayMesh> new_mesh = memnew(ArrayMesh);
	new_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays_cloth);
	return new_mesh;
}
Ref<ArrayMesh> DNA_::modify_mesh(const Ref<ArrayMesh> orig_mesh, const Dictionary same_verts) {
	Ref<ArrayMesh> ret = memnew(ArrayMesh);
	PoolVector<String> modifier_list = get_modifier_list();
	lock_all_images();

	Rect2 rect = collect_rects();
	modify_mesh_mutex->lock();
	int surface_count = orig_mesh->get_surface_count();
	for (int surface = 0; surface < surface_count; surface++) {
		Array arrays = orig_mesh->surface_get_arrays(surface);
		int uv_index = Mesh::ARRAY_TEX_UV;
		if (arrays[Mesh::ARRAY_TEX_UV2].get_type() != Variant::NIL)
			uv_index = Mesh::ARRAY_TEX_UV2;
		PoolVector<Vector3> vertices = arrays[Mesh::ARRAY_VERTEX];
		PoolVector<Vector3> normals = arrays[Mesh::ARRAY_NORMAL];
		PoolVector<Vector2> uvs = arrays[uv_index];
		mod_cache.resize(vertices.size());
		Vector3 n, v, diff, diffn;
		Vector2 uv;
		for (int index = 0; index < vertices.size(); index++) {
			// printf("init index: %d v: %ls n: %ls\n", index, String(v).c_str(), String(n).c_str());
			uv = uvs[index];
			if (!rect.has_point(uv))
				continue;
			v = vertices[index];
			n = normals[index];
			if (mod_cache[index].get_type() == Variant::NIL) {
				Dictionary data;
				mod_cache[index] = data;
			}
			for (int ki = 0; ki < modifier_list.size(); ki++) {
				String key = modifier_list[ki];
				Rect2 mrect = map_rect[key];
				float value = mod_value[key];
				if (!mrect.has_point(uv) || value < 0.0001)
					continue;
				Dictionary cache = mod_cache[index];
				if (cache.has(key)) {
					Array data = cache[key];
					diff = data[0];
					diffn = data[1];
				} else {
					diff = Vector3();
					diffn = Vector3();
					Vector2 pos(uv.x * (float)image_sizes[key].x, uv.y * (float)image_sizes[key].y);
					Color offset = map_vertices[key]->get_pixelv(pos);
					Color offsetn = map_normals[key]->get_pixelv(pos);
					Vector3 pdiff(offset.r, offset.g, offset.b);
					Vector3 ndiff(offsetn.r, offsetn.g, offsetn.b);
					for (int u = 0; u < 3; u++) {
						diff[u] = Math::range_lerp(pdiff[u], 0.0f, 1.0f, min_point[u], max_point[u]);
						diffn[u] = Math::range_lerp(ndiff[u], 0.0f, 1.0f, min_normal[u], max_normal[u]);
						if (fabs(diff[u]) < 0.0001f)
							diff[u] = 0.0f;
						if (fabs(diffn[u]) < 0.0001f)
							diffn[u] = 0.0f;
					}
					Array cache_data;
					cache_data.push_back(diff);
					cache_data.push_back(diffn);
					cache[key] = cache_data;
					// printf("diff: %ls diffn: %ls\n", String(diff).c_str(), String(diffn).c_str());
				}
				v -= diff * value;
				n -= diffn * value;
			}
			vertices.write()[index] = v;
			normals.write()[index] = n;
		}
		Array idx_keys = same_verts.keys();
		for (int vxk = 0; vxk < idx_keys.size(); vxk++) {
			int xv = idx_keys[vxk];
			Array data = same_verts[xv];
			if (data.size() <= 1)
				continue;
			Vector3 vx = vertices[data[0]];
			for (int idx = 1; idx < data.size(); idx++)
				vx.linear_interpolate(vertices[data[idx]], 0.5f);
			for (int idx = 0; idx < data.size(); idx++)
				vertices.write()[data[idx]] = vx;
		}
		arrays[Mesh::ARRAY_VERTEX] = vertices;
		arrays[Mesh::ARRAY_NORMAL] = normals;
		ret->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
		Ref<Material> mat = orig_mesh->surface_get_material(surface);
		if (mat != NULL)
			ret->surface_set_material(surface, mat);
	}
	modify_mesh_mutex->unlock();
	unlock_all_images();
	return ret;
}

Variant DNA_::get_var(FileAccess *f) const {
	uint32_t len = f->get_32();
	PoolVector<uint8_t> buff;
	buff.resize(len);
	PoolVector<uint8_t>::Write w = buff.write();
	f->get_buffer(&w[0], len);
	w.release();
	PoolVector<uint8_t>::Read r = buff.read();

	Variant v;
	decode_variant(v, &r[0], len, NULL, false);
	return v;
}

bool DNA_::load(const String &path) {
	Error err;
	FileAccess *f = FileAccess::open(path, FileAccess::READ, &err);
	if (!f || err != OK)
		return false;
	min_point = get_var(f);
	max_point = get_var(f);
	min_normal = get_var(f);
	max_normal = get_var(f);
	maps = get_var(f);
	vert_indices = get_var(f);
	printf("min: %ls max: %ls\n", String(min_point).c_str(), String(max_point).c_str());
	if (f)
		f->close();
	Array map_keys = maps.keys();
	for (int i = 0; i < map_keys.size(); i++) {
		String key = map_keys[i];
		Dictionary map = maps[key];
		Rect2 rect = map["rect"];
		map_rect[key] = rect;
		PoolVector<uint8_t> compressed_data = map["image_data"];
		PoolVector<uint8_t> compressed_normal_data = map["image_normal_data"];
		printf("%ls: %ls\n / points: %d normals: %d\n",
				key.c_str(),
				String(rect).c_str(),
				compressed_data.size(),
				compressed_normal_data.size());
		int width, height, size, normal_size, format;
		size = map["image_size"];
		normal_size = map["image_normal_size"];
		width = map["width"];
		height = map["height"];
		format = map["format"];
		printf("size: %d normal size: %d\n", size, normal_size);
		image_sizes[key] = Vector2i(width, height);
		PoolVector<uint8_t> decompressed_data, decompressed_normal_data;
		decompressed_data.resize(size);
		decompressed_normal_data.resize(normal_size);
		Compression::decompress(decompressed_data.write().ptr(), size,
				compressed_data.read().ptr(),
				compressed_data.size(), Compression::MODE_FASTLZ);
		Compression::decompress(decompressed_normal_data.write().ptr(),
				normal_size, compressed_normal_data.read().ptr(),
				compressed_normal_data.size(), Compression::MODE_FASTLZ);
		Ref<Image> vertex_image = memnew(Image);
		Ref<Image> normal_image = memnew(Image);
		vertex_image->create(width, height, false, (enum Image::Format)format, decompressed_data);
		normal_image->create(width, height, false, (enum Image::Format)format, decompressed_normal_data);
		map_vertices[key] = vertex_image;
		map_normals[key] = vertex_image;
	}
	return true;
}
