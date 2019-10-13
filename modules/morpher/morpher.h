#ifndef MORPHER_H
#define MORPHER_H
#include <core/io/marshalls.h>
#include <core/os/file_access.h>
#include <core/reference.h>
#include <core/resource.h>
#include <scene/resources/mesh.h>

class DNA_ : public Reference {
	GDCLASS(DNA_, Reference)
protected:
	Vector3 min_point, max_point, min_normal, max_normal;
	Dictionary maps;
	Dictionary vert_indices;
	Dictionary meshes;
	Dictionary clothes;
	Mutex *modify_mesh_mutex;
	HashMap<String, Ref<Image> > map_vertices, map_normals;
	HashMap<String, float> mod_value;
	HashMap<String, Rect2> map_rect;
	HashMap<String, Vector2i> image_sizes;
	Array mod_cache;

	static void _bind_methods();
	Variant get_var(FileAccess *f) const;
	void lock_all_images() {
		PoolVector<String> modifier_list = get_modifier_list();
		for (int i = 0; i < modifier_list.size(); i++) {
			String mod_name = modifier_list[i];
			Ref<Image> iv = map_vertices[mod_name];
			Ref<Image> in = map_normals[mod_name];
			iv->lock();
			in->lock();
		}
	}
	void unlock_all_images() {
		PoolVector<String> modifier_list = get_modifier_list();
		for (int i = 0; i < modifier_list.size(); i++) {
			String mod_name = modifier_list[i];
			Ref<Image> iv = map_vertices[mod_name];
			Ref<Image> in = map_normals[mod_name];
			iv->unlock();
			in->unlock();
		}
	}
	Rect2 collect_rects() {
		Rect2 rect;
		PoolVector<String> modifier_list = get_modifier_list();
		for (int i = 0; i < modifier_list.size(); i++) {
			String mod_name = modifier_list[i];
			if (mod_value[mod_name] > 0.0001f) {
				if (rect.size.length() == 0.0)
					rect = map_rect[mod_name];
				else
					rect = rect.merge(map_rect[mod_name]);
			}
		}
		return rect;
	}
	static int farthest_index(const PoolVector<Vector3> &points, const Vector3 &point) {
		if (points.size() < 2)
			return -1;
		float distance = points[0].distance_squared_to(point);
		int index = 0;
		for (int i = 1; i < points.size(); i++) {
			float d1 = points[i].distance_squared_to(point);
			if (distance < d1) {
				distance = d1;
				index = i;
			}
		}
		return index;
	}
	template <class T>
	static bool pool_has(const PoolVector<T> &data, const T value) {
		for (int i = 0; i < data.size(); i++)
			if (data[i] == value)
				return true;
		return false;
	}
	struct mesh_data {
		String name;
		Ref<ArrayMesh> orig_mesh;
#if 0
			HashMap<int, PoolVector<int> > indices;
#endif
		Dictionary indices;
	};
	HashMap<String, struct mesh_data> meshes_;

public:
	DNA_(String &path);
	DNA_();
	~DNA_();
	static PoolVector<Vector3> fill_vector(const PoolVector<Vector3> &point_pool,
			const PoolVector<int> &selected_points) {
		PoolVector<Vector3> points;
		points.resize(selected_points.size());
		for (int i = 0; i < points.size(); i++) {
			points[i] = point_pool[selected_points[i]];
		}
		return points;
	}

	int get_replace_point_id(const PoolVector<Vector3> &point_pool,
			const Vector3 &vc, const int &vb_id,
			const PoolVector<int> &selected_points) {
		if (pool_has(selected_points, vb_id))
			return -1;
		PoolVector<Vector3> points = fill_vector(point_pool, selected_points);
		int farthest = farthest_index(points, vc);
		float d1 = vc.distance_squared_to(point_pool[vb_id]);
		float d2 = vc.distance_squared_to(point_pool[farthest]);
		if (d1 < d2)
			return farthest;
		return -1;
	}
	PoolVector<String> get_modifier_list() {
		PoolVector<String> data;
		Array kdata = maps.keys();
		for (int i = 0; i < kdata.size(); i++)
			data.push_back(kdata[i]);
		return data;
	}
	Vector2 triangulate_uv(Vector3 v0, const PoolVector<Vector3> &vs, const PoolVector<Vector2> &uvs) {
		float d1 = v0.distance_to(vs[0]);
		float d2 = v0.distance_to(vs[1]);
		float d3 = v0.distance_to(vs[2]);
		float _ln = MAX(d1, MAX(d2, d3));
		Vector3 v(d1 / _ln, d2 / _ln, d3 / _ln);
		Vector2 midp = (uvs[0] + uvs[1] + uvs[2]) / 3.0f;
		Vector2 uv = midp.linear_interpolate(uvs[0], v.x) +
					 midp.linear_interpolate(uvs[1], v.y) +
					 midp.linear_interpolate(uvs[2], v.z);
		uv /= 3.0f;
		return uv;
	}
	Vector2 triangulate_uv_arrays(const Vector3 &point, int uv_index, const Array &arrays, const PoolVector<int> &points) {
		PoolVector<Vector3> vertices;
		PoolVector<Vector2> uvs;
		const PoolVector<Vector3> &pool_vertices = arrays[Mesh::ARRAY_VERTEX];
		switch (uv_index) {
			case Mesh::ARRAY_TEX_UV:
			case Mesh::ARRAY_TEX_UV2:
				break;
			default:
				return Vector2();
		}
		const PoolVector<Vector2> &pool_uvs = arrays[uv_index];
		for (int i = 0; i < points.size(); i++) {
			vertices.push_back(pool_vertices[points[i]]);
			uvs.push_back(pool_uvs[points[i]]);
		}
		return triangulate_uv(point, vertices, uvs);
	}
	void ensure_uv2(Array arrays) {
		if (arrays[Mesh::ARRAY_TEX_UV2].get_type() == Variant::NIL)
			arrays[Mesh::ARRAY_TEX_UV2] = arrays[Mesh::ARRAY_TEX_UV];
	}
	PoolVector<int> get_closest_points(const Vector3 &point, const PoolVector<Vector3> &pool_vertices, const PoolVector<int> &indices, int count) {
		PoolVector<int> ret;
		for (int i = 0; i < indices.size(); i++) {
			int v = indices[i];
			if (ret.size() >= count) {
				int id = get_replace_point_id(pool_vertices, point, v, ret);
				if (id >= 0)
					ret.write()[id] = v;
			} else if (!pool_has(ret, v))
				ret.push_back(v);
		}
		return ret;
	}
	void add_mesh(const String &part, const Ref<ArrayMesh> &mesh, const Dictionary same_verts) {
		struct mesh_data data;
		data.name = part;
		data.orig_mesh = mesh;
		data.indices = same_verts;
		meshes_[part] = data;
#if 0
			data["orig_mesh"] = mesh;
			data["indices"] = same_verts;
			meshes[part] = data;
#endif
	}
	Ref<ArrayMesh> _prepare_cloth(const Ref<ArrayMesh> &body_mesh, const Ref<ArrayMesh> &cloth_mesh);
	Ref<ArrayMesh> get_mesh(const String &part) {
		if (meshes_.has(part))
			return meshes_[part].orig_mesh;
		return NULL;
	}
	Ref<ArrayMesh> add_cloth_mesh(const String &cloth_name, String cloth_helper, const Ref<ArrayMesh> &mesh) {
		Dictionary body = meshes["body"];
		Ref<ArrayMesh> orig_mesh = get_mesh("body");
		Ref<ArrayMesh> new_mesh = _prepare_cloth(orig_mesh, mesh);
		add_mesh(cloth_name, new_mesh, Dictionary());
		Dictionary cloth;
		cloth["helper"] = cloth_helper;
		clothes[cloth_name] = cloth;
		return new_mesh;
	}
	void add_body_mesh(const Ref<ArrayMesh> &body_mesh, const Dictionary same_verts) {
		add_mesh("body", body_mesh, same_verts);
	}
	Ref<ArrayMesh> modify_mesh(const Ref<ArrayMesh> orig_mesh, const Dictionary same_verts);
	Dictionary find_body_points(const Ref<ArrayMesh> &body_mesh, const Ref<ArrayMesh> &cloth_mesh);
	Ref<ArrayMesh> modify_part(const String &part) {
		Ref<ArrayMesh> mesh = meshes_[part].orig_mesh;
		Dictionary indices = meshes_[part].indices;
		return modify_mesh(mesh, indices);
	}
	void set_modifier_value(const String &mod_name, float value) {
		mod_value[mod_name] = value;
	}
	bool load(const String &path);
};
#endif
