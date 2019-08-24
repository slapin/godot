#ifndef MORPHER_H
#define MORPHER_H
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

		static void _bind_methods();
public:
		DNA_(String &path);
		DNA_();
		~DNA_();
		Vector<String> get_modifier_list()
		{
			Vector<String> data;
			Array kdata = maps.keys();
			for (int i = 0; i < kdata.size(); i++)
				data.push_back(kdata[i]);
			return data;
		}
		Vector2 triangulate_uv(Vector3 v0, const PoolVector<Vector3> &vs, const PoolVector<Vector2> &uvs)
		{
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
		void add_mesh(const String &part, const Ref<ArrayMesh> &mesh, const Dictionary same_verts)
		{
			Dictionary data;
			data["orig_mesh"] = mesh;
			data["indices"] = same_verts;
			meshes[part] = data;
		}
		Ref<ArrayMesh> _prepare_cloth(const Ref<ArrayMesh> &body_mesh, const Ref<ArrayMesh> &cloth_mesh);
		Ref<ArrayMesh> add_cloth_mesh(const String &cloth_name, String cloth_helper, const Ref<ArrayMesh> &mesh)
		{
			Dictionary body = meshes["body"];
			Ref<ArrayMesh> orig_mesh = body["orig_mesh"];
			Ref<ArrayMesh> new_mesh = _prepare_cloth(orig_mesh, mesh);
			add_mesh(cloth_name, new_mesh, Dictionary());
			Dictionary cloth;
			cloth["helper"] = cloth_helper;
			clothes[cloth_name] = cloth;
			return new_mesh;
		}
		void add_body_mesh(const Ref<ArrayMesh> &body_mesh, const Dictionary same_verts)
		{
			add_mesh("body", body_mesh, same_verts);
		}
		Ref<ArrayMesh> modify_mesh(const Ref<ArrayMesh> orig_mesh, const Dictionary same_verts);
};
#endif

