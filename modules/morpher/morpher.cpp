#include "morpher.h"

DNA_::DNA_(String &path)
{
}
DNA_::DNA_()
{
}
DNA_::~DNA_()
{
}
void DNA_::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("add_mesh", "part", "mesh", "same_verts"), &DNA_::add_mesh);
}

Ref<ArrayMesh> DNA_::_prepare_cloth(const Ref<ArrayMesh> &body_mesh, const Ref<ArrayMesh> &cloth_mesh)
{
	Array arrays_cloth = cloth_mesh->surface_get_arrays(0);
	if (arrays_cloth[Mesh::ARRAY_TEX_UV2].get_type() == Variant::NIL) {
		PoolVector<Vector2> d;
		PoolVector<Vector3> vertices = arrays_cloth[Mesh::ARRAY_VERTEX];
		d.resize(vertices.size());
		arrays_cloth[Mesh::ARRAY_TEX_UV2] = d;
	}
	Array arrays_body = body_mesh->surface_get_arrays(0);
	const PoolVector<Vector3> &cloth_vertices = arrays_cloth[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector3> &body_vertices = arrays_body[Mesh::ARRAY_VERTEX];
	const PoolVector<Vector2> &body_uvs = arrays_body[Mesh::ARRAY_TEX_UV];
	Dictionary tmp;
	PoolVector<Vector2> cloth_uv2 = arrays_cloth[Mesh::ARRAY_TEX_UV2];
	for (int vcloth = 0; vcloth < cloth_vertices.size(); vcloth++) {
		for (int vbody = 0; vbody < body_vertices.size(); vbody++) {
			Vector3 vc = cloth_vertices[vcloth];
			Vector3 vb = body_vertices[vbody];
			if (vc.distance_to(vb) < 0.02f) {
				if (tmp.has(vcloth)) {
					Array data = tmp[vcloth];
					data.push_back(vbody);
					tmp[vcloth] = data;
				} else {
					Array data;
					data.push_back(vbody);
					tmp[vcloth] = data;
				}
			}
		}

	}
	Array tmp_keys = tmp.keys();
	for (int i = 0; i < tmp_keys.size(); i++) {
		int k = tmp_keys[i];
		Vector3 vc = cloth_vertices[k];
		Array res;
		Array data = tmp[k];
		for (int j = 0; i < data.size(); j++) {
			int v = data[j];
			Vector3 vb = body_vertices[v];
			float d1 = vc.distance_squared_to(vb);
			if (res.size() >= 3) {
				for (int mv = 0; mv < res.size(); mv++) {
					Vector3 vb1 = body_vertices[res[mv]];
					float d2 = vc.distance_squared_to(vb1);
					if (d1 < d2 && !res.has(v))
						res[mv] = v;
				}
			}

		}
		tmp[k] = res;
		if (res.size() == 3) {
			Vector3 vtx = cloth_vertices[k];
			PoolVector<Vector3> bverts;
			PoolVector<Vector2> buvs;
			for (int rj = 0; rj < res.size(); rj++) {
				int e = res[rj];
				Vector3 vb = body_vertices[e];
				Vector2 ub = body_uvs[e];
				bverts.push_back(vb);
				buvs.push_back(ub);
			}
			cloth_uv2[k] = triangulate_uv(vtx, bverts, buvs);

		}
	}
	arrays_cloth[Mesh::ARRAY_TEX_UV2] = cloth_uv2;

	Ref<ArrayMesh> new_mesh = memnew(ArrayMesh);
	new_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays_cloth);
	return new_mesh;
}
Ref<ArrayMesh> DNA_::modify_mesh(const Ref<ArrayMesh> orig_mesh, const Dictionary same_verts)
{
	Ref<ArrayMesh> ret = memnew(ArrayMesh);
	return ret;
}

