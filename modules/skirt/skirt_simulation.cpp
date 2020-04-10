#include <cassert>
#include "skirt.h"
SkirtSimulation::SkirtSimulation(): damping(0.04f),
     stiffness(0.2f), gravity(Vector3(0, -9.8, 0))
{
}
void SkirtSimulation::init(int size_x, int size_y)
{
    int count = size_x * size_y;
    particles.resize(count * 3);
    particles_prev.resize(count * 3);
    accel.resize(count * 3);
    this->size_x = size_x;
    this->size_y = size_y;
}
const float *SkirtSimulation::get_particles() const
{
    return particles.ptr();
}
float *SkirtSimulation::get_particles()
{
    return particles.ptrw();
}
const float *SkirtSimulation::get_particles_prev() const
{
    return particles_prev.ptr();
}
float *SkirtSimulation::get_particles_prev()
{
    return particles_prev.ptrw();
}
const float *SkirtSimulation::get_accel() const
{
    return accel.ptr();
}
float *SkirtSimulation::get_accel()
{
    return accel.ptrw();
}

void SkirtSimulation::verlet_step(float delta) {
	float *p = get_particles();
	float *pp = get_particles_prev();
	float *acc = get_accel();
	for (int i = 0; i < particles.size(); i++) {
		float x = p[i];
		assert(!isinf(x));
		float temp = x;
		float oldx = pp[i];
		float a = acc[i];
		assert(!isnan(a));
		assert(!isnan(oldx));
		assert(!isnan(temp));
		x += (x - oldx) * (1.0f - damping) + a * delta * delta;
		assert(!isnan(x));
		assert(!isinf(x));
		oldx = temp;
		p[i] = x;
		pp[i] = oldx;
	}
}
void SkirtSimulation::forces_step(float delta, const Transform &external_pos) {
	int i;
    Vector3 vel = external_pos.origin - external_pos_prev;
    Transform base = external_pos;
    base.origin = Vector3();
    base.xform_inv(vel);
    Vector3 acc = -vel / delta;
    Vector3 local_gravity = base.xform_inv(gravity);
    printf("acc: %f %f %f\n", vel.length(), acc.length(), delta);
	for (i = 0; i < (int)particles.size(); i++) {
        get_accel()[i] = local_gravity.coord[i % 3] * 1.8f + acc.coord[i % 3] * 12.0f;
		assert(!isnan(get_accel()[i]));
	}
    external_pos_prev = external_pos.origin;
}

void SkirtSimulation::set_particle(int id, const Vector3 &pt)
{
    float *p = &(get_particles()[id * 3]);
    p[0] = pt.coord[0];
    p[1] = pt.coord[1];
    p[2] = pt.coord[2];
}
Vector3 SkirtSimulation::get_particle(int id)
{
    Vector3 ret;
    const float *p = &(get_particles()[id * 3]);
    ret.coord[0] = p[0];
    ret.coord[1] = p[1];
    ret.coord[2] = p[2];
    return ret;
}
void SkirtSimulation::set_particle_prev(int id, const Vector3 &pt)
{
    float *p = &(get_particles_prev()[id * 3]);
    p[0] = pt.coord[0];
    p[1] = pt.coord[1];
    p[2] = pt.coord[2];
}
Vector3 SkirtSimulation::get_particle_prev(int id)
{
    Vector3 ret;
    const float *p = &(get_particles_prev()[id * 3]);
    ret.coord[0] = p[0];
    ret.coord[1] = p[1];
    ret.coord[2] = p[2];
    return ret;
}
void SkirtSimulation::constraints_step(float delta, const struct constraint * c, int count, const Vector3 *pin)
{
    int i, j, k;
   	for (k = 0; k < 15; k++) {
		for (i = 0; i < count; i++) {
			float x0[3], x1[3];
			float dx1[3], l1, d1, diff1;
			int p0, p1;
			p0 = c[i].p1;
			p1 = c[i].p2;
			for (j = 0; j < 3; j++) {
				x0[j] = get_particles()[p0 * 3 + j];
				x1[j] = get_particles()[p1 * 3 + j];
				dx1[j] = x1[j] - x0[j];
			}
			d1 = c[i].distance;
			if (d1 == 0.0f)
				continue;
			l1 = Vector3(dx1[0], dx1[1], dx1[2]).length();
			diff1 = (l1 - d1) / l1;
			for (j = 0; j < 3; j++) {
				x0[j] += dx1[j] * 0.5f * diff1 * stiffness;
				x1[j] -= dx1[j] * 0.5f * diff1 * stiffness;
			}
			for (j = 0; j < 3; j++) {
				get_particles()[p0 * 3 + j] = x0[j];
				get_particles()[p1 * 3 + j] = x1[j];
			}
		}
		process_collisions();
		/* pin top row */
		for (i = 0; i < size_x; i++) {
			set_particle(i, pin[i]);
		}
	}
}

float SkirtSimulation::distance(int p1, int p2) {
	Vector3 vp1 = get_particle(p1);
	Vector3 vp2 = get_particle(p2);
	return vp1.distance_to(vp2);
}

void SkirtSimulation::add_collider(int bone, struct collider &col)
{
    if (colliders.has(bone))
        colliders.erase(bone);
    colliders[bone] = col;
}
void SkirtSimulation::update_colliders(const Skeleton *skel)
{
    List<int> collider_list;
	colliders.get_key_list(&collider_list);
	for (List<int>::Element *c = collider_list.front(); c; c = c->next()) {
		int bone = c->get();
		assert(colliders.has(bone));
    	colliders[bone].update(skel);
	}
}