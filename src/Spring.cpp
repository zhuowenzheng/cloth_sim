#include "Spring.h"
#include "Particle.h"

using namespace std;
using namespace Eigen;

Spring::Spring(shared_ptr<Particle> p0, shared_ptr<Particle> p1, double alpha) {
    assert(p0);
    assert(p1);
    assert(p0 != p1);
    this->p0 = p0;
    this->p1 = p1;
    this->alpha = alpha;

    // Compute rest length L
    Vector3d pos0 = p0->x;
    Vector3d pos1 = p1->x;
    L = (pos1 - pos0).norm();
}

Spring::~Spring() {
    // Destructor logic (if any)
}
