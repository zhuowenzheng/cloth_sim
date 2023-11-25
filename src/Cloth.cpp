#include <iostream>
#include <fstream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Cloth.h"
#include "Particle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"

using namespace std;
using namespace Eigen;

Cloth::Cloth(int rows, int cols,
			 const Vector3d &x00,
			 const Vector3d &x01,
			 const Vector3d &x10,
			 const Vector3d &x11,
			 double mass,
			 double alpha,
			 double damping,
			 double pradius)
{
	assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(alpha >= 0.0);
	assert(damping >= 0.0);
	assert(pradius >= 0.0);
	
	this->rows = rows;
	this->cols = cols;
	
	// TODO: Create cloth
	
	// Create particles
	int nVerts = rows*cols; // Total number of vertices
	double particleMass = mass / nVerts;

	Vector3d xStep = (x01 - x00) / (cols - 1);
	Vector3d yStep = (x10 - x00) / (rows - 1);


	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			auto p = make_shared<Particle>();
			particles.push_back(p);
			p->r = pradius;
			p->d = damping;
			
			// This if/else code only works for 2x2.
			// Replace with your code.
			Vector3d pos = x00 + i * yStep + j * xStep;
			p->x = pos;
			p->m = particleMass;

			if (i == 0 && (j == 0 || j == cols - 1)) {
				p->fixed = true;
			}
			else {
				p->fixed = false;
			}
			// Populate the other member variables of p here
		}
	}
	/*
	
	// Create x springs (replace with your code)
	springs.push_back(make_shared<Spring>(particles[0], particles[1], alpha));
	springs.push_back(make_shared<Spring>(particles[2], particles[3], alpha));

	// Create y springs (replace with your code)
	springs.push_back(make_shared<Spring>(particles[0], particles[2], alpha));
	springs.push_back(make_shared<Spring>(particles[1], particles[3], alpha));

	// Create shear springs
	springs.push_back(make_shared<Spring>(particles[0], particles[3], alpha));
	springs.push_back(make_shared<Spring>(particles[1], particles[2], alpha));

	// Create x bending springs


	// Create y bending springs

	
	*/
	
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			int index = i * cols + j;
			// structural springs
			if (j < cols - 1) {
				springs.push_back(std::make_shared<Spring>(particles[index], particles[index + 1], alpha));
			}
			if (i < rows - 1) {
				springs.push_back(std::make_shared<Spring>(particles[index], particles[index + cols], alpha));
			}
			// shear springs
			if (i < rows - 1 && j < cols - 1) {
				springs.push_back(std::make_shared<Spring>(particles[index], particles[index + cols + 1], alpha));
				springs.push_back(std::make_shared<Spring>(particles[index + 1], particles[index + cols], alpha));
			}
			// bending springs
			if (j < cols - 2) {
				springs.push_back(std::make_shared<Spring>(particles[index], particles[index + 2], alpha));
			}
			if (i < rows - 2) {
				springs.push_back(std::make_shared<Spring>(particles[index], particles[index + 2 * cols], alpha));
			}
		}
	}
	
	// Build vertex buffers
	posBuf.clear();
	norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts*3);
	norBuf.resize(nVerts*3);
	updatePosNor();
	
	// Texture coordinates (don't change)
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			texBuf.push_back(i/(rows-1.0));
			texBuf.push_back(j/(cols-1.0));
		}
	}
	
	// Elements (don't change)
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			// Triangle strip
			eleBuf.push_back(k0);
			eleBuf.push_back(k1);
		}
	}
}

Cloth::~Cloth()
{
}

void Cloth::tare()
{
	for(auto p : particles) {
		p->tare();
	}
}

void Cloth::reset()
{
	for(auto p : particles) {
		p->reset();
	}
	updatePosNor();
}

void Cloth::updatePosNor()
{
	// Position
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k = i*cols + j;
			Vector3d x = particles[k]->x; // updated position
			posBuf[3*k+0] = x(0);
			posBuf[3*k+1] = x(1);
			posBuf[3*k+2] = x(2);
		}
	}
	
	// Normal
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			// Each particle has four neighbors
			//
			//      v1
			//     / | \
			// u0 /__|__\ u1
			//    \  |  /
			//     \ | /
			//      v0
			//
			// Use these four triangles to compute the normal
			int k = i*cols + j;
			int ku0 = k - 1;
			int ku1 = k + 1;
			int kv0 = k - cols;
			int kv1 = k + cols;
			Vector3d x = particles[k]->x;
			Vector3d xu0, xu1, xv0, xv1, dx0, dx1, c;
			Vector3d nor(0.0, 0.0, 0.0);
			int count = 0;
			// Top-right triangle
			if(j != cols-1 && i != rows-1) {
				xu1 = particles[ku1]->x;
				xv1 = particles[kv1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Top-left triangle
			if(j != 0 && i != rows-1) {
				xu1 = particles[kv1]->x;
				xv1 = particles[ku0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-left triangle
			if(j != 0 && i != 0) {
				xu1 = particles[ku0]->x;
				xv1 = particles[kv0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-right triangle
			if(j != cols-1 && i != 0) {
				xu1 = particles[kv0]->x;
				xv1 = particles[ku1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			nor /= count;
			nor.normalize();
			norBuf[3*k+0] = nor(0);
			norBuf[3*k+1] = nor(1);
			norBuf[3*k+2] = nor(2);
		}
	}

}

void Cloth::step(double h, const Vector3d &grav, const vector< shared_ptr<Particle> > spheres)
{
	// 1. Apply external forces and semi-implicit Euler integration
	for (auto& p : particles) {
		if (!p->fixed) {
			Vector3d force = p->m * grav - p->d * p->v; // Gravity force - Damping force
			p->v += h * force / p->m; // Update velocity
			p->p = p->x; // Store previous position
			p->x += h * p->v; // Update position
		}
	}

	// 2. Solve constraints (XPBD)
	for (auto& s : springs) {
		
			auto p0 = s->p0;
			auto p1 = s->p1;
			Vector3d x0 = p0->x;
			Vector3d x1 = p1->x;
			double w0 = 1.0 / p0->m;
			double w1 = 1.0 / p1->m;

			// Calculate constraint C and gradient ∇C
			double restLength = s->L;
			Vector3d delta = x1 - x0;
			double dist = delta.norm();
			double C = dist - restLength;
			Vector3d gradC0 = -delta / dist;
			Vector3d gradC1 = delta / dist;

			// Calculate λ (Lagrange multiplier)
			double alpha = s->alpha;
			double lambda = -C / (w0 * gradC0.squaredNorm() + w1 * gradC1.squaredNorm() + alpha / (h * h));

			// Update positions
			if (!p0->fixed) {
				p0->x += lambda * w0 * gradC0;
			}
			if (!p1->fixed) {
				p1->x += lambda * w1 * gradC1;
			}

		
	}
	// 4. Collisions
	for (auto& s : spheres) {
		for (auto& p : particles) {
			Vector3d sphereCenter = s->x;
			double sphereRadius = s->r;

			Vector3d particlePos = p->x;
			Vector3d diff = particlePos - sphereCenter;
			double dist = diff.norm();

			// Check if the particle is inside the sphere
			if (dist < sphereRadius) {
				// Project the particle out to the surface of the sphere
				Vector3d correction = diff.normalized() * (sphereRadius - dist);
				particlePos += correction;

				// Update particle position
				p->x = particlePos;
			}
		}
	}
	// 3. Update velocities
	for (auto& p : particles) {
		if (!p->fixed) {
			p->v = (1.0 / h) * (p->x - p->p);
		}
	}

	
	// Update position and normal buffers
	updatePosNor();
}

void Cloth::init()
{
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}

void Cloth::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const
{
	// Draw mesh
	glUniform3f(p->getUniform("kdFront"), 0.894f, 0.882f, 0.792f);
	glUniform3f(p->getUniform("kdBack"),  0.776f, 0.843f, 0.835f);
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	int h_pos = p->getAttribute("aPos");
	glEnableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_nor = p->getAttribute("aNor");
	glEnableVertexAttribArray(h_nor);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_tex = p->getAttribute("aTex");
	if(h_tex >= 0) {
		glEnableVertexAttribArray(h_tex);
		glBindBuffer(GL_ARRAY_BUFFER, texBufID);
		glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	}
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	for(int i = 0; i < rows; ++i) {
		glDrawElements(GL_TRIANGLE_STRIP, 2*cols, GL_UNSIGNED_INT, (const void *)(2*cols*i*sizeof(unsigned int)));
	}
	if(h_tex >= 0) {
		glDisableVertexAttribArray(h_tex);
	}
	glDisableVertexAttribArray(h_nor);
	glDisableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}
