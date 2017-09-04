#include "SimSphere.h"

cSimSphere::tParams::tParams()
{
	mType = eTypeDynamic;
	mMass = 1;
	mFriction = 0.9;
	mPos = tVector(0, 0, 0, 0);
	mRadius = 1;
	mAxis = tVector(0, 0, 1, 1);
	mVel.setZero();
	mTheta = 0;
}

cSimSphere::cSimSphere()
{
}

cSimSphere::~cSimSphere()
{
}

void cSimSphere::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	mType = params.mType;
	btScalar mass = (params.mType == eTypeDynamic) ? static_cast<btScalar>(params.mMass) : 0;

	mShape = world->BuildSphereShape(params.mRadius);

	btVector3 inertia(0, 0, 0);
	mShape->calculateLocalInertia(mass, inertia);
	btRigidBody::btRigidBodyConstructionInfo cons_info(mass, this, mShape.get(), inertia);
	mSimBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mSimBody->setFriction(static_cast<btScalar>(params.mFriction));
	
	cSimObj::Init(world);
	SetPos(params.mPos);
	SetLinearVelocity(params.mVel);
	SetRotation(params.mAxis, params.mTheta);
}

double cSimSphere::GetRadius() const
{
	double r = mWorld->GetSphereRadius(this);
	return r;
}

tVector cSimSphere::GetSize() const
{
	double r = GetRadius();
	double d = 2 * r;
	return tVector(d, d, d, 0);
}

cSimSphere::eShape cSimSphere::GetShape() const
{
	return eShapeSphere;
}