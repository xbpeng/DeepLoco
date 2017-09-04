#pragma once

#include <memory>

#include "sim/World.h"

class cSimObj : public btDefaultMotionState
{
public:
	enum eType
	{
		eTypeDynamic,
		eTypeStatic,
		eTypeMax
	};

	enum eShape
	{
		eShapeInvalid,
		eShapeBox,
		eShapePlane,
		eShapeCapsule,
		eShapeSphere,
		eShapeMax,
	};

	virtual ~cSimObj();

	virtual tVector GetPos() const;
	virtual void SetPos(const tVector& pos);
	virtual void GetRotation(tVector& out_axis, double& out_theta) const;
	virtual tQuaternion GetRotation() const;
	virtual void SetRotation(const tVector& axis, double theta);
	virtual void SetRotation(const tQuaternion& q);
	virtual tVector GetLinearVelocity() const;
	virtual tVector GetLinearVelocity(const tVector& local_pos) const;
	virtual void SetLinearVelocity(const tVector& vel);
	virtual tVector GetAngularVelocity() const;
	virtual void SetAngularVelocity(const tVector& vel);
	virtual tMatrix GetWorldTransform() const;
	virtual tMatrix GetLocalTransform() const;

	virtual void SetDamping(double linear_damping, double angular_damping);

	virtual double GetMass() const;

	virtual tVector WorldToLocalPos(const tVector& world_pos) const;
	virtual tVector LocalToWorldPos(const tVector& local_pos) const;
	virtual tMatrix3 GetLocalToWorldRotMat() const;

	virtual void ApplyForce(const tVector& force);
	virtual void ApplyForce(const tVector& force, const tVector& local_pos);
	virtual void ApplyTorque(const tVector& torque);
	virtual void ClearForces();

	virtual void RegisterContact();
	virtual void RegisterContact(int contact_flags, int filter_flags);
	virtual void UpdateContact(int contact_flags, int filter_flags);
	virtual const cContactManager::tContactHandle& GetContactHandle() const;
	virtual bool IsInContact() const;
	virtual tVector GetContactPt() const;

	virtual short GetColGroup() const;
	virtual void SetColGroup(short col_group);
	virtual short GetColMask() const;
	virtual void SetColMask(short col_mask);
	virtual void SetKinematicObject(bool is_kin);
	virtual bool IsKinematicObject() const;

	virtual void DisableDeactivation();
	virtual void CalcAABB(tVector& out_min, tVector& out_max) const;

	virtual void Constrain(const tVector& linear_factor, const tVector& angular_factor);

	virtual eType GetType() const;
	virtual eShape GetShape() const;

	virtual bool HasSimBody() const;
	virtual const std::unique_ptr<btRigidBody>& GetSimBody() const;
	virtual const std::unique_ptr<btCollisionShape>& GetCollisionShape() const;
	virtual const std::shared_ptr<cWorld>& GetWorld() const;

protected:
	std::shared_ptr<cWorld> mWorld;
	std::unique_ptr<btRigidBody> mSimBody;
	std::unique_ptr<btCollisionShape> mShape;

	cWorld::tConstraintHandle mCons;

	cContactManager::tContactHandle mContactHandle;

	eType mType;
	short mColGroup;
	short mColMask;

	cSimObj();

	virtual void Init(const std::shared_ptr<cWorld>& world);
	virtual void AddToWorld(const std::shared_ptr<cWorld>& world);
	virtual void RemoveFromWorld();

	virtual int GetNumConstraints() const;
	virtual cWorld::tConstraintHandle GetConstraint(int c) const;
};