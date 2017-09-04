#pragma once

#include <memory>

#include "btBulletDynamicsCommon.h"
#include "sim/ContactManager.h"
#include "sim/PerturbManager.h"
#include "anim/KinTree.h"
#include "util/MathUtil.h"

class cSimObj;
class cSimBox;
class cSimCapsule;
class cSimPlane;
class cSimSphere;
class cJoint;

class cWorld : public std::enable_shared_from_this<cWorld>
{
public:

	enum eSimMode
	{
		eSimMode2D,
		eSimMode3D,
		eSimModeMax
	};
	
	enum ePlaneCons
	{
		ePlaneConsNone,
		ePlaneConsXY,
		ePlaneConsMax
	};

	enum eContactFlag
	{
		eContactFlagCharacter = 0x1,
		eContactFlagEnvironment = 0x1 << 1,
		eContactFlagObject = 0x1 << 2,
		eContactFlagAll = cContactManager::gFlagAll
	};

	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tParams();
		eSimMode mSimMode;
		int mNumSubsteps;
		double mScale;
		tVector mGravity;
	};

	struct tJointParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tJointParams();
		cKinTree::eJointType mType;
		tVector mAnchor0; // parent
		tVector mAnchor1; // child
		tVector mEulerAngles0;
		tVector mEulerAngles1;
		tVector mLimLow;
		tVector mLimHigh;
		bool mEnableAdjacentCollision;

		double mTorqueLimit;
		double mForceLimit;

		tMatrix mJointChildTrans;
		
		// for revolute joints
		double mRefTheta;
	};

	struct tConstraintHandle
	{
		tConstraintHandle();
		bool IsValid() const;
		void Clear();
		btTypedConstraint* mCons;
	};

	struct tRayTestResult
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		cSimObj* mObj;
		tVector mHitPos;
	};
	typedef std::vector<tRayTestResult, Eigen::aligned_allocator<tRayTestResult>> tRayTestResults;

	static void ParseSimMode(const std::string& sim_mode_str, eSimMode& out_sim_mode);

	cWorld();
	virtual ~cWorld();
	virtual void Init(const tParams& params);
	virtual void Reset();
	virtual void Update(double time_elapsed);

	virtual void AddObject(cSimObj& obj);
	virtual void RemoveObject(cSimObj& obj);
	virtual tConstraintHandle AddJoint(cSimObj* obj, const tJointParams& params);
	virtual tConstraintHandle AddJoint(cSimObj* obj0, cSimObj* obj1, const tJointParams& params);
	virtual void Constrain(cSimObj& obj);
	virtual void Constrain(cSimObj& obj, const tVector& linear_factor, const tVector& angular_factor);
	virtual void RemoveConstraint(tConstraintHandle& handle);

	virtual void BuildConsFactor(ePlaneCons plane_cons, tVector& out_linear_factor, tVector& out_angular_factor);

	virtual void SetGravity(const tVector& gravity);

	virtual cContactManager::tContactHandle RegisterContact(int contact_flags, int filter_flags);
	virtual void UpdateContact(const cContactManager::tContactHandle& handle);
	virtual bool IsInContact(const cContactManager::tContactHandle& handle) const;
	virtual tVector GetContactPt(const cContactManager::tContactHandle& handle) const;

	virtual void RayTest(const tVector& beg, const tVector& end, tRayTestResults& results) const;
	virtual void AddPerturb(const tPerturb& perturb);
	virtual const cPerturbManager& GetPerturbManager() const;

	virtual eSimMode GetSimMode() const;
	virtual tVector GetGravity() const;
	virtual double GetScale() const;
	virtual void SetDefaultLinearDamping(double damping);
	virtual void SetDefaultAngularDamping(double damping);
	virtual void SetDamping(double linear_damping, double angular_damping, cSimObj& out_obj);

	// yuck, avoid using this
	std::unique_ptr<btDiscreteDynamicsWorld>& GetInternalWorld();

	// object interface
	virtual tVector GetPos(const cSimObj* obj) const;
	virtual void SetPos(const tVector& pos, cSimObj* out_obj) const;
	virtual tVector GetLinearVelocity(const cSimObj* obj) const;
	virtual tVector GetLinearVelocity(const cSimObj* obj, const tVector& local_pos) const;
	virtual void SetLinearVelocity(const tVector& vel, cSimObj* out_obj) const;
	virtual tVector GetAngularVelocity(const cSimObj* obj) const;
	virtual void SetAngularVelocity(const tVector& vel, const cSimObj* obj) const;
	
	virtual tMatrix GetWorldTransform(const cSimObj* obj) const;
	virtual tMatrix GetLocalTransform(const cSimObj* obj) const;
	virtual void GetRotation(const cSimObj* obj, tVector& out_axis, double& out_theta) const;
	virtual tQuaternion GetRotation(const cSimObj* obj) const;
	virtual void SetRotation(const tVector& axis, double theta, cSimObj* out_obj) const;
	virtual void SetRotation(const tQuaternion& q, cSimObj* out_obj) const;
	virtual void CalcAABB(const cSimObj* obj, tVector& out_min, tVector& out_max) const;

	virtual void ApplyForce(const tVector& force, const tVector& local_pos, cSimObj* out_obj) const;
	virtual void ApplyTorque(const tVector& torque, cSimObj* out_obj) const;

	virtual std::unique_ptr<btBoxShape> BuildBoxShape(const tVector& box_size) const;
	virtual std::unique_ptr<btCapsuleShape> BuildCapsuleShape(double radius, double height) const;
	virtual std::unique_ptr<btStaticPlaneShape> BuildPlaneShape(const tVector& normal, const tVector& origin) const;
	virtual std::unique_ptr<btSphereShape> BuildSphereShape(double radius) const;

	virtual tVector GetBoxSize(const cSimBox* box) const;
	virtual double GetCapsuleHeight(const cSimCapsule* cap) const;
	virtual double GetCapsuleRadius(const cSimCapsule* cap) const;
	virtual tVector GetPlaneCoeffs(const cSimPlane* plane) const;
	virtual double GetSphereRadius(const cSimSphere* box) const;

	virtual void CalcRotationRevolute(const cJoint* joint, tVector& out_axis, double& out_theta) const;
	virtual double CalcDisplacementPrismatic(const cJoint* joint) const;
	virtual void CalcRotationSpherical(const cJoint* joint, tVector& out_axis, double& out_theta) const;

	virtual tVector GetManifoldPtA(const btManifoldPoint& manifold_pt) const;
	virtual tVector GetManifoldPtB(const btManifoldPoint& manifold_pt) const;

protected:
	struct tConstraintEntry
	{
		cSimObj* mObj0;
		cSimObj* mObj1;
	};

	tParams mParams;
	double mDefaultLinearDamping;
	double mDefaultAngularDamping;
	std::unique_ptr<btDiscreteDynamicsWorld> mSimWorld;

	std::unique_ptr<btConstraintSolver> mSolver;
	std::unique_ptr<btCollisionDispatcher> mCollisionDispatcher;
	std::unique_ptr<btDefaultCollisionConfiguration> mCollisionConfig;
	std::unique_ptr<btBroadphaseInterface> mBroadPhase;

	cContactManager mContactManager;
	cPerturbManager mPerturbManager;

	virtual void ClearConstraints();
	virtual int GetNumConstriants() const;

	virtual tConstraintHandle AddRevoluteConstraint(cSimObj* obj0, cSimObj* obj1, const tJointParams& params);
	virtual tConstraintHandle AddPrismaticConstraint(cSimObj* obj0, cSimObj* obj1, const tJointParams& params);
	virtual tConstraintHandle AddSphericalConstraint(cSimObj* obj0, cSimObj* obj1, const tJointParams& params);

	virtual tMatrix GetWorldTransformAux(const cSimObj* obj) const;
	virtual ePlaneCons GetPlaneCons(eSimMode sim_mode) const;
};
