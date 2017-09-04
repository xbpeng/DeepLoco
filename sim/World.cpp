#include "World.h"

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "sim/SimObj.h"
#include "sim/SimBox.h"
#include "sim/SimCapsule.h"
#include "sim/SimPlane.h"
#include "sim/SimSphere.h"
#include "sim/Joint.h"

cWorld::tParams::tParams()
{
	mSimMode = eSimMode2D;
	mNumSubsteps = 1;
	mScale = 1;
	mGravity = gGravity;
}

cWorld::tJointParams::tJointParams()
{
	mType = cKinTree::eJointTypeRevolute;
	mAnchor0 = tVector::Zero();
	mAnchor1 = tVector::Zero();
	mEulerAngles0 = tVector::Zero();
	mEulerAngles1 = tVector::Zero();
	mEnableAdjacentCollision = true;
	mLimLow.setOnes(); // low > high -> no limit
	mLimHigh.setZero();
	mRefTheta = 0;
}

cWorld::tConstraintHandle::tConstraintHandle()
{
	mCons = nullptr;
}

bool cWorld::tConstraintHandle::IsValid() const
{
	return mCons != nullptr;
}

void cWorld::tConstraintHandle::Clear()
{
	delete mCons;
	mCons = nullptr;
}

void cWorld::ParseSimMode(const std::string& sim_mode_str, eSimMode& out_sim_mode)
{
	if (sim_mode_str == "2d")
	{
		out_sim_mode = eSimMode2D;
	}
	else if (sim_mode_str == "3d")
	{
		out_sim_mode = eSimMode3D;
	}
}

cWorld::cWorld()
	: mContactManager(*this)
{
	mDefaultLinearDamping = 0;
	mDefaultAngularDamping = 0;
}

cWorld::~cWorld()
{
	ClearConstraints();
}

void cWorld::Init(const tParams& params)
{
	mParams = params;

	mBroadPhase = std::unique_ptr<btBroadphaseInterface>(new btDbvtBroadphase());
	mCollisionConfig = std::unique_ptr<btDefaultCollisionConfiguration>(new btDefaultCollisionConfiguration());
	mCollisionDispatcher = std::unique_ptr<btCollisionDispatcher>(new btCollisionDispatcher(mCollisionConfig.get()));
	btGImpactCollisionAlgorithm::registerAlgorithm(mCollisionDispatcher.get());

	mSolver = std::unique_ptr<btSequentialImpulseConstraintSolver>(new btSequentialImpulseConstraintSolver());
	mSimWorld = std::unique_ptr<btDiscreteDynamicsWorld>(new btDiscreteDynamicsWorld(mCollisionDispatcher.get(), 
														mBroadPhase.get(), mSolver.get(), mCollisionConfig.get()));

	btContactSolverInfo& info = mSimWorld->getSolverInfo();
	info.m_solverMode = SOLVER_SIMD | SOLVER_USE_2_FRICTION_DIRECTIONS | SOLVER_FRICTION_SEPARATE | SOLVER_USE_WARMSTARTING;

	SetGravity(params.mGravity);
	
	mContactManager.Init();
	mPerturbManager.Clear();
}

void cWorld::Reset()
{
	mContactManager.Reset();
	mPerturbManager.Clear();

	mSimWorld->clearForces();
	mSolver->reset();
	mBroadPhase->resetPool(mCollisionDispatcher.get());

	btOverlappingPairCache* pair_cache = mSimWorld->getBroadphase()->getOverlappingPairCache();
	btBroadphasePairArray& pair_array = pair_cache->getOverlappingPairArray();
	for (int i = 0; i < pair_array.size(); ++i)
	{
		pair_cache->cleanOverlappingPair(pair_array[i], mSimWorld->getDispatcher());
	}
}

void cWorld::Update(double time_elapsed)
{
	time_elapsed = std::max(0.0, time_elapsed);
	mPerturbManager.Update(time_elapsed);

	btScalar time_step = static_cast<btScalar>(time_elapsed);
	mSimWorld->stepSimulation(time_step, mParams.mNumSubsteps, time_step / mParams.mNumSubsteps);

	mContactManager.Update();
}

void cWorld::AddObject(cSimObj& obj)
{
	const std::unique_ptr<btRigidBody>& body = obj.GetSimBody();

	short col_group = obj.GetColGroup();
	short col_mask = obj.GetColMask();
	col_mask |= cContactManager::gFlagRayTest;

	SetDamping(mDefaultLinearDamping, mDefaultAngularDamping, obj);
	mSimWorld->addRigidBody(body.get(), col_group, col_mask);

	Constrain(obj);
}

void cWorld::RemoveObject(cSimObj& obj)
{
	/*
	if (obj.GetSimBody() && obj.GetSimBody()->getMotionState())
	{
		delete obj.GetSimBody().get()->getMotionState();
	}*/
	// mSimWorld->removeCollisionObject(obj.GetSimBody().get());
	mSimWorld->removeRigidBody(obj.GetSimBody().get());
	// std::cout << "Number of colission objects" << mSimWorld->m_collisionShapes.size() << std::endl;
	// obj.GetSimBody().reset();
}

cWorld::tConstraintHandle cWorld::AddJoint(cSimObj* obj, const tJointParams& params)
{
	return AddJoint(obj, nullptr, params);
}

cWorld::tConstraintHandle cWorld::AddJoint(cSimObj* obj0, cSimObj* obj1, const tJointParams& params)
{
	bool is_root = obj0 == nullptr;
	tConstraintHandle handle;

	if (!is_root)
	{
		switch (params.mType)
		{
		case cKinTree::eJointTypeRevolute:
			handle = AddRevoluteConstraint(obj0, obj1, params);
			break;
		case cKinTree::eJointTypePrismatic:
			handle = AddPrismaticConstraint(obj0, obj1, params);
			break;
		case cKinTree::eJointTypeSpherical:
			handle = AddSphericalConstraint(obj0, obj1, params);
			break;
		default:
			assert(false);
			printf("Unsupported constraint type\n");
			break;
		}
	}

	return handle;
}

void cWorld::Constrain(cSimObj& obj)
{
	Constrain(obj, tVector::Ones(), tVector::Ones());
}

void cWorld::Constrain(cSimObj& obj, const tVector& linear_factor, const tVector& angular_factor)
{
	auto& body = obj.GetSimBody();
	tVector lin_f = tVector::Ones();
	tVector ang_f = tVector::Ones();
	BuildConsFactor(GetPlaneCons(GetSimMode()), lin_f, ang_f);

	lin_f = lin_f.cwiseProduct(linear_factor);
	ang_f = ang_f.cwiseProduct(angular_factor);

	body->setLinearFactor(btVector3(static_cast<btScalar>(lin_f[0]),
									static_cast<btScalar>(lin_f[1]),
									static_cast<btScalar>(lin_f[2])));
	body->setAngularFactor(btVector3(static_cast<btScalar>(ang_f[0]),
									static_cast<btScalar>(ang_f[1]),
									static_cast<btScalar>(ang_f[2])));
}

void cWorld::RemoveConstraint(tConstraintHandle& handle)
{
	if (handle.IsValid())
	{
		mSimWorld->removeConstraint(handle.mCons);
	}
	handle.Clear();
}

void cWorld::BuildConsFactor(ePlaneCons plane_cons, tVector& out_linear_factor, tVector& out_angular_factor)
{
	out_linear_factor = tVector::Ones();
	out_angular_factor = tVector::Ones();
	switch (plane_cons)
	{
	case ePlaneConsNone:
		break;
	case ePlaneConsXY:
		out_linear_factor = tVector(1, 1, 0, 0);
		out_angular_factor = tVector(0, 0, 1, 0);
		break;
	default:
		assert(false); // unsupported constraint
	}
}

void cWorld::SetGravity(const tVector& gravity)
{
	double scale = GetScale();
	mSimWorld->setGravity(btVector3(static_cast<btScalar>(gravity[0] * scale), 
									static_cast<btScalar>(gravity[1] * scale), 
									static_cast<btScalar>(gravity[2] * scale)));
}

cContactManager::tContactHandle cWorld::RegisterContact(int contact_flags, int filter_flags)
{
	return mContactManager.RegisterContact(contact_flags, filter_flags);
}

void cWorld::UpdateContact(const cContactManager::tContactHandle& handle)
{
	mContactManager.UpdateContact(handle);
}

tVector cWorld::GetContactPt(const cContactManager::tContactHandle& handle) const
{
	return mContactManager.GetContactPt(handle);
}

bool cWorld::IsInContact(const cContactManager::tContactHandle& handle) const
{
	return mContactManager.IsInContact(handle);
}

void cWorld::RayTest(const tVector& beg, const tVector& end, tRayTestResults& results) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	btVector3 bt_beg = scale * btVector3(static_cast<btScalar>(beg[0]),
								static_cast<btScalar>(beg[1]), 
								static_cast<btScalar>(beg[2]));
	btVector3 bt_end = scale * btVector3(static_cast<btScalar>(end[0]),
								static_cast<btScalar>(end[1]),
								static_cast<btScalar>(end[2]));
	btCollisionWorld::ClosestRayResultCallback ray_callback(bt_beg, bt_end);
	
	mSimWorld->rayTest(bt_beg, bt_end, ray_callback);
	
	results.clear();
	if (ray_callback.hasHit())
	{
		auto& obj = ray_callback.m_collisionObject;
		const auto& hit_pt = ray_callback.m_hitPointWorld;
		tRayTestResult result;
		result.mObj = static_cast<cSimObj*>(obj->getUserPointer());
		result.mHitPos = tVector(hit_pt[0], hit_pt[1], hit_pt[2], 0) / scale;

		results.push_back(result);
	}
}

void cWorld::AddPerturb(const tPerturb& perturb)
{
	mPerturbManager.AddPerturb(perturb);
}

const cPerturbManager& cWorld::GetPerturbManager() const
{
	return mPerturbManager;
}

cWorld::eSimMode cWorld::GetSimMode() const
{
	return mParams.mSimMode;
}

tVector cWorld::GetGravity() const
{
	double scale = GetScale();
	btVector3 bt_gravity = mSimWorld->getGravity();
	return tVector(bt_gravity[0], bt_gravity[1], bt_gravity[2], 0) / scale;
}

double cWorld::GetScale() const
{
	return mParams.mScale;
}

void cWorld::SetDefaultLinearDamping(double damping)
{
	mDefaultLinearDamping = damping;
}

void cWorld::SetDefaultAngularDamping(double damping)
{
	mDefaultAngularDamping = damping;
}

void cWorld::SetDamping(double linear_damping, double angular_damping, cSimObj& out_obj)
{
	const std::unique_ptr<btRigidBody>& body = out_obj.GetSimBody();
	body->setDamping(static_cast<btScalar>(linear_damping), static_cast<btScalar>(angular_damping));
}

tVector cWorld::GetPos(const cSimObj* obj) const
{
	auto& body = obj->GetSimBody();
	btTransform trans = body->getWorldTransform();
	btVector3 origin = trans.getOrigin();
	tVector pos = tVector(origin[0], origin[1], origin[2], 0);
	pos /= GetScale();
	return pos;
}

void cWorld::SetPos(const tVector& pos, cSimObj* out_obj) const
{
	auto& body = out_obj->GetSimBody();

	btTransform trans = body->getWorldTransform();

	btScalar scale = static_cast<btScalar>(GetScale());
	trans.setOrigin(scale * btVector3(static_cast<btScalar>(pos[0]),
		static_cast<btScalar>(pos[1]),
		static_cast<btScalar>(pos[2])));

	body->setWorldTransform(trans);

	if (body->isKinematicObject())
	{
		btMotionState* motion_state = body->getMotionState();
		motion_state->setWorldTransform(trans);
	}
	//body->activate();
}

tVector cWorld::GetLinearVelocity(const cSimObj* obj) const
{
	auto& body = obj->GetSimBody();
	const btVector3& bt_vel = body->getLinearVelocity();
	tVector vel = tVector(bt_vel[0], bt_vel[1], bt_vel[2], 0);
	vel /= GetScale();
	return vel;
}

tVector cWorld::GetLinearVelocity(const cSimObj* obj, const tVector& local_pos) const
{
	auto& body = obj->GetSimBody();
	btScalar scale = static_cast<btScalar>(GetScale());
	tMatrix3 rot_mat = obj->GetLocalToWorldRotMat();
	tVector rel_pos = tVector::Zero();
	rel_pos.segment(0, 3) = rot_mat * local_pos.segment(0, 3);

	btVector3 bt_vel = body->getVelocityInLocalPoint(scale * btVector3(static_cast<btScalar>(rel_pos[0]),
															static_cast<btScalar>(rel_pos[1]),
															static_cast<btScalar>(rel_pos[2])));
	tVector vel = tVector(bt_vel[0], bt_vel[1], bt_vel[2], 0);
	vel /= GetScale();
	return vel;
}

void cWorld::SetLinearVelocity(const tVector& vel, cSimObj* out_obj) const
{
	auto& body = out_obj->GetSimBody();
	btScalar scale = static_cast<btScalar>(GetScale());
	body->setLinearVelocity(scale * btVector3(static_cast<btScalar>(vel[0]),
										static_cast<btScalar>(vel[1]),
										static_cast<btScalar>(vel[2])));
	//body->activate();
}

tVector cWorld::GetAngularVelocity(const cSimObj* obj) const
{
	auto& body = obj->GetSimBody();
	const btVector3& vel = body->getAngularVelocity();
	return tVector(vel[0], vel[1], vel[2], 0);
}

void cWorld::SetAngularVelocity(const tVector& vel, const cSimObj* obj) const
{
	auto& body = obj->GetSimBody();
	body->setAngularVelocity(btVector3(static_cast<btScalar>(vel[0]), static_cast<btScalar>(vel[1]),
							static_cast<btScalar>(vel[2])));
	//body->activate();
}

tMatrix cWorld::GetWorldTransform(const cSimObj* obj) const
{
	tMatrix trans = GetWorldTransformAux(obj);
	return trans;
}

tMatrix cWorld::GetLocalTransform(const cSimObj* obj) const
{
	tMatrix trans = GetWorldTransformAux(obj);
	trans = cMathUtil::InvRigidMat(trans);
	return trans;
}

void cWorld::GetRotation(const cSimObj* obj, tVector& out_axis, double& out_theta) const
{
	auto& body = obj->GetSimBody();
	btTransform trans = body->getWorldTransform();
	btQuaternion bt_q = trans.getRotation();
	btVector3 axis = bt_q.getAxis();

	out_theta = bt_q.getAngle();
	out_axis[0] = axis[0];
	out_axis[1] = axis[1];
	out_axis[2] = axis[2];
	out_axis[3] = 0;
}

tQuaternion cWorld::GetRotation(const cSimObj* obj) const
{
	auto& body = obj->GetSimBody();
	btTransform trans = body->getWorldTransform();
	btQuaternion bt_q = trans.getRotation();
	tQuaternion q = tQuaternion(bt_q.w(), bt_q.x(), bt_q.y(), bt_q.z());
	return q;
}

tMatrix cWorld::GetWorldTransformAux(const cSimObj* obj) const
{
	auto& body = obj->GetSimBody();
	const btTransform& bt_trans = body->getWorldTransform();
	const btMatrix3x3& basis = bt_trans.getBasis();
	const btVector3& origin = bt_trans.getOrigin();
	double scale = GetScale();

	tMatrix trans = tMatrix::Identity();
	for (int i = 0; i < 3; ++i)
	{
		auto curr_row = trans.row(i);
		auto bt_row = basis.getRow(i);
		for (int j = 0; j < 3; ++j)
		{
			curr_row[j] = bt_row[j];
		}
		curr_row[3] = origin[i] / scale;
	}
	return trans;
}

void cWorld::SetRotation(const tVector& axis, double theta, cSimObj* out_obj) const
{
	tQuaternion q = cMathUtil::AxisAngleToQuaternion(axis, theta);
	SetRotation(q, out_obj);
}

void cWorld::SetRotation(const tQuaternion& q, cSimObj* out_obj) const
{
	auto& body = out_obj->GetSimBody();
	btTransform trans = body->getWorldTransform();
	btQuaternion bt_q = btQuaternion(static_cast<btScalar>(q.x()), 
									static_cast<btScalar>(q.y()), 
									static_cast<btScalar>(q.z()), 
									static_cast<btScalar>(q.w()));
	trans.setRotation(bt_q);
	body->setWorldTransform(trans);

	if (body->isKinematicObject())
	{
		btMotionState* motion_state = body->getMotionState();
		motion_state->setWorldTransform(trans);
	}

	body->updateInertiaTensor();
}

void cWorld::CalcAABB(const cSimObj* obj, tVector& out_min, tVector& out_max) const
{
	auto& body = obj->GetSimBody();
	btVector3 bt_min;
	btVector3 bt_max;
	body->getAabb(bt_min, bt_max);
	double scale = GetScale();

	out_min = tVector(bt_min[0], bt_min[1], bt_min[2], 0) / scale;
	out_max = tVector(bt_max[0], bt_max[1], bt_max[2], 0) / scale;
}

void cWorld::ApplyForce(const tVector& force, const tVector& local_pos, cSimObj* out_obj) const
{
	auto& body = out_obj->GetSimBody();
	btVector3 bt_force = btVector3(static_cast<btScalar>(force[0]), 
									static_cast<btScalar>(force[1]), 
									static_cast<btScalar>(force[2]));

	btScalar scale = static_cast<btScalar>(GetScale());
	tMatrix3 rot_mat = out_obj->GetLocalToWorldRotMat();
	tVector rel_pos = tVector::Zero();
	rel_pos.segment(0, 3) = rot_mat * local_pos.segment(0, 3);

	btVector3 bt_pos = btVector3(static_cast<btScalar>(rel_pos[0]),
								static_cast<btScalar>(rel_pos[1]),
								static_cast<btScalar>(rel_pos[2]));

	bt_force *= scale;
	bt_pos *= scale;
	body->applyForce(bt_force, bt_pos);
}

void cWorld::ApplyTorque(const tVector& torque, cSimObj* out_obj) const
{
	auto& body = out_obj->GetSimBody();
	btScalar scale = static_cast<btScalar>(GetScale());
	body->applyTorque(scale * scale * btVector3(static_cast<btScalar>(torque[0]),
										static_cast<btScalar>(torque[1]),
										static_cast<btScalar>(torque[2])));
}

std::unique_ptr<btBoxShape> cWorld::BuildBoxShape(const tVector& box_size) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	std::unique_ptr<btBoxShape> shape = std::unique_ptr<btBoxShape>(new btBoxShape(scale * btVector3(static_cast<btScalar>(box_size[0] * 0.5),
																			static_cast<btScalar>(box_size[1] * 0.5),
																			static_cast<btScalar>(box_size[2] * 0.5))));
	return shape;
}

std::unique_ptr<btCapsuleShape> cWorld::BuildCapsuleShape(double radius, double height) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	std::unique_ptr<btCapsuleShape> shape = std::unique_ptr<btCapsuleShape>(new btCapsuleShape(static_cast<btScalar>(scale * radius),
																			static_cast<btScalar>(scale * height)));
	return shape;
}

std::unique_ptr<btStaticPlaneShape> cWorld::BuildPlaneShape(const tVector& normal, const tVector& origin) const
{
	btVector3 bt_normal = btVector3(static_cast<btScalar>(normal[0]),
									static_cast<btScalar>(normal[1]),
									static_cast<btScalar>(normal[2]));
	btVector3 bt_origin = btVector3(static_cast<btScalar>(origin[0]),
									static_cast<btScalar>(origin[1]),
									static_cast<btScalar>(origin[2]));
	bt_normal.normalize();
	double scale = GetScale();
	btScalar w = static_cast<btScalar>(scale * bt_normal.dot(bt_origin));
	
	std::unique_ptr<btStaticPlaneShape> shape = std::unique_ptr<btStaticPlaneShape>(new btStaticPlaneShape(bt_normal, w));
	return shape;
}

std::unique_ptr<btSphereShape> cWorld::BuildSphereShape(double radius) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	std::unique_ptr<btSphereShape> shape = std::unique_ptr<btSphereShape>(new btSphereShape(static_cast<btScalar>(scale * radius)));
	return shape;
}

tVector cWorld::GetBoxSize(const cSimBox* box) const
{
	const btBoxShape* box_shape = reinterpret_cast<btBoxShape*>(box->GetCollisionShape().get());
	btVector3 half_len = box_shape->getHalfExtentsWithMargin();
	double scale = GetScale();
	return tVector(half_len[0] * 2, half_len[1] * 2, half_len[2] * 2, 0) / scale;
}

double cWorld::GetCapsuleHeight(const cSimCapsule* cap) const
{
	const btCapsuleShape* shape = reinterpret_cast<btCapsuleShape*>(cap->GetCollisionShape().get());
	double scale = GetScale();
	double h = shape->getHalfHeight() * 2;
	h /= scale;
	return h;
}

double cWorld::GetCapsuleRadius(const cSimCapsule* cap) const
{
	const btCapsuleShape* shape = reinterpret_cast<btCapsuleShape*>(cap->GetCollisionShape().get());
	double scale = GetScale();
	double r = shape->getRadius();
	r /= scale;
	return r;
}

tVector cWorld::GetPlaneCoeffs(const cSimPlane* plane) const
{
	const btStaticPlaneShape* shape = reinterpret_cast<btStaticPlaneShape*>(plane->GetCollisionShape().get());
	double scale = GetScale();
	btVector3 n = shape->getPlaneNormal();
	btScalar c = shape->getPlaneConstant();
	return tVector(n[0], n[1], n[2], c / scale);
}

double cWorld::GetSphereRadius(const cSimSphere* sphere) const
{
	const btSphereShape* ball_shape = reinterpret_cast<btSphereShape*>(sphere->GetCollisionShape().get());
	double r = ball_shape->getRadius();
	double scale = GetScale();
	r /= scale;
	return r;
}

void cWorld::CalcRotationRevolute(const cJoint* joint, tVector& out_axis, double& out_theta) const
{
	btHingeConstraint* hinge = reinterpret_cast<btHingeConstraint*>(joint->GetConstraintHandle().mCons);

	double ref_theta = joint->GetRefTheta();
	out_theta = hinge->getHingeAngle();
	out_theta = -out_theta; // theta flipped
	out_theta -= ref_theta;
	out_axis = joint->GetAxisRel();
}

double cWorld::CalcDisplacementPrismatic(const cJoint* joint) const
{
	btSliderConstraint* prismatic = reinterpret_cast<btSliderConstraint*>(joint->GetConstraintHandle().mCons);
	double scale = GetScale();
	double delta = prismatic->getLinearPos();
	delta /= scale;
	return delta;
}

void cWorld::CalcRotationSpherical(const cJoint* joint, tVector& out_axis, double& out_theta) const
{
	Eigen::VectorXd pose;
	joint->BuildPose(pose);
	assert(pose.size() == 4);
	tQuaternion q = cMathUtil::VecToQuat(pose);
	cMathUtil::QuaternionToAxisAngle(q, out_axis, out_theta);
}

tVector cWorld::GetManifoldPtA(const btManifoldPoint& manifold_pt) const
{
	double scale = GetScale();
	const btVector3& contact_pt = manifold_pt.getPositionWorldOnA();
	return tVector(contact_pt[0], contact_pt[1], contact_pt[2], 0) / scale;
}

tVector cWorld::GetManifoldPtB(const btManifoldPoint& manifold_pt) const
{
	double scale = GetScale();
	const btVector3& contact_pt = manifold_pt.getPositionWorldOnB();
	return tVector(contact_pt[0], contact_pt[1], contact_pt[2], 0) / scale;
}

std::unique_ptr<btDiscreteDynamicsWorld>& cWorld::GetInternalWorld()
{
	return mSimWorld;
}


void cWorld::ClearConstraints()
{
	for (int i = GetNumConstriants() - 1; i >= 0; i--)
	{
		btTypedConstraint* constraint = mSimWorld->getConstraint(i);
		mSimWorld->removeConstraint(constraint);
		delete constraint;
	}
}

int cWorld::GetNumConstriants() const
{
	return static_cast<int>(mSimWorld->getNumConstraints());
}

cWorld::tConstraintHandle cWorld::AddRevoluteConstraint(cSimObj* obj0, cSimObj* obj1, const tJointParams& params)
{
	btScalar scale = static_cast<btScalar>(GetScale());
	const tVector& euler0 = params.mEulerAngles0;
	const tVector& euler1 = params.mEulerAngles1;

	btTransform anchor0_t;
	anchor0_t.setIdentity();
	btVector3 anchor0 = scale * btVector3(static_cast<btScalar>(params.mAnchor0[0]),
								static_cast<btScalar>(params.mAnchor0[1]),
								static_cast<btScalar>(params.mAnchor0[2]));
	anchor0_t.setOrigin(anchor0);
	anchor0_t.getBasis().setEulerZYX(static_cast<btScalar>(euler0[0]), static_cast<btScalar>(euler0[1]), static_cast<btScalar>(euler0[2]));
	btTransform anchor1_t;
	anchor1_t.setIdentity();
	btVector3 anchor1 = scale * btVector3(static_cast<btScalar>(params.mAnchor1[0]),
								static_cast<btScalar>(params.mAnchor1[1]),
								static_cast<btScalar>(params.mAnchor1[2]));
	anchor1_t.setOrigin(anchor1);
	anchor1_t.getBasis().setEulerZYX(static_cast<btScalar>(euler1[0]), static_cast<btScalar>(euler1[1]), static_cast<btScalar>(euler1[2]));

	btHingeConstraint* cons = nullptr;
	cons = new btHingeConstraint(*(obj0->GetSimBody()), *(obj1->GetSimBody()),
								anchor0_t, anchor1_t);

	// bullet limits are flipped
	cons->setLimit(-static_cast<btScalar>(params.mLimHigh[0]), -static_cast<btScalar>(params.mLimLow[0]));
	mSimWorld->addConstraint(cons, !params.mEnableAdjacentCollision);

	tConstraintHandle handle;
	handle.mCons = cons;
	return handle;
}

cWorld::tConstraintHandle cWorld::AddPrismaticConstraint(cSimObj* obj0, cSimObj* obj1, const tJointParams& params)
{
	const tVector& euler0 = params.mEulerAngles0;
	const tVector& euler1 = params.mEulerAngles1;

	btTransform anchor0_t;
	anchor0_t.setIdentity();
	btScalar scale = static_cast<btScalar>(GetScale());
	btVector3 anchor0 = scale * btVector3(static_cast<btScalar>(params.mAnchor0[0]),
								static_cast<btScalar>(params.mAnchor0[1]),
								static_cast<btScalar>(params.mAnchor0[2]));
	anchor0_t.setOrigin(anchor0);
	anchor0_t.getBasis().setEulerZYX(static_cast<btScalar>(euler0[0]), static_cast<btScalar>(euler0[1]), static_cast<btScalar>(euler0[2]));
	btTransform anchor1_t;
	anchor1_t.setIdentity();
	btVector3 anchor1 = scale * btVector3(static_cast<btScalar>(params.mAnchor1[0]),
								static_cast<btScalar>(params.mAnchor1[1]),
								static_cast<btScalar>(params.mAnchor1[2]));
	anchor1_t.setOrigin(anchor1);
	anchor1_t.getBasis().setEulerZYX(static_cast<btScalar>(euler1[0]), static_cast<btScalar>(euler1[1]), static_cast<btScalar>(euler1[2]));

	btSliderConstraint* cons = nullptr;
	cons = new btSliderConstraint(*(obj0->GetSimBody()), *(obj1->GetSimBody()),
									anchor0_t, anchor1_t, true);

	cons->setLowerLinLimit(static_cast<btScalar>(scale * params.mLimLow[0]));
	cons->setUpperLinLimit(static_cast<btScalar>(scale * params.mLimHigh[0]));
	cons->setLowerAngLimit(0.0f);
	cons->setUpperAngLimit(0.0f);
	mSimWorld->addConstraint(cons, !params.mEnableAdjacentCollision);

	tConstraintHandle handle;
	handle.mCons = cons;
	return handle;
}

cWorld::tConstraintHandle cWorld::AddSphericalConstraint(cSimObj* obj0, cSimObj* obj1, const tJointParams& params)
{
	btScalar scale = static_cast<btScalar>(GetScale());
	const tVector& euler0 = params.mEulerAngles0;
	const tVector& euler1 = params.mEulerAngles1;

	btTransform anchor0_t;
	anchor0_t.setIdentity();
	btVector3 anchor0 = scale * btVector3(static_cast<btScalar>(params.mAnchor0[0]),
								static_cast<btScalar>(params.mAnchor0[1]),
								static_cast<btScalar>(params.mAnchor0[2]));
	anchor0_t.setOrigin(anchor0);
	anchor0_t.getBasis().setEulerZYX(static_cast<btScalar>(euler0[0]), static_cast<btScalar>(euler0[1]), static_cast<btScalar>(euler0[2]));
	btTransform anchor1_t;
	anchor1_t.setIdentity();
	btVector3 anchor1 = scale * btVector3(static_cast<btScalar>(params.mAnchor1[0]),
								static_cast<btScalar>(params.mAnchor1[1]),
								static_cast<btScalar>(params.mAnchor1[2]));
	anchor1_t.setOrigin(anchor1);
	anchor1_t.getBasis().setEulerZYX(static_cast<btScalar>(euler1[0]), static_cast<btScalar>(euler1[1]), static_cast<btScalar>(euler1[2]));

	btGeneric6DofSpring2Constraint* cons = nullptr;
	cons = new btGeneric6DofSpring2Constraint(*(obj0->GetSimBody()), *(obj1->GetSimBody()), anchor0_t, anchor1_t);

	cons->setLinearUpperLimit(btVector3(0, 0, 0));
	cons->setLinearLowerLimit(btVector3(0, 0, 0));

	// bullet limits are flipped
	cons->setAngularUpperLimit(-btVector3(static_cast<btScalar>(params.mLimLow[0]), 
											static_cast<btScalar>(params.mLimLow[1]), 
											static_cast<btScalar>(params.mLimLow[2])));
	cons->setAngularLowerLimit(-btVector3(static_cast<btScalar>(params.mLimHigh[0]), 
											static_cast<btScalar>(params.mLimHigh[1]), 
											static_cast<btScalar>(params.mLimHigh[2])));
	mSimWorld->addConstraint(cons, !params.mEnableAdjacentCollision);

	tConstraintHandle handle;
	handle.mCons = cons;
	return handle;
}

cWorld::ePlaneCons cWorld::GetPlaneCons(eSimMode sim_mode) const
{
	cWorld::ePlaneCons cons = cWorld::ePlaneConsXY;
	switch (sim_mode)
	{
	case cWorld::eSimMode2D:
		cons = cWorld::ePlaneConsXY;
		break;
	case cWorld::eSimMode3D:
		cons = cWorld::ePlaneConsNone;
		break;
	default:
		break;
	}
	return cons;
}