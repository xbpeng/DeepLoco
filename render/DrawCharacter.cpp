#include "DrawCharacter.h"
#include "DrawKinTree.h"
#include "render/DrawUtil.h"
#include <iostream>

void cDrawCharacter::Draw(const cCharacter& character, double link_width, const tVector& fill_col, const tVector& line_col)
{
	const Eigen::MatrixXd& joint_mat = character.GetJointMat();
	const Eigen::VectorXd& pose = character.GetPose();
	cDrawKinTree::Draw(joint_mat, pose, link_width, fill_col, line_col);
}

void cDrawCharacter::DrawShape(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col)
{
	cKinTree::eBodyShape shape = static_cast<cKinTree::eBodyShape>((int) def[cKinTree::eDrawShapeShape]);
	switch (shape)
	{
	case cKinTree::eBodyShapeBox:
		DrawShapeBox(character, def, fill_tint, line_col);
		break;
	case cKinTree::eBodyShapeCapsule:
		DrawShapeCapsule(character, def, fill_tint, line_col);
		break;
	case cKinTree::eBodyShapeNULL:
		break;
	default:
		assert(false); // unsupported draw shape
		break;
	}
}

void cDrawCharacter::DrawHeading(const cCharacter& character, double arrow_size, const tVector& arrow_col, const tVector& offset)
{
	tMatrix origin_trans = character.BuildOriginTrans();
	tMatrix heading_trans = cMathUtil::InvRigidMat(origin_trans);
	tVector root_pos = character.GetRootPos();
	cDrawUtil::SetColor(arrow_col);

	tVector start = tVector(0.25, root_pos[1], 0, 0);
	tVector end = tVector(1, root_pos[1], 0, 0);

	cDrawUtil::PushMatrix();
	cDrawUtil::GLMultMatrix(heading_trans);
	cDrawUtil::DrawArrow2D(start, end, arrow_size);
	cDrawUtil::PopMatrix();
}

void cDrawCharacter::DrawShapeBox(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col)
{
	double theta = 0;
	tVector euler = cKinTree::GetDrawShapeAttachTheta(def);
	int parent_joint = cKinTree::GetDrawShapeParentJoint(def);
	tVector attach_pt = cKinTree::GetDrawShapeAttachPt(def);
	tVector col = cKinTree::GetDrawShapeColor(def);
	tVector size = tVector(def[cKinTree::eDrawShapeParam0], def[cKinTree::eDrawShapeParam1], def[cKinTree::eDrawShapeParam2], 0);
	col = col.cwiseProduct(fill_tint);

	tMatrix world_trans = character.BuildJointWorldTrans(parent_joint);
	
	cDrawUtil::PushMatrix();
	cDrawUtil::GLMultMatrix(world_trans);
	cDrawUtil::Translate(attach_pt);
	cDrawUtil::Rotate(euler);

	cDrawUtil::SetColor(col);
	cDrawUtil::DrawBox(tVector::Zero(), size, cDrawUtil::eDrawSolid);

	if (line_col[3] > 0)
	{
		cDrawUtil::SetColor(line_col);
		cDrawUtil::DrawBox(tVector::Zero(), size, cDrawUtil::eDrawWire);
	}

	cDrawUtil::PopMatrix();
}

void cDrawCharacter::DrawShapeCapsule(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col)
{
	double theta = 0;
	tVector axis = tVector(0, 0, 1, 0);
	cKinTree::GetDrawShapeRotation(def, axis, theta);
	int parent_joint = cKinTree::GetDrawShapeParentJoint(def);
	tVector attach_pt = cKinTree::GetDrawShapeAttachPt(def);
	tVector col = cKinTree::GetDrawShapeColor(def);
	tVector size = tVector(def[cKinTree::eDrawShapeParam0], def[cKinTree::eDrawShapeParam1], 0, 0);
	col = col.cwiseProduct(fill_tint);

	tMatrix world_trans = character.BuildJointWorldTrans(parent_joint);

	cDrawUtil::PushMatrix();
	cDrawUtil::GLMultMatrix(world_trans);
	cDrawUtil::Translate(attach_pt);
	cDrawUtil::Rotate(theta, axis);

	cDrawUtil::SetColor(col);
	cDrawUtil::DrawCapsule(size[0], size[1], cDrawUtil::eDrawSolid);

	if (line_col[3] > 0)
	{
		cDrawUtil::SetColor(line_col);
		cDrawUtil::DrawCapsule(size[0], size[1], cDrawUtil::eDrawWire);
	}

	cDrawUtil::PopMatrix();
}