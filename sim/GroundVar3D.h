#pragma once

#include "sim/Ground.h"
#include "sim/TerrainGen3D.h"

class cGroundVar3D : public cGround
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	struct tSlab : public cSimObj
	{
		static const double gGridSpacingX;
		static const double gGridSpacingZ;

		tSlab();
		~tSlab();

		void Clear();
		void Init(const std::shared_ptr<cWorld>& world, double friction);
		size_t GetNumVerts() const;
		tVector GetScaling() const;
		tVector GetVertex(int i, int j) const;
		int GetFlags(int i, int j) const;
		int CalcDataIdx(int i, int j) const;

		bool ContainsPos(const tVector& pos) const;
		tVector CalcGridCoord(const tVector& pos) const;
		tVector ClampCoord(const tVector& coord) const;
		double SampleHeight(const tVector& pos, bool& out_valid_sample) const;

		bool IsValid() const;

		std::vector<float> mData;
		std::vector<int> mFlags;
		tVector mMin;
		tVector mMax;
		int mResX;
		int mResZ;
	};
	
	static bool ParseParamsJson(const Json::Value& json, Eigen::VectorXd& out_params);

	cGroundVar3D();
	virtual ~cGroundVar3D();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params,
					const tVector& bound_min, const tVector& bound_max);
	virtual void Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max);
	virtual void Clear();

	virtual double SampleHeight(const tVector& pos) const;
	virtual double SampleHeight(const tVector& pos, bool& out_valid_sample) const;
	virtual void SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const;

	virtual tVector GetVertex(int i, int j) const;
	virtual void LocateSlab(int i, int j, int& out_slab_idx, int& out_slab_i, int& out_slab_j) const;
	virtual tVector GetPos() const;
	virtual void GetRes(int& out_x_res, int& out_z_res) const;

	virtual void SetTerrainFunc(cTerrainGen3D::tTerrainFunc func);
	virtual bool HasSimBody() const;

	virtual int GetSlabID(int s) const;
	virtual const tSlab& GetSlab(int s) const;
	virtual eClass GetGroundClass() const;
	virtual int GetNumSlabs() const;

	virtual void CalcAABB(tVector& out_min, tVector& out_max) const;

	virtual bool Output(const std::string& out_file) const;

protected:
	static const int gNumSlabs = 4;

	cTerrainGen3D::tTerrainFunc mTerrainFunc;

	tSlab mSlabs[gNumSlabs];
	int mSlabOrder[gNumSlabs];

	virtual void ResetParams();
	virtual int GetBlendParamSize() const;
	virtual void ClearSlabs();
	virtual void ClearSlab(int s);
	virtual void InitSlabs(const tVector& bound_min, const tVector& bound_max);

	virtual tSlab& GetSlab(int s);
	virtual void BuildSlab(int s, const tVector& bound_min, const tVector& bound_max);
	virtual void BuildSlabHeighData(const tVector& bound_min, const tVector& bound_max, 
									std::vector<float>& out_data, std::vector<int>& out_flags);
	virtual void GetBounds(tVector& bound_min, tVector& bound_max) const;

	virtual void ClearSlabData(int s);
	virtual void FillSlabBorders(int s);
};
