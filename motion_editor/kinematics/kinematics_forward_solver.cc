#include "kinematics_forward_solver.h"
#include <algorithm>
#include "math_utils.h"
#include "acclaim_skeleton.h"
#include "acclaim_motion.h"
#include "helper_forward_kinematics.h"
#include "kinematics_artic_idx.h"
#include "kinematics_pose.h"

namespace kinematics {

// public func.

ForwardSolver::ForwardSolver()
    :skeleton_(nullptr),
    motion_(nullptr),
    artic_path_(new ArticIdxColl_t),
    helper_fk_(new helper::ForwardKinematics)
{
}

ForwardSolver::~ForwardSolver()
{
}

std::shared_ptr<acclaim::Skeleton> ForwardSolver::skeleton() const
{
    return skeleton_;
}

std::shared_ptr<acclaim::Motion> ForwardSolver::motion() const
{
    return motion_;
}

void ForwardSolver::set_skeleton(const std::shared_ptr<acclaim::Skeleton> &skeleton)
{
    skeleton_ = skeleton;
    helper_fk_->set_skeleton(skeleton_);
}

void ForwardSolver::set_motion(const std::shared_ptr<acclaim::Motion> &motion)
{
    motion_ = motion;
	helper_fk_->set_motion(motion_);
}

void ForwardSolver::ConstructArticPath()
{

    helper_fk_->ConstructArticPath();
}

PoseColl_t ForwardSolver::ComputeSkeletonPose(const int32_t frame_idx)
{
    return this->ComputeSkeletonPose(motion_->joint_spatial_pos(frame_idx));
}

/*
namespace kinematics {
typedef std::vector<ArticIdx> ArticIdxColl_t;
typedef std::vector<Pose> PoseColl_t;
} // namespace kinematics {
*/

PoseColl_t ForwardSolver::ComputeSkeletonPose(const math::Vector6dColl_t &joint_spatial_pos)
{
    // TO DO

	PoseColl_t pose(joint_spatial_pos.size());
	math::Vector3d_t v0_s(joint_spatial_pos[0].linear_vector());
	math::RotMat3d_t Iden;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
		if (i == j) Iden(i, j) = 1;
		else Iden(i, j) = 0;
	}
	pose[0].set_rotation(Iden);


	math::RotMat3d_t delta_mat;
	math::RotMat3d_t local_mat;
	math::RotMat3d_t totla_local_mat;
	math::Vector3d_t v;
	math::Vector3d_t translation;


	pose[0].set_start_pos(v0_s);
	pose[0].set_end_pos(v0_s);
	for (int bone = 1; bone < skeleton_->bone_num(); bone++) {
		delta_mat = math::ComputeRotMatXyz(math::ToRadian(joint_spatial_pos[bone].angular_vector()));
		for (int i = 0; i < 3; i++) {
			for(int j = 0; j < 3; j++) {
				local_mat(j, i) = skeleton_->bone_ptr(bone)->rot_parent_current[i][j];
			}
		}
		
		math::RotMat3d_t rot_par = math::ComputeRotMat(*skeleton_->bone_ptr(bone)->rot_parent_current).transpose();
		totla_local_mat = local_mat * delta_mat;


		int _parent = skeleton_->bone_idx(skeleton_->bone_ptr(bone)->parent->name);
		if (_parent == 0) {
			pose[_parent].set_rotation(math::ComputeRotMatXyz(math::ToRadian(joint_spatial_pos[0].angular_vector())));
		}
		pose[bone].set_rotation(pose[_parent].rotation() * totla_local_mat);

		v = skeleton_->bone_ptr(bone)->dir * skeleton_->bone_ptr(bone)->length;

		pose[bone].set_start_pos(pose[_parent].end_pos());

		math::Quaternion_t total_transform = math::ComputeQuaternionXyz(math::ComputeEulerAngleXyz(pose[bone].rotation())[0],
																		math::ComputeEulerAngleXyz(pose[bone].rotation())[1],
																		math::ComputeEulerAngleXyz(pose[bone].rotation())[2]);
		
		math::Quaternion_t quaternion_vector(0, v[0], v[1], v[2]);
		
		quaternion_vector = total_transform * (quaternion_vector * total_transform.inverse());

		math::Vector3d_t real_vector(quaternion_vector.vec());

		translation = real_vector + pose[bone].start_pos();
		/*Bonus
			//in matrix form
			translation = pose[bone].rotation() * v + pose[bone].start_pos();
		*/
		

		pose[bone].set_end_pos(translation);
	}


	return pose;
	
	//return helper_fk_->ComputeSkeletonPose(joint_spatial_pos);
}

// protected func.

// private func.

} // namespace kinematics {
