#include "kinematics_time_warper.h"
#include <utility>
#include "boost/numeric/conversion/cast.hpp"
#include "math_utils.h"

namespace kinematics {

// public func.

TimeWarper::TimeWarper()
    :original_motion_sequence_(new math::SpatialTemporalVector6d_t),
    hard_constraint_coll_(new TimeWarpHardConstraintColl_t),
    time_step_(double{0.0}),
    min_time_step_(double{0.0}),
    max_time_step_(double{0.0})
{
}

TimeWarper::~TimeWarper()
{
}

double TimeWarper::time_step() const
{
    return time_step_;
}

double TimeWarper::min_time_step() const
{
    return min_time_step_;
}

double TimeWarper::max_time_step() const
{
    return max_time_step_;
}

void TimeWarper::Configure(
        const math::SpatialTemporalVector6d_t &original_motion_sequence,
        const double time_step,
        const double min_time_step,
        const double max_time_step
        )
{
    *original_motion_sequence_ = original_motion_sequence;
    time_step_ = time_step;
    min_time_step_ = min_time_step;
    max_time_step_ = max_time_step;
}

math::SpatialTemporalVector6d_t TimeWarper::ComputeWarpedMotion(
        const TimeWarpHardConstraintColl_t &hard_constraint_coll
        )
{
    // TO DO
    *hard_constraint_coll_ = hard_constraint_coll;
	
	
	
	math::SpatialTemporalVector6d_t new_motion_sequence = *original_motion_sequence_;


	int total_frame = (*original_motion_sequence_).temporal_size() - 1;
	double total_sec = hard_constraint_coll[2].play_second / hard_constraint_coll[2].frame_idx;
	int change_frame = hard_constraint_coll[1].play_second / total_sec; 
	int offset_frame = hard_constraint_coll[1].frame_idx;

	int old_frame_number = (total_frame - change_frame);
	int new_frame_number = (total_frame - offset_frame);
	double _time = double(new_frame_number) / double(old_frame_number);

	for (int frame = 0; frame < total_frame; frame++) { // if start in change_frame

		int _frame; // offset the frame 
		if (frame >= change_frame) { 
			_frame = frame - change_frame;
			_time = double(new_frame_number) / double(old_frame_number);
		}
		else { 
			_frame = frame; 
			_time = offset_frame / double(change_frame);
		}
		for (int skeleton_bone = 0; skeleton_bone < (*original_motion_sequence_).spatial_size(); skeleton_bone++) {
			if (frame == change_frame) {
				new_motion_sequence.set_element(skeleton_bone, hard_constraint_coll[1].frame_idx, (*original_motion_sequence_).element(skeleton_bone, change_frame));
			}
			
			double x0_frame = ((_frame) * _time);
			if (x0_frame == 0) { x0_frame += 0.001; }
			double x1_frame = ((_frame+1)* _time);

			int number_of_interpolation = int(x1_frame) - int(x0_frame); // interploate the "bigger" interval
			
			if (number_of_interpolation == 0)	 {
				if ((change_frame > offset_frame && frame < change_frame) ||
					(change_frame < offset_frame && frame > change_frame)
					)
				number_of_interpolation++;
			}						
				// in some case will overlap 2 slot, so need to checks
			    // use int() it's not a good way but can judge overlap how many integer
				math::Quaternion_t q_frame1 =
					math::ComputeQuaternionXyz(
					math::ToRadian((*original_motion_sequence_).element(skeleton_bone, frame )[0]),
					math::ToRadian((*original_motion_sequence_).element(skeleton_bone, frame )[1]),
					math::ToRadian((*original_motion_sequence_).element(skeleton_bone, frame )[2])
					);
				math::Quaternion_t q_frame2 =
					math::ComputeQuaternionXyz(
					math::ToRadian((*original_motion_sequence_).element(skeleton_bone, frame+1)[0]),
					math::ToRadian((*original_motion_sequence_).element(skeleton_bone, frame+1)[1]),
					math::ToRadian((*original_motion_sequence_).element(skeleton_bone, frame+1)[2])
					);

				std::vector<double> q1(4);
				std::vector<double> q2(4);
				q1[0] = q_frame1.w(); q1[1] = q_frame1.x(); q1[2] = q_frame1.y(); q1[3] = q_frame1.z();
				q2[0] = q_frame2.w(); q2[1] = q_frame2.x(); q2[2] = q_frame2.y(); q2[3] = q_frame2.z();

				if (
					(	
						!(q1[0] * q2[0] > 0)
						&& 
						!(q1[1] * q2[1] > 0)
					)
					&&
					(
						!(q1[2] * q2[2] > 0)
						&&
						!(q1[3] * q2[3] > 0)
					)
				   ) 
				{
					q_frame1.w() = -q_frame1.w();
					q_frame1.z() = -q_frame1.z();
					q_frame1.y() = -q_frame1.y();
					q_frame1.x() = -q_frame1.x();
				}


				math::Quaternion_t q_linear_interpolation;
				math::Vector6d_t new_dof;
				for (int interpolation_add = 0; interpolation_add < number_of_interpolation; interpolation_add++) {
					int x_frame;
					if (
						(change_frame > offset_frame && frame <= change_frame)
						||
						(change_frame < offset_frame && frame > change_frame)
					   )   
					{
						x_frame = int(x1_frame);
					}
					else {
						if (x0_frame == int(x0_frame)) { x0_frame++; }
						x_frame = std::ceil(x0_frame) + interpolation_add;
					}

					if (x_frame == offset_frame) break;
					
					q_linear_interpolation.w() = q_frame1.w() + (x_frame - x0_frame)*(1.0 / (x1_frame - x0_frame))*(q_frame2.w() - q_frame1.w());
					q_linear_interpolation.z() = q_frame1.z() + (x_frame - x0_frame)*(1.0 / (x1_frame - x0_frame))*(q_frame2.z() - q_frame1.z());
					q_linear_interpolation.y() = q_frame1.y() + (x_frame - x0_frame)*(1.0 / (x1_frame - x0_frame))*(q_frame2.y() - q_frame1.y());
					q_linear_interpolation.x() = q_frame1.x() + (x_frame - x0_frame)*(1.0 / (x1_frame - x0_frame))*(q_frame2.x() - q_frame1.x()); 


					new_dof[0] = math::ToDegree(math::ComputeEulerAngleXyz(math::ComputeRotMat(q_linear_interpolation))[0]);
					new_dof[1] = math::ToDegree(math::ComputeEulerAngleXyz(math::ComputeRotMat(q_linear_interpolation))[1]);
					new_dof[2] = math::ToDegree(math::ComputeEulerAngleXyz(math::ComputeRotMat(q_linear_interpolation))[2]);
					if (skeleton_bone != 0) {
						new_dof[3] = 0;
						new_dof[4] = 0;
						new_dof[5] = 0;
					}
					else {
						math::Vector3d_t v_frame1(
							(*original_motion_sequence_).element(skeleton_bone, frame)[3],
							(*original_motion_sequence_).element(skeleton_bone, frame)[4],
							(*original_motion_sequence_).element(skeleton_bone, frame)[5]
							);
						math::Vector3d_t v_frame2(
							(*original_motion_sequence_).element(skeleton_bone, frame + 1)[3],
							(*original_motion_sequence_).element(skeleton_bone, frame + 1)[4],
							(*original_motion_sequence_).element(skeleton_bone, frame + 1)[5]
							);


						math::Vector3d_t v_new_frame;
						v_new_frame = v_frame1 + (x_frame - x0_frame)*(1.0 / (x1_frame - x0_frame))*(v_frame2 - v_frame1);
						new_dof[3] = v_new_frame[0];
						new_dof[4] = v_new_frame[1];
						new_dof[5] = v_new_frame[2];
										
					}

					if (frame >= change_frame)  { // for offset_ case
						new_motion_sequence.set_element(skeleton_bone, x_frame + hard_constraint_coll[1].frame_idx, new_dof);
					}
					else {
						new_motion_sequence.set_element(skeleton_bone, x_frame , new_dof);
					}

			
				}
		}



	}

	return new_motion_sequence;
}

// protected func.

// private func.

} // namespace kinematics {
