#include "utilities/transforms.hpp"
#include <cmath>
#include <optional>
#include <utility>

namespace {
    Eigen::Isometry3d form_TF_rotation(double q, int axis){
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R;
    if (axis == 1) {R = Eigen::AngleAxisd(q, Eigen::Vector3d::UnitX()).toRotationMatrix();}
    else if (axis == 2) {R = Eigen::AngleAxisd(q, Eigen::Vector3d::UnitY()).toRotationMatrix();}
    else if (axis == 3) {R = Eigen::AngleAxisd(q, Eigen::Vector3d::UnitZ()).toRotationMatrix();}
    else {R = Eigen::Matrix3d::Identity();}

    T.linear() = R;
    T.translation() = Eigen::Vector3d::Zero();
    return T;
}

Eigen::Isometry3d TF(double roll, double pitch, double yaw, double x, double y, double z, double q, int axis){
    Eigen::Isometry3d T_fixed = form_TF_fixed(roll, pitch, yaw, x, y, z);
    Eigen::Isometry3d T_rot = form_TF_rotation(q, axis);

    return T_fixed * T_rot;
}
}

struct LegAngles{
    double hip_yaw;
    double hip_pitch;
    double knee_pitch;
};

struct JointLimit{
    double min;
    double max;
    bool contains(double q) const {return q >= min && q <= max;}
};

struct LegLimit{
    JointLimit hip_yaw;
    JointLimit hip_pitch;
    JointLimit knee_pitch;
};

struct JointOrigin {
    double roll, pitch, yaw;
    double x, y, z;
    int axis;
};

struct LegParams {
    JointOrigin hip_yaw;
    JointOrigin hip_pitch;
    JointOrigin knee_pitch;
    JointOrigin ankle;

    double PHI1;
    double PSI;

    LegLimit limits;
};

enum class Side {Left, Right};


LegParams params_for(Side side){
    if (side == Side::Left){
        return LegParams{
            // {roll,    pitch,   yaw,      x,       y,        z,       axis}
            /*hip_yaw   */ {0.0,     1.5708,  0.0,     0.125,   0.02088, -0.0125,  1},
            /*hip_pitch */ {1.5708,  0.0,    -1.5708,  0.074,   0.0,      0.0,     2},
            /*knee_pitch*/ {3.14159, 0.0,     0.0,     0.28154, 0.0,      0.05165, 2},
            /*ankle     */ {-1.5708, 1.10916, 0.0,     0.29957, 0.0,      0.01196, 0},
            /*PHI1*/ M_PI / 2 + std::atan2(0.05165, 0.28154),
            /*PSI */ 0.221341,
            /*limits*/ {{-1.57, 0.5}, {-0.18, 2.0}, {0.0, 2.57}}
        };
    } else {  // Side::Right — mirrored origins from the URDF
        return LegParams{
            /*hip_yaw   */ {3.14159, 1.5708,  0.0,    -0.125,   0.02088, -0.0125,  1},
            /*hip_pitch */ {1.5708,  0.0,     1.5708,  0.074,   0.0,      0.0,     2},
            /*knee_pitch*/ {0.0,     0.0,     0.0,     0.28154, 0.0,     -0.05165, 2},
            /*ankle     */ {-1.5708, 1.10916, 0.0,     0.29977, 0.0,      0.01196, 0},
            /*PHI1*/ 0.0,   // PLACEHOLDER — DERIVE by probing right-leg FK (not 0, not the left value)
            /*PSI */ 0.0,   // PLACEHOLDER — DERIVE by probing right-leg FK
            /*limits*/ {{-0.5, 1.57}, {-2.0, 0.18}, {0.0, 2.57}}
        };
    }
}


class Leg{
    public:
        explicit Leg(Side side) : side_(side), params_(params_for(side)){
            if (side_ == Side::Left) {
                limits_ = {{-1.57, 0.5}, {-0.18, 2.0}, {0, 2.57}};
            } else {
                // TODO: Check the flip value for the knee since it is belt
                // driven and this may be correct for sim but not physical hardware
                // (Do later once testing on hardware)
                limits_ = {{-0.5, 1.57}, {-2.0, 0.18}, {0, 2.57}};
            }
        }

        Eigen::Vector3d FK(double q_yaw, double q_pitch, double q_knee) const {
            Eigen::Isometry3d T_hip_yaw   = TF(0,       1.5708, 0,  0.125,   0.02088, -0.0125, q_yaw,   1);
            Eigen::Isometry3d T_hip_pitch = TF(1.5708,  0,     -1.5708, 0.074, 0.0,  0.0,  q_pitch, 2);
            Eigen::Isometry3d T_knee      = TF(3.14159, 0,      0,  0.28154, 0.0,    0.05165, q_knee,  2);
            Eigen::Isometry3d T_ankle     = form_TF_fixed(-1.5708, 1.10916, 0, 0.29957, 0.0, 0.01196);
            Eigen::Isometry3d T = T_hip_yaw * T_hip_pitch * T_knee * T_ankle;
            return T.translation();
        }

        std::optional<LegAngles> IK(Eigen::Vector3d p_body, bool elbow_up) const {
            Eigen::Isometry3d body_2_hip = form_TF_fixed(0.0, 1.5708, 0.0, 0.125, 0.02088, -0.0125);
            Eigen::Vector3d p_hip = body_2_hip.inverse() * p_body;

            // Natural (in-limits) yaw branch: atan2(z,y) + pi, wrapped to (-pi, pi].
            double hip_yaw = std::remainder(std::atan2(p_hip.z(), p_hip.y()) + M_PI, 2.0 * M_PI);

            // Undo the yaw rotation, then shift to the hip-pitch origin -> planar frame.
            Eigen::Vector3d p_planar = Eigen::AngleAxisd(-hip_yaw, Eigen::Vector3d::UnitX()) * p_hip
                                    - Eigen::Vector3d(0.074, 0.0, 0.0);
            auto p_ik = planar_ik(p_planar.x(), p_planar.y(), elbow_up);
            if (!p_ik){
                return std::nullopt;
            }

            LegAngles angles;
            angles.hip_yaw = hip_yaw;
            angles.hip_pitch = p_ik->first;
            angles.knee_pitch = p_ik->second;

            if (!within_limits(angles)){
                return std::nullopt;
            } 
            return angles;
        }

        LegAngles get_joint_angles() const {
            return this->joint_angles;
        }

        void set_joint_angles(LegAngles q){
            this->joint_angles = q;
        }

        bool within_limits(LegAngles q) const {
            return limits_.hip_yaw.contains(q.hip_yaw)
                   && limits_.hip_pitch.contains(q.hip_pitch)
                   && limits_.knee_pitch.contains(q.knee_pitch);
        }

    private:
        double L1_ = hypot(0.28154, 0.05165);
        double L2_ = hypot(0.29957, 0.01196);
        // REMOVE PHI and PSI
        double PHI1_ = M_PI/2 + atan2(0.05165, 0.28154);
        double PSI_ = 0.221341;

        Side side_;

        // Struct to hold current joint angles of the leg
        LegAngles joint_angles {0, 0, 0};
        LegLimit limits_;
        LegParams params_;


        std::optional<std::pair<double, double>> planar_ik (double x, double y, bool elbow_up) const {
            double r = hypot(x, y);
            if (r > L1_ + L2_ || r < std::abs(L1_ - L2_)) {
                return std::nullopt;
            }

            double cos_knee_pitch = -(pow(L1_, 2) + pow(L2_, 2) - pow(x,2) - pow(y,2))/(2*L1_*L2_);
            double elbow_sign = elbow_up ? 1.0 : -1.0;

            double D = cos_knee_pitch;
            double C = sqrt(1-pow(D,2)) * elbow_sign;
            double q2 = atan2(C, D);

            double gamma = atan2(y,x);
            double cos_beta = (pow(L1_, 2) + pow(x,2) + pow(y,2) - pow(L2_,2))/(2*L1_*sqrt(pow(x,2) + pow(y,2)));
            double B = cos_beta;

            double sin_beta = sqrt(1-pow(B,2)) * elbow_sign;
            double G = sin_beta;

            double BETA = atan2(G,B);
            double q1 = gamma - BETA;

            double hip_pitch = q1 + PHI1_;
            double knee_pitch = PSI_ - q2;

            return std::make_pair(hip_pitch, knee_pitch);
}
};