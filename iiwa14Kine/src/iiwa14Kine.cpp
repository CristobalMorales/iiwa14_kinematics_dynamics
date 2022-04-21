#include <iiwa14Kine/iiwa14Kine.h>

using namespace Eigen;

void iiwa14_kinematic::init()
{
    DH_params[0][0] = 0.0;      DH_params[1][0] = 0.0;      DH_params[2][0] = 0.0;      DH_params[3][0] = 0.0;
    DH_params[0][1] = -M_PI_2;  DH_params[1][1] = M_PI_2;   DH_params[2][1] = M_PI_2;   DH_params[3][1] = -M_PI_2;
    DH_params[0][2] = 0.2025;   DH_params[1][2] = 0.0;      DH_params[2][2] = 0.42;     DH_params[3][2] = 0.0;
    DH_params[0][3] = 0.0;      DH_params[1][3] = 0.0;      DH_params[2][3] = 0.0;      DH_params[3][3] = 0.0;

    DH_params[4][0] = 0.0;      DH_params[5][0] = 0.0;      DH_params[6][0] = 0.0;
    DH_params[4][1] = -M_PI_2;  DH_params[5][1] = M_PI_2;   DH_params[6][1] = 0.0;
    DH_params[4][2] = 0.4;      DH_params[5][2] = 0.0;      DH_params[6][2] = 0.126;
    DH_params[4][3] = 0.0;      DH_params[5][3] = 0.0;      DH_params[6][3] = 0.0;

    joint_offset[0] = 0.0;
    joint_offset[1] = 0.0;
    joint_offset[2] = 0.0;
    joint_offset[3] = 0.0;
    joint_offset[4] = 0.0;
    joint_offset[5] = 0.0;
    joint_offset[6] = 0.0;

    joint_limit_min[0] = -170*M_PI/180;
    joint_limit_min[1] = -120*M_PI/180;
    joint_limit_min[2] = -170*M_PI/180;
    joint_limit_min[3] = -120*M_PI/180;
    joint_limit_min[4] = -170*M_PI/180;
    joint_limit_min[5] = -120*M_PI/180;
    joint_limit_min[6] = -175*M_PI/180;

    joint_limit_max[0] = 170*M_PI/180;
    joint_limit_max[1] = 120*M_PI/180;
    joint_limit_max[2] = 170*M_PI/180;
    joint_limit_max[3] = 120*M_PI/180;
    joint_limit_max[4] = 170*M_PI/180;
    joint_limit_max[5] = 120*M_PI/180;
    joint_limit_max[6] = 175*M_PI/180;

    //The mass of each link.
    mass.resize(7);
    mass << 4, 4, 3, 2.7, 1.7, 1.8, 0.3;

    //Moment on inertia of each link.
    Ixyz.resize(7, 3);
    Ixyz << 0.1, 0.09, 0.02,
            0.05, 0.018, 0.044,
            0.08, 0.075, 0.01,
            0.03, 0.01, 0.029,
            0.02, 0.018, 0.005,
            0.005, 0.0036, 0.0047,
            0.001, 0.001, 0.001;

    //gravity
    g = 9.8;

    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 5, &iiwa14_kinematic::joint_state_callback, this);
}


void iiwa14_kinematic::joint_state_callback(const sensor_msgs::JointState::ConstPtr &q)
{
    VectorXd J(7);

    for(int i = 0; i < 7; i++)
        J(i) = q->position.at(i);

    current_pose = forward_kine(J, 7);
    broadcast_pose(current_pose);
}

Matrix4d iiwa14_kinematic::dh_matrix_standard(double a, double alpha, double d, double theta)
{
    Matrix4d A;
    A(3, 3) = 1.0;
    A(3, 2) = 0.0;
    A(3, 1) = 0.0;
    A(3, 0) = 0.0;

    A(0, 0) = cos(theta);
    A(0, 1) = -sin(theta)*cos(alpha);
    A(0, 2) = sin(theta)*sin(alpha);
    A(0, 3) = a * cos(theta);

    A(1, 0) = sin(theta);
    A(1, 1) = cos(theta)*cos(alpha);
    A(1, 2) = -cos(theta)*sin(alpha);
    A(1, 3) = a * sin(theta);

    A(2, 0) = 0.0;
    A(2, 1) = sin(alpha);
    A(2, 2) = cos(alpha);
    A(2, 3) = d;

    return A;
}

void iiwa14_kinematic::broadcast_pose(Matrix4d pose)
{

    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose;

    geometry_msgs::TransformStamped T = tf2::eigenToTransform(pose_affine);

    T.header.stamp = ros::Time::now();
    T.header.frame_id = "iiwa_link_0";
    T.child_frame_id = "iiwa_ee";

    pose_br.sendTransform(T);
}

//Useful Transformation function.
Matrix4d T_translation(Vector3d t)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    for (int i = 0; i < 3; i++)
        T(i, 3) = t(i);
    return T;
}

//Useful Transformation function.
Matrix4d T_rotationZ(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta);
    T(1, 0) = sin(theta);
    T(1, 1) = cos(theta);
    return T;
}

//Useful Transformation function.
Matrix4d T_rotationY(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(0, 0) = cos(theta);
    T(0, 2) = sin(theta);
    T(2, 0) = -sin(theta);
    T(2, 2) = cos(theta);
    return T;
}

//Useful Transformation function.
Matrix4d T_rotationX(double theta)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(1, 1) = cos(theta);
    T(1, 2) = -sin(theta);
    T(2, 1) = sin(theta);
    T(2, 2) = cos(theta);
    return T;
}

Matrix4d iiwa14_kinematic::forward_kine(VectorXd joint_val, int frame)
{
    Matrix4d A;
    Matrix4d T = Matrix4d::Identity(4, 4);
    T(2, 3) = 0.1575;
    for(int i = 0;i < frame; i++)
    {
        A = dh_matrix_standard(DH_params[i][0], DH_params[i][1], DH_params[i][2], (joint_val[i] + DH_params[i][3]));
        T = T * A;
    }
    return T;
}

MatrixXd iiwa14_kinematic::forward_kine_cm(VectorXd joint_val, int frame)
{
    Matrix4d T = Matrix4d::Identity(4, 4);
    double inertial_pos[7][3];
    inertial_pos[0][0] = 0.0;   inertial_pos[1][0] = 0.0003;    inertial_pos[2][0] = 0.0;   inertial_pos[3][0] = 0.0;
    inertial_pos[0][1] = -0.03; inertial_pos[1][1] = -0.059;     inertial_pos[2][1] = 0.03;  inertial_pos[3][1] = 0.067;
    inertial_pos[0][2] = 0.12;  inertial_pos[1][2] = 0.042;     inertial_pos[2][2] = 0.3345;  inertial_pos[3][2] = 0.034;

    inertial_pos[4][0] = 0.0001;    inertial_pos[5][0] = 0.0;       inertial_pos[6][0] = 0.0;
    inertial_pos[4][1] = -0.021;     inertial_pos[5][1] = 0.0006;    inertial_pos[6][1] = 0.0;
    inertial_pos[4][2] = 0.076 + 0.1845;     inertial_pos[5][2] = 0.0004;    inertial_pos[6][2] = 0.101;

    Matrix4d i1Ti = forward_kine(joint_val, frame);
    Matrix4d RotZi1 = Matrix4d::Identity(4, 4);
    Matrix4d transMat = Matrix4d::Identity(4, 4);
    if (frame >= 0)
        RotZi1 = T_rotationZ(joint_val[frame] + DH_params[frame][3]);
    
    if (frame >= 0){
        Vector3d trans;
        trans << inertial_pos[frame][0], inertial_pos[frame][1], inertial_pos[frame][2];
        transMat = T_translation(trans);
    }
    T = i1Ti * RotZi1 * transMat;
    return T;
}

MatrixXd iiwa14_kinematic::get_jacobian(VectorXd joint_val)
{
    MatrixXd jacobian(6, 7);
    Matrix4d* forw_kin_matr = new Matrix4d[8];
    Vector3d* z_i = new Vector3d[8];
    Vector3d* o_i = new Vector3d[8];
    for (int frame = 0; frame <= 7; frame++) 
    {
        forw_kin_matr[frame] = forward_kine(joint_val, frame);
        z_i[frame] = forw_kin_matr[frame].block(0,2,3,1);
        o_i[frame] = forw_kin_matr[frame].block(0,3,3,1);
    }
    for (int frame = 1; frame <= 7; frame++) 
    {
        Vector3d J_pi = z_i[frame - 1].cross(o_i[7] - o_i[frame - 1]);
        Vector3d J_oi = z_i[frame - 1];
        for (int pos = 0; pos < 3; pos++) 
        {
            jacobian(pos, frame - 1) = J_pi(pos);
            jacobian(3 + pos, frame - 1) = J_oi(pos);
        }
    }
    return jacobian;
}

MatrixXd iiwa14_kinematic::get_jacobian_cm(VectorXd joint_val, int frame)
{
    MatrixXd jacobian(6, 7);
    Matrix4d fordward_cm;
    Matrix4d* forw_kin_matr = new Matrix4d[8];
    Vector3d* z_i = new Vector3d[8];
    Vector3d* p_i = new Vector3d[8];
    fordward_cm = forward_kine_cm(joint_val, frame);
    Vector3d p_li = fordward_cm.block(0,3,3,1);
    for (int frame_aux = 0; frame_aux <= frame; frame_aux++) 
    {
        forw_kin_matr[frame_aux] = forward_kine(joint_val, frame_aux);
        z_i[frame_aux] = forw_kin_matr[frame_aux].block(0,2,3,1);
        p_i[frame_aux] = forw_kin_matr[frame_aux].block(0,3,3,1);
    }
    for (int frame_aux = 1; frame_aux <= 7; frame_aux++) 
    {
        if (frame_aux <= frame + 1)
        {
            Vector3d J_pi = z_i[frame_aux - 1].cross(p_li - p_i[frame_aux - 1]);
            Vector3d J_oi = z_i[frame_aux - 1];
            for (int pos = 0; pos < 3; pos++) 
            {
                jacobian(pos, frame_aux - 1) = J_pi(pos);
                jacobian(3 + pos, frame_aux - 1) = J_oi(pos);
            }
        }
        else
        {
            for (int pos = 0; pos < 3; pos++) 
            {
                jacobian(pos, frame_aux - 1) = 0;
                jacobian(3 + pos, frame_aux - 1) = 0;
            }
        }
    }
    return jacobian;
}

VectorXd iiwa14_kinematic::obtain_pose_vector(Matrix4d pose_matrix) {
    VectorXd pose_vector(6);
    double target_angle = acos((pose_matrix(0,0) + pose_matrix(1,1) + pose_matrix(2,2) - 1)/2);
    pose_vector(0) = pose_matrix(0, 3);
    pose_vector(1) = pose_matrix(1, 3);
    pose_vector(2) = pose_matrix(2, 3);
    if (sin(target_angle) != 0)
    {
        pose_vector(3) = (1/(2*sin(target_angle))) * (pose_matrix(2,1) - pose_matrix(1,2));
        pose_vector(4) = (1/(2*sin(target_angle))) * (pose_matrix(0,2) - pose_matrix(2,0));
        pose_vector(5) = (1/(2*sin(target_angle))) * (pose_matrix(1,0) - pose_matrix(0,1));
    }
    else
    {
        pose_vector(3) = 0;
        pose_vector(4) = 0;
        pose_vector(5) = 0;
    }
    return pose_vector;
}
VectorXd iiwa14_kinematic::inverse_kine_ite(Matrix4d pose, VectorXd joint_val)
{
    double gamma = 0.6;
    VectorXd target_pose(6);
    VectorXd current_pose(6);
    double error_var = 0.000001;
    target_pose = obtain_pose_vector(pose);
    double vari = 1;
    double pose_error = 1, pose_error_prev = 0;
    int iter_n = 0;
    while (error_var < vari)
    {
        Matrix4d current_pose_matrix = forward_kine(joint_val, 7);
        current_pose = obtain_pose_vector(current_pose_matrix);
        pose_error_prev = pose_error;
        if (check_singularity(joint_val))
            std::cout << "Is singularity?: " << check_singularity(joint_val) << std::endl;
        MatrixXd jacobian = get_jacobian(joint_val);
        MatrixXd Ide = MatrixXd::Identity(7, 7);
        MatrixXd fu = (jacobian.transpose()*jacobian + (gamma*gamma)*Ide).inverse()*jacobian.transpose();
        if (iter_n > 10000)
        {
            break;
        }
        joint_val = joint_val + fu*(target_pose - current_pose);
        pose_error = (target_pose - current_pose).norm();
        vari = abs(pose_error - pose_error_prev);
        iter_n++;
    }
    return joint_val;
}


MatrixXd iiwa14_kinematic::inverse_kine_closed_form(Matrix4d pose)
{
    //TODO: Fill in this function to complete Q2.
}

MatrixXd iiwa14_kinematic::getB(VectorXd joint_val)
{
    MatrixXd B = MatrixXd::Zero(7, 7);
    Matrix3d* inertia_tensors = new Matrix3d[7];
    for (int frame = 0; frame < 7; frame ++) {
        MatrixXd i_li_oi(3,3);
        i_li_oi << Ixyz.coeff(frame, 0), 0, 0,
                    0, Ixyz.coeff(frame, 1), 0, 
                    0, 0, Ixyz.coeff(frame, 2);
        Matrix4d T_gi = forward_kine_cm(joint_val, frame);
        inertia_tensors[frame] = T_gi.block(0,0,3,3)*i_li_oi*T_gi.block(0,0,3,3).transpose();
    }
    for (int frame = 0; frame < 7; frame++) {
        MatrixXd J_li = get_jacobian_cm(joint_val, frame);
        MatrixXd J_oi = J_li.block(3, 0, 3, 7);
        MatrixXd J_pi = J_li.block(0, 0, 3, 7);
        B = B + mass[frame]*J_pi.transpose()*J_pi + J_oi.transpose()*inertia_tensors[frame]*J_oi;
    }
    return B;
}

MatrixXd iiwa14_kinematic::getC(VectorXd joint_val, VectorXd joint_vel)
{
    MatrixXd C = MatrixXd::Zero(7, 7);
    MatrixXd* Bs = new MatrixXd[8];
    double epsi = 0.00001;
    Bs[0] = getB(joint_val);
    for (int frame = 0; frame < 7; frame++) {
        VectorXd joint_aux = joint_val;
        joint_aux[frame] = joint_aux[frame] + epsi;
        Bs[frame + 1] = getB(joint_aux);
    }
    MatrixXd basic_B = Bs[0];
    for (int i = 0; i < 7; i++)
    {
        MatrixXd B_der_i = Bs[i + 1];
        for (int j = 0; j < 7; j++)
        {
            VectorXd hij(7);
            for (int k = 0; k < 7; k++ )
            {
                MatrixXd B_der_k = Bs[k + 1];
                double sec_factor = (-1/2) * (B_der_i(j, k) - basic_B(j, k))/epsi;
                double fir_factor = (B_der_k(i, j) - basic_B(i, j))/epsi;
                hij(k) = fir_factor + sec_factor;
            }
            C(i,j) = hij.dot(joint_vel);
        }
    }
    return C;
}

VectorXd iiwa14_kinematic::getG(VectorXd joint_val)
{
    VectorXd G(7);
    double epsi = 0.00001;
    Vector3d gravity;
    gravity(2) = -g;
    for (int i = 0; i < 7; i++) 
    {
        Matrix4d* fkine_cm = new Matrix4d[8];
        VectorXd joint_aux = joint_val;
        joint_aux[i] = joint_aux[i] + epsi;
        double g_i = 0;
        for (int frame = 0; frame < 7; frame++) 
        {
            fkine_cm[frame + 1] = forward_kine_cm(joint_aux, frame);
            Matrix4d base_fkine = forward_kine_cm(joint_val, frame);
            Vector3d p_li_b = base_fkine.block(0,3,3,1);
            Vector3d p_li_e = fkine_cm[frame + 1].block(0,3,3,1);
            Vector3d d_p_li = (p_li_e - p_li_b)/epsi;
            g_i = g_i + mass[frame]*gravity.transpose().dot(d_p_li);
        }
        G(i) = -g_i;
    }
    return G;
}

bool iiwa14_kinematic::check_singularity(VectorXd joint_val)
{
    MatrixXd jacobian = get_jacobian(joint_val);
    MatrixXd Ide = MatrixXd::Identity(7, 7);
    MatrixXd inverse = jacobian.transpose()*jacobian;
    return inverse.determinant() == 0;
}
