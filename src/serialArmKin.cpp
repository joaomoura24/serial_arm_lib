/**
 *      @file  serialArmKin.cpp
 *      @brief  Definition of SerialArmKin class
 *
 *
 *     @author  joão moura, Joao.Moura@ed.ac.uk
 *
 *   @internal
 *     Created  13-Feb-2017
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Edinburgh Centre for Robotics
 *   Copyright  Copyright (c) 2016, João Moura
 *
 * ===============================================================
 */

#include <serial_arm_lib/serialArmKin.h>

SerialArmKin::SerialArmKin()
{
	nrJoints = 0; // Initialize to 0 - if it remains 0 then the class was not initialized or there is something wrong with the urdf file
}

SerialArmKin::~SerialArmKin()
{
}

void SerialArmKin::initFromString(const std::string &xml, const std::string &link)
{
	// Get urdf model from ROS param
	urdf::Model model;
	if(!model.initString(xml)) throw std::runtime_error(SerialMsgError("Failed to load urdf::Model from robot description string"));
	// Get kdl tree from ROS param
	KDL::Tree tree;
	if(!kdl_parser::treeFromString(xml, tree)) throw std::runtime_error(SerialMsgError("Failed to load KDL::tree from robot description string"));
	// save last link name as private variable
	lastLink_ = link; // save link as private variable
	// Set kinematic parameters based on urdf model
	setModelParameters_(model);
	// Initialize KDL solvers:
	initKdlSolvers_(tree);

}

void SerialArmKin::initFromParam(const std::string &param, const std::string &link)
{
	// Test if param exists
	if(!ros::param::has(param)) throw std::runtime_error(SerialMsgError(param + " does not exist on parameter server"));
	// Get robot description urdf file as a std::string
	std::string robot_description;
	if(!ros::param::get(param,robot_description)) throw std::runtime_error(SerialMsgError("Failed to get robot description from " + param));
	// Initialize class from string
	initFromString(robot_description, link);
}

void SerialArmKin::initFromFile(const std::string &file, const std::string &link)
{
	// Test if file exists
	std::ifstream streamfile(file.c_str());
	if(!streamfile) throw std::runtime_error(SerialMsgError("File " + file + " does not exist"));
	// Read file
    std::stringstream buffer;
    buffer << streamfile.rdbuf();
    std::string robot_description = buffer.str();
	// Initialize class from string
	initFromString(robot_description, link);
}

void SerialArmKin::initKdlSolvers_(KDL::Tree &tree)
{
	// resizes the joint state vectors in non-realtime
	if(nrJoints != tree.getNrOfJoints()) throw std::runtime_error(SerialMsgError("Mismatch in number of joints"));
	jacobianKdl_.resize(nrJoints);
	qKdl_.resize(nrJoints);
	// joint lower and upper limits as kdl vectors
	q_kdl_min.data = jointLowerLimits;
	q_kdl_max.data = jointUpperLimits;
	// Define endpoints as the last segment name
	std::vector<std::string> endpoints (1, lastLink_);
	// constructs the kdl solvers in non-realtime
	jac_kdl_solver.reset(new KDL::TreeJntToJacSolver(tree));
	fk_kdl_solver.reset(new KDL::TreeFkSolverPos_recursive(tree));
	vel_ik_kdl_solver.reset(new KDL::TreeIkSolverVel_wdls(tree, endpoints));
	ik_kdl_solver.reset(new KDL::TreeIkSolverPos_NR_JL(tree, endpoints, q_kdl_min, q_kdl_max, *fk_kdl_solver, *vel_ik_kdl_solver));
}

void SerialArmKin::setModelParameters_(urdf::Model &model)
{
	// get base link name
    boost::shared_ptr<const urdf::Link> baseLink_ptr = model.getRoot();
	baseLink = baseLink_ptr->name;
	// Prov std vectors
	std::vector<double> jointLowerLimits_prov;
	std::vector<double> jointUpperLimits_prov;
	std::vector<double> jointVelocity_prov;
	std::vector<double> jointEffort_prov;
	// Get joint names, types, limits, etc
	for(std::map<std::string,boost::shared_ptr<urdf::Joint>>::iterator it=model.joints_.begin(); it!=model.joints_.end(); ++it)
	{
		if(it->second->type == urdf::Joint::REVOLUTE) jointTypes.push_back("revolute");
		if(it->second->type == urdf::Joint::PRISMATIC) jointTypes.push_back("prismatic");
		if(it->second->type == urdf::Joint::REVOLUTE || it->second->type == urdf::Joint::PRISMATIC)
		{
			jointNames.push_back(it->second->name);
			jointLowerLimits_prov.push_back(it->second->limits->lower);
			jointUpperLimits_prov.push_back(it->second->limits->upper);
			jointVelocity_prov.push_back(it->second->limits->velocity);
			jointEffort_prov.push_back(it->second->limits->effort);
		}
	}
	nrJoints = jointNames.size();
	// copy joint limits, velocities, and efforts to eigen vector
	jointLowerLimits = STD2EIGEN(jointLowerLimits_prov);
	jointUpperLimits = STD2EIGEN(jointUpperLimits_prov);
	jointVelocity = STD2EIGEN(jointVelocity_prov);
	jointEffort = STD2EIGEN(jointEffort_prov);
	// get link names
	for(std::map<std::string,boost::shared_ptr<urdf::Link>>::iterator it=model.links_.begin(); it!=model.links_.end(); ++it)
	{
		linkNames_.push_back(it->second->name); // get the name of all links
	}
	if(!(std::find(linkNames_.begin(), linkNames_.end(), lastLink_) != linkNames_.end())) throw std::runtime_error(SerialMsgError("urdf does not contain specified link"));
}

bool SerialArmKin::getIK(Eigen::Matrix<double, Eigen::Dynamic, 1> &q_out, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q_in, const Eigen::Matrix<double, 3, 1> &pos, const Eigen::Matrix<double, 4, 1> &quat)
{
	// Get End-effector frame
	KDL::Vector pos_out(pos(0), pos(1), pos(2));
	KDL::Rotation rot_out; rot_out = KDL::Rotation::Quaternion(quat(0), quat(1), quat(2), quat(3));
	KDL::Frame frame_out(rot_out, pos_out);
	// Get Frame maps
	KDL::Frames frame_out_maps;
	frame_out_maps[lastLink_] = frame_out;
	// get kdl join values input
	qKdl_.data = q_in;
	// kdl joint vector for output
	KDL::JntArray q_kdl_out;
	// compute final configuratiopn for desired end-effector position
	if(ik_kdl_solver->CartToJnt(qKdl_, frame_out_maps, q_kdl_out) < 0.0) return false;
	// get end-effector position
	q_out = Eigen::Matrix<double,Eigen::Dynamic,1>(q_kdl_out.data);
	// Succeded
	return true;
}

void SerialArmKin::getFK(Eigen::Matrix<double, 3, 1> &pos, Eigen::Matrix<double, 4, 1> &quat, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q)
{
	// get frame
	KDL::Frame frame = getFrame_(lastLink_, q);
	// get end-effector position
	pos = Eigen::Matrix<double,3,1>(frame.p.data);
	// get end-effector orientation in quaternions
	frame.M.GetQuaternion(quat(0), quat(1), quat(2), quat(3)); // x, y, z, w
	quat.head(3) = -quat.head(3); // inverse orientation
}

void SerialArmKin::getFK(Eigen::Matrix<double, 3, 1> &pos, Eigen::Matrix<double, 3, 3> &rot, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q)
{
	// get frame
	KDL::Frame frame = getFrame_(lastLink_, q);
	// get end-effector position
	pos = Eigen::Matrix<double,3,1>(frame.p.data);
	// get end-effector rotation matrix
	rot = (Eigen::Map<Eigen::Matrix<double,3,3>>(frame.M.data)).transpose();
}

void SerialArmKin::getBetweenFK(Eigen::Matrix<double, 3, 1> &pos, Eigen::Matrix<double, 3, 3> &rot, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q, const std::string &nameB, const std::string &nameT)
{
	// check if nameB link exists
	if(!(std::find(linkNames_.begin(), linkNames_.end(), nameB) != linkNames_.end())) throw std::runtime_error(SerialMsgError("urdf does not contain specified link"));
	// check if nameT link exists
	if(!(std::find(linkNames_.begin(), linkNames_.end(), nameT) != linkNames_.end())) throw std::runtime_error(SerialMsgError("urdf does not contain specified link"));
	// get frame for Base
	KDL::Frame frameB = getFrame_(nameB, q);
	// get frame for Tool
	KDL::Frame frameT = getFrame_(nameT, q);
	// get end-effector position for base
	Eigen::Matrix<double, 3, 1> posB = Eigen::Matrix<double,3,1>(frameB.p.data);
	// get end-effector position for tool
	Eigen::Matrix<double, 3, 1> posT = Eigen::Matrix<double,3,1>(frameT.p.data);
	// get end-effector rotation matrix for Base
	Eigen::Matrix<double, 3, 3> rotB = (Eigen::Map<Eigen::Matrix<double,3,3>>(frameB.M.data)).transpose();
	// get end-effector rotation matrix for Tool
	Eigen::Matrix<double, 3, 3> rotT = (Eigen::Map<Eigen::Matrix<double,3,3>>(frameT.M.data)).transpose();
	// get difference between positions
	pos = (rotB.transpose())*(posT - posB);
	// get difference between rotations
	rot = (rotB.transpose())*rotT;
}

void SerialArmKin::getJac(Eigen::Matrix<double, 6,Eigen::Dynamic> &jacobian, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q)
{
	// test input
	if(q.size() != nrJoints) throw std::runtime_error(SerialMsgError("Robot with " + std::to_string(nrJoints) + " joints. Joint vector size is " + std::to_string(q.size())));
	// set kdl joint vector
    qKdl_.data = q;
	// get kdl Jacobian
    jac_kdl_solver->JntToJac(qKdl_, jacobianKdl_, lastLink_);
	// get Jacobian as eigen matrix
	jacobian = jacobianKdl_.data;
}

void SerialArmKin::getJacInv(Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobianInv, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q, const Eigen::Matrix<double, Eigen::Dynamic, 1> &w_diag)
{
	if(w_diag.size() != nrJoints) throw std::runtime_error(SerialMsgError("Robot with " + std::to_string(nrJoints) + " joints. Weight vector size is " + std::to_string(w_diag.size())));
	// Get Jacobian
	Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
	getJac(jacobian, q);
	// Invert jacobian
	invertJac_(jacobianInv, jacobian, q, w_diag);
}

void SerialArmKin::getLocJac(Eigen::Matrix<double, 6,Eigen::Dynamic> &locJacobian, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q)
{
	// Get Jacobian
	Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
	getJac(jacobian, q);
	// Get rotation matrix
	Eigen::Matrix<double, 3, 1> pos;
	Eigen::Matrix<double, 3, 3> rot;
	getFK(pos, rot, q);
	// get transformation matrix between end-effector and global frames
	Eigen::Matrix<double, 6, 6> Tinv;
	Tinv << rot.transpose(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), rot.transpose();
	// transform global jacobian to local Jacobian
	locJacobian = (Tinv)*jacobian;
	// Print for debug:
/*
	ROS_WARN_STREAM("Jac:");
	std::cout << jacobian << std::endl;
	ROS_WARN_STREAM("Tinv:");
	std::cout << Tinv << std::endl;
	ROS_WARN_STREAM("Loc Jac:");
	std::cout << locJacobian << std::endl;
*/
}

void SerialArmKin::getLocJacInv(Eigen::Matrix<double, Eigen::Dynamic, 6> &locJacobianInv, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q, const Eigen::Matrix<double, Eigen::Dynamic, 1> &w_diag)
{
	if(w_diag.size() != nrJoints) throw std::runtime_error(SerialMsgError("Robot with " + std::to_string(nrJoints) + " joints. Weight vector size is " + std::to_string(w_diag.size())));
	// Get Jacobian
	Eigen::Matrix<double, 6, Eigen::Dynamic> locJacobian;
	getLocJac(locJacobian, q);
	// Invert locJacobian
	invertJac_(locJacobianInv, locJacobian, q, w_diag);
}

void SerialArmKin::invertJac_(Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobianInv, const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q, const Eigen::Matrix<double, Eigen::Dynamic, 1> &w_diag)
{
	// Get inverse of weighting matrix
	Eigen::Matrix<double, Eigen::Dynamic, 1> w_diag_inv;
	w_diag_inv = w_diag.cwiseInverse();
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W_inv;
	W_inv = w_diag_inv.asDiagonal();
	// Check if robot is in singular configuration
	Eigen::Matrix<double, 6, 6> provMat = (jacobian*W_inv)*(jacobian.transpose());
	Eigen::FullPivLU<Eigen::Matrix<double,6,6>> lu_decomp(provMat);
	// Print eigen values
/*
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eigensolver(provMat);
	if(eigensolver.info() == Eigen::Success){
		ROS_WARN_STREAM("Eigen Values:");
		std::cout << eigensolver.eigenvalues().transpose() << std::endl;
	}
*/
	if(lu_decomp.isInvertible()){ 
		// Compute Inverse Jacobian
		jacobianInv = ((W_inv*(jacobian.transpose()))*(provMat.inverse()));
	}
	else{ // rank of jacobian is smaller than 6
		ROS_WARN_STREAM("Robot in singularity -> setting velocities to zero");
		jacobianInv.resize(nrJoints,6);
		jacobianInv.setZero();
	}
	//ROS_WARN_STREAM("JacInv:");
	//std::cout << jacobianInv << std::endl;
}

KDL::Frame SerialArmKin::getFrame_(const std::string &name, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q)
{
	// test input
	if(q.size() != nrJoints) throw std::runtime_error(SerialMsgError("Robot with " + std::to_string(nrJoints) + " joints. Joint vector size is " + std::to_string(q.size())));
	// set kdl joint vector
	KDL::JntArray qKdl; qKdl.data = q; // joint positions
	// get kdl frame
	KDL::Frame frame; fk_kdl_solver->JntToCart(qKdl, frame, name);
	return frame;
}

Eigen::Matrix<double, 3, 3> SerialArmKin::makeSkewSymmetric(const Eigen::Matrix<double, 3, 1> &v)
{
	Eigen::Matrix<double, 3, 3> out;
	out << 0, -v[2], v[1],
		v[2], 0, -v[0],
		-v[1], v[0], 0;
	return out;
}
