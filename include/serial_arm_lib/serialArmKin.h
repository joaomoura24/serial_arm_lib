/**
 *      @file  serialArmKin.h
 *      @brief  Declaration of SerialArmKin class
 *
 * ... to complete with detailed description
 *
 *     @author  João Moura, Joao.Moura@ed.ac.uk
 *
 *   @internal
 *     Created  13-Feb-2017
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Edinburgh Centre for Robotics
 *   Copyright  Copyright (c) 2017, João Moura
 *
 * ===============================================================
 */
#ifndef _SERIAL_ARM_KIN_
#define _SERIAL_ARM_KIN_

#include <iostream> // print to terminal
#include <fstream> // check files (ifile)
#include <sstream> // file buffer
#include <string> // std::string
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h> // ROS header
#include <Eigen/Dense> // Library for vectors, matrices, and algebra operations
#include <urdf/model.h> // urdf::Model
#include <kdl_parser/kdl_parser.hpp> // KDL libraries
// KDL solvers
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>

#define SerialMsgError(msg) (std::string("SerialArmKin: ") + (msg) + "\nLine: " + std::to_string(__LINE__) + "; Function " + __PRETTY_FUNCTION__ + "; File " + __FILE__)
#define STD2EIGEN(vec) Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>((vec).data(), (vec).size())

class SerialArmKin
{
	public:
		SerialArmKin();
		~SerialArmKin();
		/**
		 * \brief Initializes the urdf model and the kdl tree from a string robot description
		 * @param xml: string with robot description in xml format
		 */
		void initFromString(const std::string &xml, const std::string &link);
		/**
		 * \brief Initializes the urdf model and the kdl tree from a ROS parameter robot description
		 * @param param: name of a parameter on the ROS parameter server
		 */
		void initFromParam(const std::string &param, const std::string &link);
		/**
		 * \brief Initializes the urdf model and the kdl tree from a urdf file name
		 * @param file: name of the urdf file
		 */
		void initFromFile(const std::string &file, const std::string &link);

		/**
		 * \brief Returns the robot Jacobian for a given configuration q
		 * @param jacobian: outputs robot jacobian
		 * @param q: inputs joints position
		 */
		void getJac(Eigen::Matrix<double, 6,Eigen::Dynamic> &jacobian, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q);

		/**
		 * \brief Returns the robot pseudo inverse Jacobian up to a specific link and for a given configuration q, and given a vector of weights for the joints
		 * @param jacobianInv: outputs robot pseudo inverse jacobian
		 * @param q: inputs joints position
		 * @param w_diag: weights for the importance of each link in the pseudo inverse jacobian computation
		 */
		void getJacInv(Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobianInv, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q, const Eigen::Matrix<double, Eigen::Dynamic, 1> &w_diag);

		/**
		 * \brief Returns the specified link position and orientation for a given configuration q
		 * @param pos: output cartesian position
		 * @param quat: outputs orientation in quaternions [qx, qy, qz, qw]
		 * @param q: input joints position
		 */
		void getFK(Eigen::Matrix<double, 3, 1> &pos, Eigen::Matrix<double, 4, 1> &quat, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q);

		/**
		 * \brief Returns the specified link position and orientation for a given configuration q
		 * @param pos: output cartesian position
		 * @param rot: output rotation matrix
		 * @param q: input joints position
		 */
		void getFK(Eigen::Matrix<double, 3, 1> &pos, Eigen::Matrix<double, 3, 3> &rot, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q);

		/**
		 * \brief Returns the relative position and orientation between the 2 specified links for a given configuration q
		 * @param pos: output cartesian position
		 * @param rot: output rotation matrix
		 * @param q: input joints position
		 * @param nameB: Base link name
		 * @param nameT: Tool link name
		 */
		void getBetweenFK(Eigen::Matrix<double, 3, 1> &pos, Eigen::Matrix<double, 3, 3> &rot, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q, const std::string &nameB, const std::string &nameT);

		/**
		 * \brief Returns the a joints vector for a specified end-effector position and orientation
		 * @return false if IK solver fails
		 * @param q_out: output joint positions
		 * @param q_in: input initial guess of joint positions
		 * @param pos: input cartesian position
		 * @param rot: input rotation matrix
		 */
		bool getIK(Eigen::Matrix<double, Eigen::Dynamic, 1> &q_out, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q_in, const Eigen::Matrix<double, 3, 1> &pos, const Eigen::Matrix<double, 4, 1> &quat);

		/**
		 * \brief Returns the robot Jacobian relative to the last link reference frame for a given configuration q
		 * @param jacobian: outputs robot jacobian
		 * @param q: inputs joints position
		 */
		void getLocJac(Eigen::Matrix<double, 6,Eigen::Dynamic> &jacobian, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q);

		/**
		 * \brief Returns the robot pseudo inverse local Jacobian up to a specific link and for a given configuration q, and given a vector of weights for the joints
		 * @param locJacobianInv: outputs robot pseudo inverse local jacobian
		 * @param q: inputs joints position
		 * @param w_diag: weights for the importance of each link in the pseudo inverse jacobian computation
		 */
		void getLocJacInv(Eigen::Matrix<double, Eigen::Dynamic, 6> &locJacobianInv, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q, const Eigen::Matrix<double, Eigen::Dynamic, 1> &w_diag);

		/**
		 * \brief TRansforms 3D vector in skew symmetric matrix
		 * @param v: 3D vector
		 * @return 3x3 skew symmetric matrix
		 */
		Eigen::Matrix<double, 3, 3> makeSkewSymmetric(const Eigen::Matrix<double, 3, 1> &v);

		// Robot constant variables (given by the urdf file):
		int nrJoints;
		std::string baseLink;
		std::vector<std::string> jointNames;
		std::vector<std::string> jointTypes;
		Eigen::Matrix<double, Eigen::Dynamic, 1> jointLowerLimits;
		Eigen::Matrix<double, Eigen::Dynamic, 1> jointUpperLimits;
		Eigen::Matrix<double, Eigen::Dynamic, 1> jointVelocity;
		Eigen::Matrix<double, Eigen::Dynamic, 1> jointEffort;
	private:
		/**
		 * \brief sets the joint names, types, limits, velocity, and efforts
		 * @param param: urdf model
		 */
		void setModelParameters_(urdf::Model &model);

		/**
		 * \brief sets the kdl solvers
		 * @param param: model tree
		 */
		void initKdlSolvers_(KDL::Tree &tree);

		/**
		 * \brief sets the kdl solvers
		 * @param param: model tree
		 */
		void invertJac_(Eigen::Matrix<double, Eigen::Dynamic, 6> &jacobianInv, const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q, const Eigen::Matrix<double, Eigen::Dynamic, 1> &w_diag);

		/**
		 * \brief sets the kdl solvers
		 * @return frame: frame for specified link and joint variables
		 * @param name: name of the link
		 * @param q: joints vector
		 */
		KDL::Frame getFrame_(const std::string &name, const Eigen::Matrix<double, Eigen::Dynamic, 1> &q);

		// Declare solvers
		boost::scoped_ptr<KDL::TreeJntToJacSolver> jac_kdl_solver;
		boost::scoped_ptr<KDL::TreeFkSolverPos_recursive> fk_kdl_solver;
		boost::scoped_ptr<KDL::TreeIkSolverVel_wdls> vel_ik_kdl_solver;
		boost::scoped_ptr<KDL::TreeIkSolverPos_NR_JL> ik_kdl_solver;
		// KDL variables
		KDL::Jacobian jacobianKdl_;
		KDL::JntArray qKdl_; // joint positions
		KDL::Frame ref_pose_;
		KDL::JntArray q_kdl_min; // joints lower limit
		Eigen::Matrix<double, Eigen::Dynamic, 1> cenas;
		KDL::JntArray q_kdl_max; // joints upper limit
		// Last link
		std::string lastLink_;
		// All link names
		std::vector<std::string> linkNames_;
};

#endif // _SERIAL_ARM_KIN_
