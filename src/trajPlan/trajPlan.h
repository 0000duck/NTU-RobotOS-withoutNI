#pragma once
#include "Eigen\Eigen\Dense"
#include <vector>

namespace tp{
	// from Hai brother's code
	Eigen::MatrixXd Splines212(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::MatrixXd& qm, double t_total, int select);
	Eigen::MatrixXd Splines212(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, double t_total, int select);
	Eigen::MatrixXd Splines434(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& q0dd, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::VectorXd& qfdd, const Eigen::MatrixXd& qm, double t_total, int select);
	Eigen::MatrixXd Splines434(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& q0dd, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::VectorXd& qfdd, double t_total, int select);
	Eigen::MatrixXd Splines535(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& q0dd, const Eigen::VectorXd& q0ddd, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::VectorXd& qfdd, const Eigen::VectorXd& qfddd, const Eigen::MatrixXd& qm, double t_total, int select);
	Eigen::MatrixXd Splines535(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& q0dd, const Eigen::VectorXd& q0ddd, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::VectorXd& qfdd, const Eigen::VectorXd& qfddd, double t_total, int select);
	Eigen::MatrixXd CubicPolynomials(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, const Eigen::MatrixXd& qm, double t_total, int select);
	Eigen::MatrixXd CubicPolynomials(const Eigen::VectorXd& q0, const Eigen::VectorXd& q0d, const Eigen::VectorXd& qf, const Eigen::VectorXd& qfd, double t_total, int select);
	// by me
	void interpolation(unsigned number, const std::vector<double>& start, const std::vector<double>& end, std::vector<double>& delx);
}