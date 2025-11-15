#include <MomentumObserver.hpp>

MomentumObserver::MomentumObserver(
            const Eigen::VectorXd& diagKo,
            const pinocchio::Model& robot_model, 
            const pinocchio::Data& robot_data,
            double dt)
            :
            robot_model(robot_model),
            robot_data(robot_data),
            dt(dt){
                if(robot_model.nv != diagKo.size()){
                    std::cerr << "ERRORE: dimensione passate all'osservatore errate" << std::endl;
                    return;
                }
                Ko = diagKo.asDiagonal();
                r_integral_term = Eigen::VectorXd::Zero(robot_model.nv);
                last_r = Eigen::VectorXd::Zero(robot_model.nv);
                P0 = Eigen::VectorXd::Zero(robot_model.nv);
                is_zero_initialized = false;
                J_contact_point = Eigen::MatrixXd::Zero(6, robot_model.nv);

            }

Eigen::VectorXd MomentumObserver::update(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& tau){
    Eigen::MatrixXd M = pinocchio::crba(robot_model, robot_data, q);
    // We need to do this since the inertia matrix in Pinocchio is only upper triangular
    M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
    const auto& c = pinocchio::rnea(robot_model, robot_data, q, qdot, Eigen::VectorXd::Zero(robot_model.nv));
    const auto& coriolis = pinocchio::computeCoriolisMatrix(robot_model, robot_data, q, qdot);
    const auto& coriolisT = coriolis.transpose();
    const auto& gravity = c - coriolis * qdot;
    const auto& beta = gravity - coriolisT * qdot;
    const auto& Pt = M*qdot;
    if(!is_zero_initialized){
      P0 = Pt;
      is_zero_initialized = true;
    }

    // Residual method
    r_integral_term += (tau - beta + last_r)* dt;
    Eigen::VectorXd r = Ko * (Pt  - r_integral_term - P0);
    last_r = r;

    return r;
}

//Call before passing the jacobian
//pinocchio::framesForwardKinematics(robot_model, robot_data, q);
//pinocchio::updateFramePlacements(robot_model, robot_data);
//pinocchio::computeJointJacobians(robot_model, robot_data, q);
//pinocchio::getFrameJacobian(robot_model, robot_data, id_, pinocchio::ReferenceFrame::YOURCHOICE, J);
Eigen::VectorXd MomentumObserver::reconstructForceWrench(const Eigen::MatrixXd& J){
      return J.transpose().completeOrthogonalDecomposition().pseudoInverse() * last_r;
}