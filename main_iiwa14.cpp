// Pinocchio
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/crba.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>


// Eigen
#include <Eigen/Dense>


#include <mujoco/mujoco.h>
#include "MujocoUI.hpp"
#include <iostream>
#include <memory>
#include <filesystem>
#include <fstream> // Aggiunto per std::ofstream
#include <chrono>  // Aggiunto per il controllo del tempo
#include <thread>  // Aggiunto per std::this_thread::sleep_for

#include <MomentumObserver.hpp>

#define FORCE_START 2.0
#define FORCE_STOP 10.0


void append_vector_to_csv(const std::string& filename, const Eigen::VectorXd& vec, double current_time) {
    const static Eigen::IOFormat CsvFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

    std::ofstream file;
    file.open(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "ERRORE: Impossibile aprire il file " << filename << std::endl;
        return;
    }

    file << current_time << ", ";
    file << vec.transpose().format(CsvFormat) << std::endl;
}

void print_vector(const std::string& name, const Eigen::VectorXd& vec) {
    std::cout << name << ": [";
    for (int i = 0; i < vec.size(); ++i) {
        std::cout << " " << vec(i);
    }
    std::cout << " ]" << std::endl;
}

static int framerate = 60.0;

int main(int argc, char** argv)
{
    // Load MuJoCo scene
    std::string scene_path = "../robot/kuka_iiwa_14/scene.xml";
    
    std::cout << "Loading scene from: " << scene_path << std::endl;
    std::cout << "Current working directory: " << std::filesystem::current_path() << std::endl;
    
    const int kErrorLength = 1024;
    char loadError[kErrorLength] = "";
    mjModel* mj_model_ptr = mj_loadXML(scene_path.c_str(), nullptr, loadError, kErrorLength);
    
    if (!mj_model_ptr) {
        std::cerr << "Failed to load scene: " << loadError << std::endl;
        return 1;
    }
    
    std::cout << "Successfully loaded iiwa14 scene with robot and ground" << std::endl;

    // Carica il modello URDF in Pinocchio
    pinocchio::Model pin_model;
    std::string urdf_path = "../robot/kuka_iiwa_14/iiwa14_primitive_collision.urdf";
    pinocchio::urdf::buildModel(urdf_path, pin_model);
    std::cout << "Successfully loaded model in Pinocchio." << std::endl;

    // Crea dati per MuJoCo e Pinocchio
    mjData* mj_data_ptr = mj_makeData(mj_model_ptr);
    pinocchio::Data pin_data(pin_model);

    std::cout << "Number of DoFs (MuJoCo): " << mj_model_ptr->nv << std::endl;
    std::cout << "Number of DoFs (Pinocchio): " << pin_model.nv << std::endl;
    
    // --- Inizializza l'interfaccia UI ---
    MujocoUI* mujoco_ui = MujocoUI::getInstance(mj_model_ptr, mj_data_ptr);

    // --- Definizione della traiettoria ---
    const double T_period = 5.0;
    const double t_start = 0.0;
    Eigen::VectorXd A(pin_model.nv);
    A << 1, 0.0, 0.0, 1, 0.0, 0.0, 0.0;
    const double phi = 0.0;
    const double omega = 2.0 * M_PI / T_period;

    // --- Guadagni del controllore ---
    double Kp = 5.0;
    double Kd = 1.0;

    

    // --- Inizializzazione file CSV ---
    std::ofstream file;
    file.open("q.csv", std::ios::out);
    file << "Time, q_1, q_2, q_3, q_4, q_5, q_6, q_7" << std::endl;
    file.close();

    file.open("qd.csv", std::ios::out);
    file << "Time, qd_1, qd_2, qd_3, qd_4, qd_5, qd_6, qd_7" << std::endl;
    file.close();

    file.open("r.csv", std::ios::out);
    file << "Time, r_1, r_2, r_3, r_4, r_5, r_6, r_7" << std::endl;
    file.close();

    file.open("wrench.csv", std::ios::out);
    file << "Time, f_1, f_2, f_3, f_4, f_5, f_6" << std::endl;
    file.close();

    // --- Ciclo di Simulazione ---
    Eigen::VectorXd q_desired(pin_model.nv);
    Eigen::VectorXd qd_desired(pin_model.nv);
    Eigen::VectorXd qdd_desired(pin_model.nv);
    q_desired << 1.57, 0, 0, 1.57, 0.0, 0.0, 0.0;
    qd_desired.setZero();
    qdd_desired.setZero();

    // --- Inizializzazione della posizione del robot ---
    for (int i = 0; i < mj_model_ptr->nq; i++) {
        mj_data_ptr->qpos[i] = q_desired[i];
        mj_data_ptr->qvel[i] = 0.0;

        //mj_data_ptr->qpos[i] = A[i] * sin(phi);
        //mj_data_ptr->qvel[i] = A[i] * omega * cos(phi);
    }
    mj_forward(mj_model_ptr, mj_data_ptr);

    //Forza
    bool is_force_applied = false;
    Eigen::Vector3d force_frame;
    force_frame << 0.3, -0.5, 0.2;
    Eigen::Vector3d neg_force_frame = -force_frame;
    int body_id = mj_name2id(mj_model_ptr, mjOBJ_BODY, "link7");
    const pinocchio::FrameIndex frame_id = pin_model.getFrameId("iiwa_link_7");
    Eigen::Vector3d point_of_application_world;// = pin_data.oMf[ee_frame_id].translation();
    Eigen::Vector3d force_world;
    Eigen::Vector3d neg_force_world;
    Eigen::Matrix3d R_world_ee;
    //for (int i = 0; i < mj_model_ptr->nbody; ++i) {
    //  const char* body_name = mj_id2name(mj_model_ptr, mjOBJ_BODY, i);
    //        std::cout << "Nome body {"<<i<<"} :" << body_name << std::endl;
    //}
    //return 0;

    Eigen::VectorXd Ko_gains(pin_model.nv);
    Ko_gains.setConstant(5.0);
    MomentumObserver observer(Ko_gains, pin_model, pin_data, 0.001);


    for (pinocchio::FrameIndex frame_id = 0; frame_id < pin_model.nframes; ++frame_id) {
        const auto& frame = pin_model.frames[frame_id];
        
        std::cout << "Frame ID: " << frame_id 
                  << ", Nome: \"" << frame.name << "\""
                  << ", Attaccato al Giunto ID: " << frame.parent 
                  << " (Nome Giunto: \"" << pin_model.names[frame.parent] << "\")" 
                  << std::endl;
    }

    std::cout << "\nInizio del ciclo di simulazione..." << std::endl;
    while (!mujoco_ui->windowShouldClose()) {
        mjtNum simstart = mj_data_ptr->time;
        while( mj_data_ptr->time - simstart < 1.0/framerate ){
            
            // --- Calcolo Traiettoria Desiderata ---
            double t = mj_data_ptr->time;
            const double wt_phi = omega * t + phi;
            //q_desired = A * sin(wt_phi);
            //qd_desired = A * omega * cos(wt_phi);
            //qdd_desired = -A * omega * omega * sin(wt_phi);

            // --- Acquisizione Stato Corrente ---
            Eigen::Map<const Eigen::VectorXd> q_current(mj_data_ptr->qpos, pin_model.nq);
            Eigen::Map<const Eigen::VectorXd> qd_current(mj_data_ptr->qvel, pin_model.nv);


            

            if(t>=FORCE_START && t < FORCE_STOP){
                //Applica forza al end-effector
                if(is_force_applied)
                    mj_applyFT(mj_model_ptr, mj_data_ptr, neg_force_world.data(), nullptr, point_of_application_world.data(), body_id, mj_data_ptr->qfrc_applied);
                
                pinocchio::forwardKinematics(pin_model, pin_data, q_current);
                pinocchio::updateFramePlacements(pin_model, pin_data);
                //R_world_ee = pin_data.oMf[ee_frame_id].rotation();
                //mjtNum* ee_com_pos = mj_data_ptr->xipos + (ee_body_id * 3);
                //point_of_application_world = Eigen::Vector3d(ee_com_pos[0], ee_com_pos[1], ee_com_pos[2]);
                point_of_application_world = pin_data.oMf[frame_id].translation();
                force_world = force_frame;
                neg_force_world = neg_force_frame;
                mj_applyFT(mj_model_ptr, mj_data_ptr, force_world.data(), nullptr, point_of_application_world.data(), body_id, mj_data_ptr->qfrc_applied);
                
                if (!is_force_applied) {
                    std::cout << "Force applied at second: " << t << std::endl;
                    is_force_applied = true;
                }
                std::cout<< "Punto di applicazione: "<< point_of_application_world <<std::endl;
                std::cout<< "Forza nel frame del mondo: "<< force_world <<std::endl;
            
            }
            
            if (is_force_applied && t >= FORCE_STOP) {
                mj_applyFT(mj_model_ptr, mj_data_ptr, neg_force_world.data(), nullptr, point_of_application_world.data(), body_id, mj_data_ptr->qfrc_applied);
                is_force_applied = false;
                std::cout << "Force removed at second: " << t << std::endl;
            }

            // --- Calcolo Dinamica con Pinocchio ---
            pinocchio::crba(pin_model, pin_data, q_current);
            pin_data.M.triangularView<Eigen::StrictlyLower>() = pin_data.M.transpose().triangularView<Eigen::StrictlyLower>();
            // Calcola nle (C(q,q̇)q̇ + g(q))
            //pinocchio::nonLinearEffects(pin_model, pin_data, q_current, qd_current);
            // Gravity compensation only
            //pinocchio::rnea(pin_model, pin_data, q_current, Eigen::VectorXd::Zero(pin_model.nv), Eigen::VectorXd::Zero(pin_model.nv));
            pinocchio::rnea(pin_model, pin_data, q_current, qd_current, qdd_desired);

            // --- Legge di Controllo (Feedback Linearization CORRETTA) ---
            Eigen::VectorXd tau_fb = Kp * (q_desired - q_current) + Kd * (qd_desired - qd_current);
            Eigen::VectorXd tau_cmd = pin_data.tau + pin_data.M*tau_fb;

            // --- Applicazione del Controllo ---
            for (int i = 0; i < mj_model_ptr->nu; ++i) {
                mj_data_ptr->ctrl[i] = tau_cmd[i];
            }
            
            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, pin_model.nv);
            pinocchio::framesForwardKinematics(pin_model, pin_data, q_current);
            pinocchio::updateFramePlacements(pin_model, pin_data);
            pinocchio::computeJointJacobians(pin_model, pin_data, q_current);
            pinocchio::getFrameJacobian(pin_model, pin_data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

            Eigen::VectorXd r = observer.update(q_current, qd_current, tau_cmd);
            Eigen::VectorXd force_wrench = observer.reconstructForceWrench(J);

            // --- Log dei dati ---
            append_vector_to_csv("wrench.csv", force_wrench, t);
            append_vector_to_csv("r.csv", r, t);
            append_vector_to_csv("q.csv", q_current, t);
            append_vector_to_csv("qd.csv", qd_current, t);
            std::cout << "time: " << mj_data_ptr->time << std::endl;
            // --- Avanzamento della Simulazione (singolo step) ---
            mj_step(mj_model_ptr, mj_data_ptr);

        }
        
        // --- Renderizzazione ---
        mujoco_ui->render();
    }
    
    std::cout << "\nSimulazione completata." << std::endl;
    print_vector("Posizione finale (q)", Eigen::Map<Eigen::VectorXd>(mj_data_ptr->qpos, mj_model_ptr->nq));

    // Cleanup
    delete mujoco_ui;
    mj_deleteData(mj_data_ptr);
    mj_deleteModel(mj_model_ptr);
    
    std::cout << "Exiting..." << std::endl;
    
    return 0;
}