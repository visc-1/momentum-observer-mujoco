#include <mujoco/mujoco.h>
#include "MujocoUI.hpp"
#include <iostream>
#include <memory>
#include <filesystem>

// Pinocchio
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

// Eigen
#include <Eigen/Dense>


// Funzione per mappare gli array di MuJoCo a Eigen
template <typename EigenVector>
auto toEigen(mjtNum* data, const EigenVector& target_type) {
    return Eigen::Map<EigenVector>(data, target_type.size());
}

template <typename EigenVector>
auto toEigen(const mjtNum* data, const EigenVector& target_type) {
    return Eigen::Map<const EigenVector>(data, target_type.size());
}

static int framerate = 60.0;

int main(int argc, char** argv)
{
    // Load MuJoCo UR10 scene
    std::string scene_path = "../robot/ur10_scene.xml";
    
    std::cout << "Loading scene from: " << scene_path << std::endl;
    std::cout << "Current working directory: " << std::filesystem::current_path() << std::endl;
    
    // Load MuJoCo scene (includes robot + ground plane)
    const int kErrorLength = 1024;
    char loadError[kErrorLength] = "";
    mjModel* mj_model_ptr = mj_loadXML(scene_path.c_str(), nullptr, loadError, kErrorLength);
    
    if (!mj_model_ptr) {
        std::cerr << "Failed to load scene: " << loadError << std::endl;
        return 1;
    }
    
    std::cout << "Successfully loaded UR10 scene with robot and ground" << std::endl;
    

    // Carica il modello URDF in Pinocchio
    pinocchio::Model pin_model;
    std::string urdf_path = "../robot/ur10/ur10.urdf";
    pinocchio::urdf::buildModel(urdf_path, pin_model);
    std::cout << "Successfully loaded model in Pinocchio." << std::endl;

    // Crea dati per MuJoCo e Pinocchio
    mjData* mj_data_ptr = mj_makeData(mj_model_ptr);
    pinocchio::Data pin_data(pin_model);

    std::cout << "Number of DoFs (MuJoCo): " << mj_model_ptr->nv << std::endl;
    std::cout << "Number of DoFs (Pinocchio): " << pin_model.nv << std::endl;

    // Set initial joint positions (home configuration)
    // UR10 has 6 revolute joints
    // Set to a reasonable home pose
    for (int i = 0; i < mj_model_ptr->nq; i++) {
        mj_data_ptr->qpos[i] = 0.0;
    }

    mj_data_ptr->qpos[1] = -M_PI/2.0; // Inizializza in una posa più "naturale"
    mj_data_ptr->qpos[2] = M_PI/2.0;

    // Posizione target per il controllore
    Eigen::VectorXd q_target(pin_model.nq);
    q_target << 0, -M_PI / 2.0, M_PI / 2.0, 0.0, 0.0, 0;
    

    // Guadagni del controllore PD
    double Kp = 0.0001;
    double Kd = 0.0005;
    
    // Forward kinematics to update positions
    mj_forward(mj_model_ptr, mj_data_ptr);
    
    // Initialize MuJoCo UI
    MujocoUI* mujoco_ui = MujocoUI::getInstance(mj_model_ptr, mj_data_ptr);
    
    std::cout << "Starting visualization. Press ESC or close window to exit." << std::endl;
    
    // Main rendering loop
    while (!mujoco_ui->windowShouldClose()) {

        mjtNum simstart = mj_data_ptr->time;
        while( mj_data_ptr->time - simstart < 1.0/framerate ) {
            // Ottieni stato corrente (posizione e velocità)
            Eigen::Map<const Eigen::VectorXd> q_current(mj_data_ptr->qpos, pin_model.nq);
            Eigen::Map<const Eigen::VectorXd> v_current(mj_data_ptr->qvel, pin_model.nv);
            
            mj_step1(mj_model_ptr, mj_data_ptr);

            // Calcola la coppia di compensazione della gravità con Pinocchio
            // La funzione rnea con velocità e accelerazione nulle calcola le coppie di gravità
            const Eigen::VectorXd& g = pinocchio::computeGeneralizedGravity(pin_model, pin_data, q_current);

            // Calcola la coppia del controllore PD
            Eigen::VectorXd tau_pd = Kp * (q_target - q_current) - Kd * v_current;
            
            // Coppia totale da applicare (PD + compensazione gravità)
            Eigen::VectorXd tau_cmd = g; //tau_pd;
            
            // Applica le coppie agli attuatori di MuJoCo
            for (int i = 0; i < mj_model_ptr->nu; ++i) {
                mj_data_ptr->ctrl[i] = tau_cmd[i];
            }

            mj_step2(mj_model_ptr, mj_data_ptr);
            std::cout << "time: " << mj_data_ptr->time << std::endl;
        }
        

        // Render
        mujoco_ui->render();
        
    }

    // Cleanup
    delete mujoco_ui;
    mj_deleteData(mj_data_ptr);
    mj_deleteModel(mj_model_ptr);
    
    std::cout << "Exiting..." << std::endl;
    
    return 0;
}
