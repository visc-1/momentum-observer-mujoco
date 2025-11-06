// Pinocchio
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include "pinocchio/algorithm/crba.hpp"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

#include <mujoco/mujoco.h>
#include "MujocoUI.hpp"
#include <iostream>
#include <memory>
#include <filesystem>



// Eigen
#include <Eigen/Dense>


// Funzione per stampare vettori Eigen in modo pulito
void print_vector(const std::string& name, const Eigen::VectorXd& vec) {
    std::cout << name << ": [";
    for (int i = 0; i < vec.size(); ++i) {
        std::cout << " " << vec(i);
    }
    std::cout << " ]" << std::endl;
}


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
    // Load MuJoCo scene
    std::string scene_path = "../robot/kuka_iiwa_14/scene.xml";
    
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
    
    std::cout << "\n--- MuJoCo Joints ---" << std::endl;
    for (int i = 0; i < mj_model_ptr->njnt; ++i) {
        const char* name = mj_id2name(mj_model_ptr, mjOBJ_JOINT, i);
        if (name) { // mj_id2name restituisce nullptr se non c'è un nome
            std::cout << "Joint " << i << ": " << name << std::endl;
        }
    }

    std::cout << "\n--- Pinocchio Joints ---" << std::endl;
    // I giunti in Pinocchio partono dall'indice 1 (l'indice 0 è 'universe')
    for (pinocchio::JointIndex i = 0; i < pin_model.njoints; ++i) {
        std::cout << "Joint " << i << ": " << pin_model.names[i] << std::endl;
    }   

    
    
    // Initialize MuJoCo UI
    MujocoUI* mujoco_ui = MujocoUI::getInstance(mj_model_ptr, mj_data_ptr);

    // Defining joint space trajectory
    const double T_period = 5.0;
    const double t_start = 0.0;
    const double t_end = 2.0 * T_period; // Due giri completi
    
    // In MuJoCo, t_step è il passo della simulazione
    const double simulation_step = 0.001; 

    // Parametri della sinusoide
    Eigen::VectorXd A(7); // Vettore delle ampiezze
    A << 0.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    const double phi = 0.0; // Fase
    const double omega = 2.0 * M_PI / T_period; // Frequenza angolare

    std::cout << "Parametri della traiettoria impostati." << std::endl;
    print_vector("Ampiezze (A)", A);
    std::cout << "Periodo (T): " << T_period << "s, Frequenza (omega): " << omega << " rad/s" << std::endl;


    // Guadagni del controllore PD
    double Kp = 100;
    double Kd = 20;

    for (int i = 0; i < mj_model_ptr->nq; i++) {
        mj_data_ptr->qpos[i] = A[i] * sin(phi);
        mj_data_ptr->qvel[i] = A[i] * omega * cos(phi);
        mj_data_ptr->qacc[i] = -A[i] * omega * omega * sin(phi);
    }

    // Forward kinematics to update positions
    mj_forward(mj_model_ptr, mj_data_ptr);


    // --- 3. Ciclo di Simulazione ---
    
    // Vettori Eigen per memorizzare i valori desiderati ad ogni passo
    Eigen::VectorXd q_desired(pin_model.nv);
    Eigen::VectorXd qd_desired(pin_model.nv);
    Eigen::VectorXd qdd_desired(pin_model.nv);

    std::cout << "\nInizio del ciclo di simulazione..." << std::endl;
    while (!mujoco_ui->windowShouldClose()) {
        mjtNum simstart = mj_data_ptr->time;
        while( mj_data_ptr->time - simstart < 1.0/framerate ){
            auto loop_start = std::chrono::high_resolution_clock::now();

            // --- 3.1 Calcolo della Traiettoria all'istante t ---
            // Questa è la traduzione diretta delle tue formule MATLAB.
            // Grazie a Eigen, possiamo usare operazioni vettoriali.
            double t = mj_data_ptr->time;
            const double wt_phi = omega * t + phi;
            const double sin_val = sin(wt_phi);
            const double cos_val = cos(wt_phi);
            q_desired = A * sin_val;
            qd_desired = A * omega * cos_val;
            qdd_desired = -A * omega * omega * sin_val;
            // Stampiamo i valori desiderati al primo passo per verifica
            //if (t == t_start) {
            print_vector("q_desired", q_desired);
            print_vector("qd_desired", qd_desired);
            //}
            //
            

            

            Eigen::Map<const Eigen::VectorXd> q_current(mj_data_ptr->qpos, pin_model.nq);
            Eigen::Map<const Eigen::VectorXd> qd_current(mj_data_ptr->qvel, pin_model.nv);
            //Eigen::VectorXd q_current(pin_model.nv);
            //Eigen::VectorXd qd_current(pin_model.nv);
            //// 2. Copia i dati da MuJoCo a Eigen elemento per elemento
            //for (int i = 0; i < pin_model.nv; ++i) {
            //    q_current(i) = mj_data_ptr->qpos[i];
            //    qd_current(i) = mj_data_ptr->qvel[i];
            //}

            //print_vector("q_current", q_current);
            //print_vector("qd_current", qd_current);

            //pinocchio::rnea(pin_model, pin_data, q_current, qd_current, Eigen::VectorXd::Zero(pin_model.nv));
            //pinocchio::crba(pin_model, pin_data, q_current);
            //pin_data.M.triangularView<Eigen::StrictlyLower>() = pin_data.M.transpose().triangularView<Eigen::StrictlyLower>();
            ////// Calcola la coppia del controllore PD
            //Eigen::VectorXd tau_fb = Kp * (q_desired - q_current) + Kd * (qd_desired - qd_current);
            //
            //// Coppia totale da applicare (PD + compensazione gravità)
            //Eigen::VectorXd tau_cmd = pin_data.nle + M * (tau_fb + qdd_desired);
            ////std::cout << "size g: " << g.size() << " size mujoco u:" << mj_model_ptr->nu<< std::endl;
            //// Applica le coppie agli attuatori di MuJoCo
            //for (int i = 0; i < mj_model_ptr->nu; ++i) {
            //    mj_data_ptr->ctrl[i] = tau_cmd[i];
            //}

            // --- 3.3 Avanzamento della Simulazione ---
            mj_step(mj_model_ptr, mj_data_ptr);
            std::cout << "time: " << mj_data_ptr->time << std::endl;

            // --- (Opzionale) Gestione del tempo per esecuzione in tempo reale ---
            // Se vuoi che la simulazione giri in tempo reale e non il più velocemente possibile.
            auto loop_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> loop_duration = loop_end - loop_start;
            if (loop_duration.count() < simulation_step) {
                std::this_thread::sleep_for(std::chrono::duration<double>(simulation_step - loop_duration.count()));
            }
            // Render
            
        }
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