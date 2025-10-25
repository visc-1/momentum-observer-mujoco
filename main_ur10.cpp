#include <mujoco/mujoco.h>
#include "MujocoUI.hpp"
#include <iostream>
#include <memory>
#include <filesystem>

int main(int argc, char** argv)
{
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
    std::cout << "Number of DoFs: " << mj_model_ptr->nv << std::endl;
    std::cout << "Number of joints: " << mj_model_ptr->njnt << std::endl;
    
    // Create MuJoCo data
    mjData* mj_data_ptr = mj_makeData(mj_model_ptr);
    
    // Set initial joint positions (home configuration)
    // UR10 has 6 revolute joints
    // Set to a reasonable home pose
    for (int i = 0; i < mj_model_ptr->nq; i++) {
        mj_data_ptr->qpos[i] = 0.0;
    }
    
    // Forward kinematics to update positions
    mj_forward(mj_model_ptr, mj_data_ptr);
    
    // Initialize MuJoCo UI
    MujocoUI* mujoco_ui = MujocoUI::getInstance(mj_model_ptr, mj_data_ptr);
    
    std::cout << "Starting visualization. Press ESC or close window to exit." << std::endl;
    
    // Main rendering loop
    while (!mujoco_ui->windowShouldClose()) {
        // Step simulation (passive dynamics with gravity)
        mj_step(mj_model_ptr, mj_data_ptr);
        
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
