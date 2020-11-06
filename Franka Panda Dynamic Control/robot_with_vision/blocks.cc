#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

namespace gazebo
{
class Factory : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    srand((unsigned)time(NULL));

    // Define Constants
    double r = 0.855;
    double ocx = 0.23728;
    double ocy = 0.001731;
    double fov = 1.047198;
    double block_width = 0.1;

    int num_blocks = 3;
    const char *color_array[3] = {"1 0 0 1", "0 1 0 1", "0 0 1 1"};
    double block_positions[2][num_blocks]; 

    for (int i = 0; i <  num_blocks; i++){
    // Set the initial po vector outside of the robot workspace
    double pox = 100;
    double poy = 100;

    do {
      double r_zero = 0;

      // Choose a random x-distance away from origin {o} within the robot workspace and infront of the camera
      while (r_zero < (ocx + block_width)) {
        double random = (double)rand()/(double)RAND_MAX;
        r_zero = r * random; 
      }

      // Calculate the x-distance in {c}
      double rcx = r_zero - ocx;

      // Choose a random angle between [-fov/2 fov/2]
      double alpha = fov*(double)rand()/(double)RAND_MAX - (fov/2);

      // Calulate the block location in {c}
      double pcx = rcx;
      double pcy = rcx * tan(alpha);

      // Calculate the block loction in {o}
      pox = r_zero;
      poy = pcy + ocy;

      // Compare block location to previously chosen block locations, make sure blocks do not collide
      if (i > 0) {
        for (int j = 0; j <= i-1; j++){
          double distx = block_positions[0][j] - pox;
          double disty = block_positions[1][j] - poy;
          if (sqrt((distx*distx)+(disty*disty))<block_width){
            // If the blocks are too close, look for another location
            pox = 100;
            poy = 100;
          }
        }
      }
    } while (sqrt((pox*pox)+(poy*poy)) > r);

    // Store the block location
    block_positions[0][i] = pox;
    block_positions[1][i] = poy;

    // Place the block in the world
    sdf::SDF boxSDF;
        boxSDF.SetFromString(
          "<sdf version ='1.4'>\
              <model name ='box'>\
                <pose>1 0 0 0 0 0</pose>\
                <link name ='link'>\
                  <pose>" + std::to_string(pox) + " " + std::to_string(poy) + " 0 0 0 0</pose>\
                  <collision name ='collision'>\
                    <geometry>\
                      <box><size>0.1 0.1 0.1</size></box>\
                    </geometry>\
                  </collision>\
                  <visual name ='visual'>\
                    <geometry>\
                      <box><size>0.1 0.1 0.1</size></box>\
                    </geometry>\
                    <material>\
                      <ambient> " + color_array[i] + " </ambient>\
                      <diffuse> 0 0 0 1 </diffuse>\
                      <specular> 0 0 0 0 </specular>\
                      <emissive> 0 0 0 1 </emissive>\
                    </material>\
                  </visual>\
                </link>\
              </model>\
            </sdf>");
        // Name the block.
        sdf::ElementPtr model = boxSDF.Root()->GetElement("model");
        model->GetAttribute("name")->SetFromString("block " + std::to_string(i));
        _parent->InsertModelSDF(boxSDF);
    }

    // Print the block locations
    for (int col = 0; col<num_blocks; col++){
        printf("Position (%f, %f)\n", block_positions[0][col], block_positions[1][col]);
    }
  } 
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}
