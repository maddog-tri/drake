/// @file yaml_to_obj.cc
///
/// Take a yaml file as input, build the resulting monolane road geometry, and
/// render the road surface to a WaveFront OBJ output file.
#include <iostream>
#include <string>

#include <gflags/gflags.h>

#include "drake/automotive/maliput/monolane/loader.h"
#include "drake/automotive/maliput/utility/generate_obj.h"

namespace mono = drake::maliput::monolane;
namespace utility = drake::maliput::utility;

DEFINE_string(yaml_file, "",
              "yaml input file defining a monolane road geometry");
DEFINE_string(obj_dir, ".", "Directory to contain rendered road surface");
DEFINE_string(obj_file, "",
              "Basename for output Wavefront OBJ and MTL files");
DEFINE_double(max_grid_unit, utility::ObjFeatures().max_grid_unit,
              "Maximum size of a grid unit in the rendered mesh covering the"
              " road surface");
DEFINE_double(min_grid_resolution, utility::ObjFeatures().min_grid_resolution,
              "Minimum number of grid-units in either lateral or longitudinal"
              " direction in the rendered mesh covering the road surface");

int main(int argc, char* argv[]) {
  std::cerr << "main() !\n";
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_yaml_file.empty()) {
    std::cerr << "no input file!\n";
    return 1;
  }
  if (FLAGS_obj_file.empty()) {
    std::cerr << "no output file!\n";
    return 1;
  }
  std::cerr << "loading road geometry !\n";
  auto rg = mono::LoadFile(FLAGS_yaml_file);


  utility::ObjFeatures features;
  features.max_grid_unit = FLAGS_max_grid_unit;
  features.min_grid_resolution = FLAGS_min_grid_resolution;

  std::cerr << "generating obj !\n";
  utility::GenerateObjFile(rg.get(), FLAGS_obj_dir, FLAGS_obj_file, features);

  return 0;
}
