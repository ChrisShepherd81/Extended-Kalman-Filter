#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "EkfFileHandler.h"

#define GNU_PLOT 0

#if GNU_PLOT
#include "plot/gnuplot_i.hpp"
#include "plot/PlotData.hpp"
#endif

using Eigen::VectorXd;

using std::string;
using std::cout;
using std::endl;

void check_arguments(int argc, char* argv[]);

int main(int argc, char* argv[])
{
  check_arguments(argc, argv);
  string in_file_name_ = argv[1];
  string out_file_name_ = argv[2];

  EkfFileHandler fileHandler(in_file_name_, out_file_name_);

  if(!fileHandler.check_files())
    exit(EXIT_FAILURE);

  //Read in the measurements and ground truth from file
  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;
  fileHandler.read_file(measurement_pack_list, gt_pack_list);

  // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

#if GNU_PLOT
  PlotData plot_estimations("Estimations");
  PlotData plot_laser("Laser Measurements");
  PlotData plot_radar("Radar Measurements");
  PlotData plot_ground("Ground truth");
#endif

  //Call the EKF-based fusion
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    VectorXd estimation = fusionEKF.ekf.GetX();
    fileHandler.write_to_file(estimation);

#if GNU_PLOT
    plot_estimations.addPoint(estimation);
#endif

    // output the measurements
    if (measurement_pack_list[k].sensor_type == MeasurementPackage::LASER)
    {
      // output the measurements
      fileHandler.write_to_file(measurement_pack_list[k].values, 2);
#if GNU_PLOT
      plot_laser.addPoint(measurement_pack_list[k].values);
#endif
    }
    else if (measurement_pack_list[k].sensor_type == MeasurementPackage::RADAR)
    {
      // output the measurements in the cartesian coordinates
      float ro = measurement_pack_list[k].values(0);
      float phi = measurement_pack_list[k].values(1);
      double x = ro * cos(phi);
      double y = ro * sin(phi);

      fileHandler.write_to_file(x);
      fileHandler.write_to_file(y);

#if GNU_PLOT
      plot_radar.addPoint(x, y);
#endif
    }

    // output the ground truth packages
    fileHandler.write_to_file(gt_pack_list[k].values);
    fileHandler.write_to_file("\n");

#if GNU_PLOT
    plot_ground.addPoint(gt_pack_list[k].values);
#endif

    estimations.push_back(estimation);
    ground_truth.push_back(gt_pack_list[k].values);
  }

  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

#if GNU_PLOT
  Gnuplot gp;
  gp.set_legend("top left");

  gp.set_style("points pt 5 ps .5 lc rgb 'red'");
  gp.plot_xy(plot_ground.getAllX(), plot_ground.getAllY(), plot_ground.getTitle());

  gp.set_style("points pt 2 ps 0.5 lc rgb 'green'");
  gp.plot_xy(plot_radar.getAllX(), plot_radar.getAllY(), plot_radar.getTitle());

  gp.set_style("points pt 3 ps 0.5 lc rgb 'blue'");
  gp.plot_xy(plot_laser.getAllX(), plot_laser.getAllY(), plot_laser.getTitle());

  gp.set_style("points pt 1 ps 1 lc rgb 'black'");
  gp.plot_xy(plot_estimations.getAllX(), plot_estimations.getAllY(), plot_estimations.getTitle());

  std::cout << "Press 'Enter' to exit program.\n";
  std::cin.get();

  gp.remove_tmpfiles();
#endif
  return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
