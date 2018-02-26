#include <cstdio>
#include <time.h>

#include <phil/common/args.h>
#include <phil/common/common.h>
#include <phil/common/udp.h>
#include <phil/common/csv.h>

int main(int argc, const char **argv) {
  args::ArgumentParser parser("re-published a CSV file so the phil_main server to run without a real RoboRIO");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::Positional<std::string>
      rio_csv(parser, "rio_csv", "csv file of data logged on the roborio", args::Options::Required);
  args::Flag step_flag(parser, "step", "press enter to publish each line/packet of the data", {'s', "step"});

  try {
    parser.ParseCLI(argc, argv);
  }
  catch (args::Help &e) {
    std::cout << parser;
    return EXIT_SUCCESS;
  }
  catch (args::RequiredError &e) {
    std::cout << parser;
    return EXIT_FAILURE;
  }

  io::CSVReader<7> reader(args::get(rio_csv));
  reader.read_header(io::ignore_extra_column,
                     "world_accel_x",
                     "world_accel_y",
                     "yaw",
                     "left_encoder_rate",
                     "right_encoder_rate",
                     "fpga time",
                     "navx time");

  phil::UDPClient client("localhost");
  client.SetTimeout({0, 20000});

  double ax, ay, yaw, encoder_l, encoder_r, fpga_t;
  long navx_t;
  while (reader.read_row(ax, ay, yaw, encoder_l, encoder_r, fpga_t, navx_t)) {
    phil::data_t data{};
    data.world_accel_x = ax;
    data.world_accel_y = ay;
    data.yaw = yaw;
    data.left_encoder_rate = encoder_l;
    data.right_encoder_rate = encoder_r;
    data.fpga_t = fpga_t;
    data.navx_t = navx_t;

    phil::data_t response = client.Transaction(data);
    phil::print_data_t(response);

    if (args::get(step_flag)) {
      std::cin.get();
    } else {
      struct timespec deadline{};
      deadline.tv_sec = 0;
      deadline.tv_nsec = 20000000;
      clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
    }
  }

  return EXIT_SUCCESS;
}
