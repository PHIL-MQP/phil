#include <cstdio>
#include <time.h>
#include <unistd.h>

#include <phil/common/args.h>
#include <phil/common/common.h>
#include <phil/common/udp.h>
#include <phil/common/csv.h>

int main(int argc, const char **argv) {
  args::ArgumentParser parser("re-published a CSV file so the phil_main server to run without a real RoboRIO");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::Positional<std::string> rio_csv(parser, "rio_csv", "csv file of data logged on the roborio", args::Options::Required);
  args::Positional<std::string> kobuki_csv(parser, "rio_csv", "csv file of data logged on the kobuki", args::Options::Required);
  args::Flag step_flag(parser, "step", "press enter to publish each line/packet of the data", {'s', "step"});
  args::ValueFlag<unsigned int>
      period_flag(parser, "period", "publish a new packet of data every [period] milliseconds", {'p', "period"});

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

  unsigned int rate = args::get(period_flag);

  io::CSVReader<4> kobuki_reader(args::get(kobuki_csv));
  kobuki_reader.read_header(io::ignore_extra_column,
                         "left_ticks",
                         "right_ticks");
  io::CSVReader<6> rio_reader(args::get(rio_csv));
  rio_reader.read_header(io::ignore_extra_column,
                     "raw_accel_x",
                     "raw_accel_y",
                     "raw_accel_z",
                     "yaw",
                     "fpga time",
                     "navx time");

  phil::UDPClient client("localhost");
  client.SetTimeout({0, 50000});

  double ax, ay, az, yaw, ticks_l, ticks_r, fpga_t;
  long navx_t;
  while (rio_reader.read_row(ax, ay, az, yaw, fpga_t, navx_t) && kobuki_reader.read_row(ticks_l, ticks_r)) {
    phil::data_t data{};
    data.raw_acc_x = ax;
    data.raw_acc_y = ay;
    data.raw_acc_z = az;
    data.yaw = yaw;
    // FIXME: figure out a way to stich together encoder data since it's at a slightly different rate
    data.left_encoder_rate = 0;
    data.right_encoder_rate = 0;
    data.fpga_t = fpga_t;
    data.navx_t = navx_t;

    phil::data_t response = client.Transaction(data);
    phil::print_data_t(response);

    if (args::get(step_flag)) {
      std::cin.get();
    } else if (rate == 0) {
      // do nothing
      continue;
    } else {
      usleep(rate * 1000);
    }
  }

  return EXIT_SUCCESS;
}
