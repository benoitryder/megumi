#include <iostream>
#include <boost/program_options.hpp>
#include "log.h"
#include "input_hex.h"
#include "model/x128a1.h"


namespace po = boost::program_options;


static void usage(const char* progname, const po::options_description& opts)
{
  std::cout
      << "usage: " << progname << " [options] file" << std::endl
      << std::endl
      << opts << std::endl
      ;
}


int main(int argc, char* argv[])
{
  try {
    // parse command line
    po::options_description opts("Options");
    opts.add_options()
        ("help,h", "this help")
        ;

    po::options_description opts_hidden; // hidden
    opts_hidden.add_options()
        ("input-file", po::value<std::string>(), "input file")
        ;
    po::positional_options_description popts;
    popts.add("input-file", 1);

    po::options_description opts_all;
    opts_all.add(opts).add(opts_hidden);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv)
              .options(opts_all).positional(popts).run(), vm);
    po::notify(vm);

    // process command line parameters
    if(vm.count("help")) {
      usage(argv[0], opts);
      return 0;
    } else if(vm.count("input-file") == 0) {
      std::cerr << "missing input file" << std::endl;
      return 1;
    }

    Log::setMinimumSeverity(Log::INFO);
    LOG(NOTICE) << "logging start";

    model::ATxmega128A1 device;
    DLOG(NOTICE) << "device created";
    std::vector<uint8_t> progdata = parse_hex_file(vm["input-file"].as<std::string>());
    device.loadFlash(progdata);
    DLOG(NOTICE) << "flash data loaded";
    device.reset();
    for(int i=0; i<1000; i++) {
      device.step();
    }

  } catch(const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  } catch(...) {
    std::cerr << "unknown error" << std::endl;
  }
}

