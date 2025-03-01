#include "MavlinkToLattice.hpp"
#include <filesystem>
#include <signal.h>
#include <iostream>
#include <toml.hpp>
#include <unistd.h>
#include <sys/types.h>

static void signal_handler(int signum);

std::shared_ptr<MavlinkToLattice> _mavlink_to_lattice;

int main()
{
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	setbuf(stdout, NULL); // Disable stdout buffering

	toml::table config;

	try {
		config = toml::parse_file(std::string(getenv("HOME")) + "/.local/share/polaris/config.toml");

	} catch (const toml::parse_error& err) {
		std::cerr << "Parsing failed:\n" << err << "\n";
		return -1;

	} catch (const std::exception& err) {
		std::cerr << "Error: " << err.what() << "\n";
		return -1;
	}

	MavlinkToLattice::Settings settings = {
		.mavsdk_connection_url = config["connection_url"].value_or("0.0.0"),
		.polaris_api_key = config["polaris_api_key"].value_or("<your_key_goes_here>")
	};

	_mavlink_to_lattice = std::make_shared<MavlinkToLattice>(settings);

	_mavlink_to_lattice->run();

	std::cout << "exiting" << std::endl;

	return 0;
}

static void signal_handler(int signum)
{
	if (_mavlink_to_lattice.get()) _mavlink_to_lattice->stop();
}
