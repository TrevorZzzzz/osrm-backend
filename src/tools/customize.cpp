#include "customizer/customizer.hpp"

#include "osrm/exception.hpp"
#include "util/log.hpp"
#include "util/meminfo.hpp"
#include "util/version.hpp"

#include "util/program_options_path.hpp"
#include <boost/program_options.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <set>
#include <thread>

using namespace osrm;

enum class return_code : unsigned
{
    ok,
    fail,
    exit
};

namespace
{
return_code parseMetricSpecs(const std::vector<std::string> &raw_specs,
                             customizer::CustomizationConfig &customization_config)
{
    std::set<std::string> seen_names;
    for (const auto &raw : raw_specs)
    {
        customizer::MetricCustomizationSpec spec;
        std::size_t begin = 0;
        auto next = raw.find(':');
        spec.name = raw.substr(begin, next == std::string::npos ? next : next - begin);
        while (next != std::string::npos)
        {
            begin = next + 1;
            next = raw.find(':', begin);
            auto csv = raw.substr(begin, next == std::string::npos ? next : next - begin);
            if (!csv.empty())
                spec.segment_speed_lookup_paths.push_back(csv);
        }
        const bool valid_name =
            !spec.name.empty() &&
            std::all_of(spec.name.begin(),
                        spec.name.end(),
                        [](unsigned char c)
                        { return std::isalnum(c) || c == '_' || c == '-'; });
        if (!valid_name)
        {
            util::Log(logERROR) << "Invalid metric name in --metric " << raw
                                << " (allowed: [A-Za-z0-9_-]+)";
            return return_code::fail;
        }
        if (!seen_names.insert(spec.name).second)
        {
            util::Log(logERROR) << "Duplicate metric name: " << spec.name;
            return return_code::fail;
        }
        customization_config.metrics.push_back(std::move(spec));
    }
    return return_code::ok;
}
} // namespace

return_code parseArguments(int argc,
                           char *argv[],
                           std::string &verbosity,
                           customizer::CustomizationConfig &customization_config)
{
    std::vector<std::string> metric_specs;
    // declare a group of options that will be allowed only on command line
    boost::program_options::options_description generic_options("Options");
    generic_options.add_options()("version,v", "Show version")("help,h", "Show this help message")(
        "list-inputs", "List required and optional input file extensions")(
        "verbosity,l",
        boost::program_options::value<std::string>(&verbosity)->default_value("INFO"),
        std::string("Log verbosity level: " + util::LogPolicy::GetLevels()).c_str());

    // declare a group of options that will be allowed both on command line
    boost::program_options::options_description config_options("Configuration");
    config_options.add_options()
        //
        ("threads,t",
         boost::program_options::value<unsigned int>(&customization_config.requested_num_threads)
             ->default_value(std::thread::hardware_concurrency()),
         "Number of threads to use")(
            "segment-speed-file",
            boost::program_options::value<std::vector<std::string>>(
                &customization_config.updater_config.segment_speed_lookup_paths)
                ->composing(),
            "Lookup files containing nodeA, nodeB, speed data to adjust edge weights")(
            "turn-penalty-file",
            boost::program_options::value<std::vector<std::string>>(
                &customization_config.updater_config.turn_penalty_lookup_paths)
                ->composing(),
            "Lookup files containing from_, to_, via_nodes, and turn penalties to adjust turn "
            "weights")("edge-weight-updates-over-factor",
                       boost::program_options::value<double>(
                           &customization_config.updater_config.log_edge_updates_factor)
                           ->default_value(0.0),
                       "Use with `--segment-speed-file`. Provide an `x` factor, by which Extractor "
                       "will log edge "
                       "weights updated by more than this factor")(
            "parse-conditionals-from-now",
            boost::program_options::value<std::time_t>(
                &customization_config.updater_config.valid_now)
                ->default_value(0),
            "Optional for conditional turn restriction parsing, provide a UTC time stamp from "
            "which "
            "to evaluate the validity of conditional turn restrictions")(
            "time-zone-file",
            boost::program_options::value<std::string>(
                &customization_config.updater_config.tz_file_path)
                ->default_value(""),
            "Required for conditional turn restriction parsing, provide a geojson file containing "
            "time zone boundaries")(
            "output,o",
            boost::program_options::value<std::filesystem::path>(&customization_config.output_path),
            "Output base path for generated files (default: same as input)")(
            "metric",
            boost::program_options::value<std::vector<std::string>>(&metric_specs)->composing(),
            "Named metric spec '<name>[:<segment-speed-file>...]', repeatable. Creates one "
            "dataset serving several weight metrics; the first metric is the default. "
            "Incompatible with --segment-speed-file, --turn-penalty-file and "
            "--parse-conditionals-from-now");

    // hidden options, will be allowed on command line, but will not be
    // shown to the user
    boost::program_options::options_description hidden_options("Hidden options");
    hidden_options.add_options()(
        "input,i",
        boost::program_options::value<std::filesystem::path>(&customization_config.base_path),
        "Input base file path");

    // positional option
    boost::program_options::positional_options_description positional_options;
    positional_options.add("input", 1);

    // combine above options for parsing
    boost::program_options::options_description cmdline_options;
    cmdline_options.add(generic_options).add(config_options).add(hidden_options);

    const auto *executable = argv[0];
    boost::program_options::options_description visible_options(
        std::filesystem::path(executable).filename().string() + " <input.osrm> [options]");
    visible_options.add(generic_options).add(config_options);

    // parse command line options
    boost::program_options::variables_map option_variables;
    try
    {
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
                                          .options(cmdline_options)
                                          .positional(positional_options)
                                          .run(),
                                      option_variables);
    }
    catch (const boost::program_options::error &e)
    {
        util::Log(logERROR) << e.what();
        return return_code::fail;
    }

    if (option_variables.contains("version"))
    {
        std::cout << OSRM_VERSION << std::endl;
        return return_code::exit;
    }

    if (option_variables.contains("help"))
    {
        std::cout << visible_options;
        return return_code::exit;
    }

    if (option_variables.contains("list-inputs"))
    {
        customizer::CustomizationConfig config;
        std::set<std::string> seen;
        config.ListInputFiles(std::cout, seen);
        config.updater_config.ListInputFiles(std::cout, seen);
        return return_code::exit;
    }

    boost::program_options::notify(option_variables);

    if (!option_variables.contains("input"))
    {
        std::cout << visible_options;
        return return_code::fail;
    }

    if (!metric_specs.empty())
    {
        if (!customization_config.updater_config.segment_speed_lookup_paths.empty() ||
            !customization_config.updater_config.turn_penalty_lookup_paths.empty() ||
            customization_config.updater_config.valid_now != 0)
        {
            util::Log(logERROR) << "--metric cannot be combined with --segment-speed-file, "
                                   "--turn-penalty-file or --parse-conditionals-from-now";
            return return_code::fail;
        }
        return parseMetricSpecs(metric_specs, customization_config);
    }

    return return_code::ok;
}

int main(int argc, char *argv[])
try
{
    util::LogPolicy::GetInstance().Unmute();
    std::string verbosity;
    customizer::CustomizationConfig customization_config;

    const auto result = parseArguments(argc, argv, verbosity, customization_config);

    if (return_code::fail == result)
    {
        return EXIT_FAILURE;
    }

    if (return_code::exit == result)
    {
        return EXIT_SUCCESS;
    }

    util::LogPolicy::GetInstance().SetLevel(verbosity);

    // set the default in/output names
    customization_config.UseDefaultOutputNames(customization_config.base_path);

    if (!customization_config.output_path.empty())
    {
        // Strip known extensions from the user-provided output path
        std::string path = customization_config.output_path.string();
        const std::array<std::string, 6> known_extensions{
            {".osm.bz2", ".osm.pbf", ".osm.xml", ".pbf", ".osm", ".osrm"}};
        for (const auto &ext : known_extensions)
        {
            const auto pos = path.find(ext);
            if (pos != std::string::npos)
            {
                path.replace(pos, ext.size(), "");
                break;
            }
        }
        customization_config.output_path = path;
    }

    if (1 > customization_config.requested_num_threads)
    {
        util::Log(logERROR) << "Number of threads must be 1 or larger";
        return EXIT_FAILURE;
    }

    if (!customization_config.IsValid())
    {
        return EXIT_FAILURE;
    }

    auto exitcode = customizer::Customizer().Run(customization_config);

    util::DumpMemoryStats();

    return exitcode;
}
catch (const osrm::RuntimeError &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << e.what();
    return e.GetCode();
}
catch (const util::exception &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << e.what();
    return EXIT_FAILURE;
}
catch (const std::bad_alloc &e)
{
    util::DumpMemoryStats();
    util::Log(logERROR) << "[exception] " << e.what();
    util::Log(logERROR) << "Please provide more memory or consider using a larger swapfile";
    return EXIT_FAILURE;
}
#ifdef _WIN32
catch (const std::exception &e)
{
    util::Log(logERROR) << "[exception] " << e.what() << std::endl;
    return EXIT_FAILURE;
}
#endif
