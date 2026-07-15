#include "extractor/files.hpp"
#include "extractor/node_data_container.hpp"

#include "customizer/cell_customizer.hpp"
#include "customizer/customizer.hpp"
#include "customizer/edge_based_graph.hpp"
#include "customizer/files.hpp"

#include "partitioner/cell_statistics.hpp"
#include "partitioner/cell_storage.hpp"
#include "partitioner/edge_based_graph_reader.hpp"
#include "partitioner/files.hpp"
#include "partitioner/multi_level_partition.hpp"

#include "storage/shared_memory_ownership.hpp"

#include "updater/updater.hpp"

#include "util/exclude_flag.hpp"
#include "util/log.hpp"
#include "util/timing_util.hpp"

#include <boost/assert.hpp>

#include <tbb/global_control.h>

#include <map>
#include <optional>

namespace osrm::customizer
{

namespace
{

template <typename Partition, typename CellStorage>
void printUnreachableStatistics(const Partition &partition,
                                const CellStorage &storage,
                                const CellMetric &metric)
{
    util::Log() << "Unreachable nodes statistics per level";

    for (std::size_t level = 1; level < partition.GetNumberOfLevels(); ++level)
    {
        auto num_cells = partition.GetNumberOfCells(level);
        std::size_t invalid_sources = 0;
        std::size_t invalid_destinations = 0;
        for (std::uint32_t cell_id = 0; cell_id < num_cells; ++cell_id)
        {
            const auto &cell = storage.GetCell(metric, level, cell_id);
            for (auto node : cell.GetSourceNodes())
            {
                const auto &weights = cell.GetOutWeight(node);
                invalid_sources +=
                    std::all_of(weights.begin(),
                                weights.end(),
                                [](auto weight) { return weight == INVALID_EDGE_WEIGHT; });
            }
            for (auto node : cell.GetDestinationNodes())
            {
                const auto &weights = cell.GetInWeight(node);
                invalid_destinations +=
                    std::all_of(weights.begin(),
                                weights.end(),
                                [](auto weight) { return weight == INVALID_EDGE_WEIGHT; });
            }
        }

        if (invalid_sources > 0 || invalid_destinations > 0)
        {
            util::Log(logWARNING) << "Level " << level << " unreachable boundary nodes per cell: "
                                  << (invalid_sources / (float)num_cells) << " sources, "
                                  << (invalid_destinations / (float)num_cells) << " destinations";
        }
    }
}

auto LoadAndUpdateEdgeExpandedGraph(const CustomizationConfig &config,
                                    const partitioner::MultiLevelPartition &mlp,
                                    std::vector<EdgeWeight> &node_weights,
                                    std::vector<EdgeDuration> &node_durations,
                                    std::vector<EdgeDistance> &node_distances,
                                    std::uint32_t &connectivity_checksum,
                                    const std::vector<std::string> &segment_speed_lookup_paths,
                                    extractor::SegmentDataContainer *output_segment_data)
{
    updater::UpdaterConfig updater_config = config.updater_config;
    updater_config.segment_speed_lookup_paths = segment_speed_lookup_paths;
    updater::Updater updater(updater_config);

    std::vector<extractor::EdgeBasedEdge> edge_based_edge_list;
    EdgeID num_nodes = updater.LoadAndUpdateEdgeExpandedGraph(
        edge_based_edge_list, node_weights, node_durations, connectivity_checksum,
        output_segment_data);

    extractor::files::readEdgeBasedNodeDistances(config.GetPath(".osrm.enw"), node_distances);

    auto directed = partitioner::splitBidirectionalEdges(edge_based_edge_list);

    auto tidied =
        partitioner::prepareEdgesForUsageInGraph<partitioner::MultiLevelEdgeBasedGraph::InputEdge>(
            std::move(directed));

    auto edge_based_graph = partitioner::MultiLevelEdgeBasedGraph(mlp, num_nodes, tidied);

    return edge_based_graph;
}

auto LoadAndUpdateEdgeExpandedGraph(const CustomizationConfig &config,
                                    const partitioner::MultiLevelPartition &mlp,
                                    std::vector<EdgeWeight> &node_weights,
                                    std::vector<EdgeDuration> &node_durations,
                                    std::vector<EdgeDistance> &node_distances,
                                    std::uint32_t &connectivity_checksum)
{
    return LoadAndUpdateEdgeExpandedGraph(config,
                                          mlp,
                                          node_weights,
                                          node_durations,
                                          node_distances,
                                          connectivity_checksum,
                                          config.updater_config.segment_speed_lookup_paths,
                                          nullptr);
}

std::vector<CellMetric> customizeFilteredMetrics(const partitioner::MultiLevelEdgeBasedGraph &graph,
                                                 const partitioner::CellStorage &storage,
                                                 const CellCustomizer &customizer,
                                                 const std::vector<std::vector<bool>> &node_filters)
{
    std::vector<CellMetric> metrics;
    metrics.reserve(node_filters.size());

    for (const auto &filter : node_filters)
    {
        auto metric = storage.MakeMetric();
        customizer.Customize(graph, storage, filter, metric);
        metrics.push_back(std::move(metric));
    }

    return metrics;
}

void maskNodeWeightFlags(std::vector<EdgeWeight> &node_weights)
{
    std::for_each(
        node_weights.begin(), node_weights.end(), [](auto &w) { w &= EdgeWeight{0x7fffffff}; });
}

int RunMultiMetricCustomization(const CustomizationConfig &config,
                                const partitioner::MultiLevelPartition &mlp,
                                const partitioner::CellStorage &storage,
                                const extractor::EdgeBasedNodeDataContainer &node_data,
                                extractor::ProfileProperties &properties)
{
    BOOST_ASSERT(!config.metrics.empty());

    std::unordered_map<std::string, std::vector<CellMetric>> metric_exclude_classes;
    std::map<std::string, files::MetricNodeWeights> extra_node_metrics;
    std::map<std::string, extractor::MetricSegmentWeights> extra_segment_metrics;

    std::uint32_t connectivity_checksum = 0;
    std::vector<std::vector<bool>> filter;

    for (std::size_t spec_index = config.metrics.size(); spec_index-- > 0;)
    {
        const auto &spec = config.metrics[spec_index];
        const bool is_default = spec_index == 0;

        util::Log() << "Customizing metric '" << spec.name << "' ("
                    << spec.segment_speed_lookup_paths.size() << " segment speed files)";

        TIMER_START(metric_customize);
        extractor::SegmentDataContainer segment_data;
        std::vector<EdgeWeight> node_weights;
        std::vector<EdgeDuration> node_durations;
        std::vector<EdgeDistance> node_distances;
        auto graph = LoadAndUpdateEdgeExpandedGraph(config,
                                                    mlp,
                                                    node_weights,
                                                    node_durations,
                                                    node_distances,
                                                    connectivity_checksum,
                                                    spec.segment_speed_lookup_paths,
                                                    &segment_data);
        BOOST_ASSERT(graph.GetNumberOfNodes() == node_weights.size());
        maskNodeWeightFlags(node_weights);
        util::Log() << "Loaded edge based graph: " << graph.GetNumberOfEdges() << " edges, "
                    << graph.GetNumberOfNodes() << " nodes";

        if (filter.empty())
        {
            filter =
                util::excludeFlagsToNodeFilter(graph.GetNumberOfNodes(), node_data, properties);
        }

        auto cells = customizeFilteredMetrics(graph, storage, CellCustomizer{mlp}, filter);
        TIMER_STOP(metric_customize);
        util::Log() << "Metric '" << spec.name << "' customization took "
                    << TIMER_SEC(metric_customize) << " seconds";

        for (const auto &metric : cells)
        {
            printUnreachableStatistics(mlp, storage, metric);
        }
        metric_exclude_classes[spec.name] = std::move(cells);

        if (is_default)
        {
            TIMER_START(writing_mld_data);
            files::writeCellMetrics(config.GetOutputPath(".osrm.cell_metrics"),
                                    metric_exclude_classes);
            TIMER_STOP(writing_mld_data);
            util::Log() << "MLD customization writing took " << TIMER_SEC(writing_mld_data)
                        << " seconds";

            TIMER_START(writing_graph);
            MultiLevelEdgeBasedGraph shaved_graph{std::move(graph),
                                                  std::move(node_weights),
                                                  std::move(node_durations),
                                                  std::move(node_distances)};
            customizer::files::writeGraph(config.GetOutputPath(".osrm.mldgr"),
                                          shaved_graph,
                                          connectivity_checksum,
                                          extra_node_metrics);
            TIMER_STOP(writing_graph);
            util::Log() << "Graph writing took " << TIMER_SEC(writing_graph) << " seconds";

            TIMER_START(writing_segments);
            extractor::files::writeSegmentData(
                config.updater_config.GetPath(".osrm.geometry"), segment_data,
                extra_segment_metrics);
            TIMER_STOP(writing_segments);
            util::Log() << "Segment data writing took " << TIMER_SEC(writing_segments)
                        << " seconds";

            updater::UpdaterConfig updater_config = config.updater_config;
            updater_config.segment_speed_lookup_paths = spec.segment_speed_lookup_paths;
            updater::Updater(updater_config).SaveDatasourcesNames();

            properties.SetWeightName(spec.name);
            extractor::files::writeProfileProperties(config.GetPath(".osrm.properties"),
                                                     properties);
            util::Log() << "Default metric is '" << spec.name << "'";
        }
        else
        {
            auto [fwd_weights, rev_weights, fwd_durations, rev_durations] =
                segment_data.TakeWeightsAndDurations();
            extra_segment_metrics.emplace(spec.name,
                                          extractor::MetricSegmentWeights{
                                              std::move(fwd_weights),
                                              std::move(rev_weights),
                                              std::move(fwd_durations),
                                              std::move(rev_durations)});
            extra_node_metrics.emplace(
                spec.name,
                files::MetricNodeWeights{std::move(node_weights), std::move(node_durations)});
        }
    }

    return 0;
}
} // namespace

int Customizer::Run(const CustomizationConfig &config)
{
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism,
                           config.requested_num_threads);

    TIMER_START(loading_data);

    partitioner::MultiLevelPartition mlp;
    partitioner::files::readPartition(config.GetPath(".osrm.partition"), mlp);

    partitioner::CellStorage storage;
    partitioner::files::readCells(config.GetPath(".osrm.cells"), storage);

    extractor::EdgeBasedNodeDataContainer node_data;
    extractor::files::readNodeData(config.GetPath(".osrm.ebg_nodes"), node_data);

    extractor::ProfileProperties properties;
    extractor::files::readProfileProperties(config.GetPath(".osrm.properties"), properties);

    TIMER_STOP(loading_data);
    util::Log() << "Loading partition data took " << TIMER_SEC(loading_data) << " seconds";

    if (!config.metrics.empty())
    {
        return RunMultiMetricCustomization(config, mlp, storage, node_data, properties);
    }

    std::vector<EdgeWeight> node_weights;
    std::vector<EdgeDuration> node_durations; // TODO: remove when durations are optional
    std::vector<EdgeDistance> node_distances; // TODO: remove when distances are optional
    std::uint32_t connectivity_checksum = 0;
    auto graph = LoadAndUpdateEdgeExpandedGraph(
        config, mlp, node_weights, node_durations, node_distances, connectivity_checksum);
    BOOST_ASSERT(graph.GetNumberOfNodes() == node_weights.size());
    maskNodeWeightFlags(node_weights);
    util::Log() << "Loaded edge based graph: " << graph.GetNumberOfEdges() << " edges, "
                << graph.GetNumberOfNodes() << " nodes";

    TIMER_START(cell_customize);
    auto filter = util::excludeFlagsToNodeFilter(graph.GetNumberOfNodes(), node_data, properties);
    auto metrics = customizeFilteredMetrics(graph, storage, CellCustomizer{mlp}, filter);
    TIMER_STOP(cell_customize);
    util::Log() << "Cells customization took " << TIMER_SEC(cell_customize) << " seconds";

    partitioner::printCellStatistics(mlp, storage);
    for (const auto &metric : metrics)
    {
        printUnreachableStatistics(mlp, storage, metric);
    }

    TIMER_START(writing_mld_data);
    std::unordered_map<std::string, std::vector<CellMetric>> metric_exclude_classes = {
        {properties.GetWeightName(), std::move(metrics)},
    };
    files::writeCellMetrics(config.GetOutputPath(".osrm.cell_metrics"), metric_exclude_classes);
    TIMER_STOP(writing_mld_data);
    util::Log() << "MLD customization writing took " << TIMER_SEC(writing_mld_data) << " seconds";

    TIMER_START(writing_graph);
    MultiLevelEdgeBasedGraph shaved_graph{std::move(graph),
                                          std::move(node_weights),
                                          std::move(node_durations),
                                          std::move(node_distances)};
    customizer::files::writeGraph(
        config.GetOutputPath(".osrm.mldgr"), shaved_graph, connectivity_checksum);
    TIMER_STOP(writing_graph);
    util::Log() << "Graph writing took " << TIMER_SEC(writing_graph) << " seconds";

    return 0;
}

} // namespace osrm::customizer
