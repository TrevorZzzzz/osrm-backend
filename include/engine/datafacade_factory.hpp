#ifndef OSRM_ENGINE_DATAFACADE_FACTORY_HPP
#define OSRM_ENGINE_DATAFACADE_FACTORY_HPP

#include "extractor/class_data.hpp"
#include "extractor/profile_properties.hpp"

#include "engine/algorithm.hpp"
#include "engine/api/base_parameters.hpp"
#include "engine/api/tile_parameters.hpp"

#include "util/integer_range.hpp"

#include "storage/shared_datatype.hpp"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace osrm::engine
{
// This class selects the right facade for
template <template <typename A> class FacadeT, typename AlgorithmT> class DataFacadeFactory
{
    static constexpr auto has_exclude_flags = routing_algorithms::HasExcludeFlags<AlgorithmT>{};

  public:
    using Facade = FacadeT<AlgorithmT>;
    DataFacadeFactory() = default;

    template <typename AllocatorT>
    DataFacadeFactory(const std::shared_ptr<AllocatorT> &allocator)
        : DataFacadeFactory(std::move(allocator), has_exclude_flags)
    {
        BOOST_ASSERT_MSG(facades.size() >= 1, "At least one datafacade is needed");
    }

    template <typename ParameterT> std::shared_ptr<const Facade> Get(const ParameterT &params) const
    {
        return Get(params, has_exclude_flags);
    }

  private:
    // Algorithm with exclude flags
    template <typename AllocatorT>
    DataFacadeFactory(const std::shared_ptr<AllocatorT> &allocator, std::true_type)
    {
        const auto &index = allocator->GetIndex();
        properties = index.template GetBlockPtr<extractor::ProfileProperties>("/common/properties");
        default_metric_name = properties->GetWeightName();

        std::vector<std::string> metric_prefixes;
        auto metrics_path =
            std::string("/") + routing_algorithms::identifier<AlgorithmT>() + "/metrics/";
        index.List(metrics_path, std::back_inserter(metric_prefixes));

        for (const auto &metric_prefix : metric_prefixes)
        {
            const auto metric_name = metric_prefix.substr(metrics_path.size());

            std::vector<std::string> exclude_prefixes;
            auto exclude_path = metric_prefix + "/exclude/";
            index.List(exclude_path, std::back_inserter(exclude_prefixes));
            auto &metric_facades = facades[metric_name];
            metric_facades.resize(exclude_prefixes.size());

            for (const auto &exclude_prefix : exclude_prefixes)
            {
                auto index_begin = exclude_prefix.find_last_of('/');
                BOOST_ASSERT_MSG(index_begin != std::string::npos,
                                 "The exclude prefix needs to be a valid data path.");
                std::size_t exclude_index =
                    std::stoi(exclude_prefix.substr(index_begin + 1, exclude_prefix.size()));
                BOOST_ASSERT(exclude_index < metric_facades.size());
                metric_facades[exclude_index] =
                    std::make_shared<const Facade>(allocator, metric_name, exclude_index);
            }
        }

        if (facades.count(default_metric_name) == 0 || facades[default_metric_name].empty())
        {
            throw util::exception(std::string("Could not find any metrics for ") +
                                  routing_algorithms::name<AlgorithmT>() +
                                  " in the data. Did you load the right dataset?");
        }

        for (const auto index : util::irange<std::size_t>(0, properties->class_names.size()))
        {
            const std::string name = properties->GetClassNameForIndex(index);
            if (!name.empty())
            {
                name_to_class[name] = extractor::getClassData(index);
            }
        }
    }

    // Algorithm without exclude flags
    template <typename AllocatorT>
    DataFacadeFactory(std::shared_ptr<AllocatorT> allocator, std::false_type)
    {
        const auto &index = allocator->GetIndex();
        properties = index.template GetBlockPtr<extractor::ProfileProperties>("/common/properties");
        default_metric_name = properties->GetWeightName();
        facades[default_metric_name].push_back(
            std::make_shared<const Facade>(allocator, default_metric_name, 0));
    }

    const std::vector<std::shared_ptr<const Facade>> *
    FindMetricFacades(const std::optional<std::string> &requested_metric) const
    {
        const auto &metric_name =
            requested_metric.has_value() ? *requested_metric : default_metric_name;
        auto iter = facades.find(metric_name);
        if (iter == facades.end() || iter->second.empty())
        {
            return nullptr;
        }
        return &iter->second;
    }

    std::shared_ptr<const Facade> Get(const api::TileParameters &, std::false_type) const
    {
        return facades.at(default_metric_name)[0];
    }

    // Default for non-exclude flags: return only facade
    std::shared_ptr<const Facade> Get(const api::BaseParameters &params, std::false_type) const
    {
        if (!params.exclude.empty())
        {
            return {};
        }
        const auto *metric_facades = FindMetricFacades(params.metric);
        if (metric_facades == nullptr)
        {
            return {};
        }

        return (*metric_facades)[0];
    }

    // TileParameters don't drive from BaseParameters and generally don't have use for exclude flags
    std::shared_ptr<const Facade> Get(const api::TileParameters &, std::true_type) const
    {
        return facades.at(default_metric_name)[0];
    }

    // Selection logic for finding the corresponding datafacade for the given parameters
    std::shared_ptr<const Facade> Get(const api::BaseParameters &params, std::true_type) const
    {
        const auto *metric_facades = FindMetricFacades(params.metric);
        if (metric_facades == nullptr)
        {
            return {};
        }
        if (params.exclude.empty())
            return (*metric_facades)[0];

        extractor::ClassData mask = 0;
        for (const auto &name : params.exclude)
        {
            auto class_mask_iter = name_to_class.find(name);
            if (class_mask_iter == name_to_class.end())
            {
                return {};
            }
            else
            {
                mask |= class_mask_iter->second;
            }
        }

        auto exclude_iter = std::find(
            properties->excludable_classes.begin(), properties->excludable_classes.end(), mask);
        if (exclude_iter != properties->excludable_classes.end())
        {
            auto exclude_index =
                std::distance(properties->excludable_classes.begin(), exclude_iter);
            if (static_cast<std::size_t>(exclude_index) >= metric_facades->size())
            {
                return {};
            }
            return (*metric_facades)[exclude_index];
        }

        return {};
    }

    std::unordered_map<std::string, std::vector<std::shared_ptr<const Facade>>> facades;
    std::unordered_map<std::string, extractor::ClassData> name_to_class;
    std::string default_metric_name;
    const extractor::ProfileProperties *properties = nullptr;
};
} // namespace osrm::engine

#endif
