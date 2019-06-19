/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   G2ODataset.cpp
 * @brief  "Fake" SLAM FrontEnd for reading from a G2O dataset file
 * @author Jose Luis Blanco Claraco
 * @date   Jan 15, 2018
 */

/** \defgroup mola_fe_g2o_dataset_grp mola_fe_g2o_dataset_grp
 * "Fake" SLAM FrontEnd for reading from a G2O dataset file
 *
 *
 */

#include <mola-fe-g2o-dataset/G2ODataset.h>
#include <mola-kernel/yaml_helpers.h>
#include <mrpt/core/initializer.h>
#include <yaml-cpp/yaml.h>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT_NS_PREFIX(G2ODataset, FrontEndBase, mola);

MRPT_INITIALIZER(do_register_G2ODataset) { MOLA_REGISTER_MODULE(G2ODataset); }

G2ODataset::G2ODataset() = default;

void G2ODataset::initialize(const std::string& cfg_block)
{
    MRPT_TRY_START

    // Load:
    auto c   = YAML::Load(cfg_block);
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    YAML_LOAD_MEMBER_REQ(g2o_file, std::string);

    MRPT_LOG_INFO_STREAM("Lg2o_dataset_feoading dataset file: " << g2o_file_);

    graph_t dataset;
    dataset.loadFromTextFile(g2o_file_);

    MRPT_LOG_INFO_STREAM(
        "Read: " << dataset.nodeCount() << " keyframes, " << dataset.edgeCount()
                 << " edges.");

    // sort the data so we can reproduce them by a logical temporal order:
    for (const auto& edge : dataset.edges)
    {
        const auto [g2o_id_from, g2o_id_to] = edge.first;
        const auto latest_id                = std::max(g2o_id_from, g2o_id_to);
        dataset_sorted_[latest_id].push_back(edge);
    }

    // Init:
    next_to_publish_ = dataset_sorted_.begin();

    MRPT_TRY_END
}
void G2ODataset::spinOnce()
{
    MRPT_TRY_START
    ProfilerEntry tleg(profiler_, "spinOnce");

    // Send new observation?
    if (next_to_publish_ == dataset_sorted_.end())
    {
        MRPT_LOG_THROTTLE_WARN(
            10.0,
            "End of dataset reached! Nothing else to publish (CTRL+C to quit)");
        return;
    }

    // Take next edges:
    const auto edges  = next_to_publish_->second;
    const auto cur_kf = next_to_publish_->first;
    ++next_to_publish_;

    MRPT_LOG_DEBUG_STREAM(
        "Current KF: " << cur_kf << " with " << edges.size() << " edges.");

    for (const auto& edge : edges)
    {
        const auto [g2o_id_from, g2o_id_to] = edge.first;

        // Create new KFs:
        const mola::id_t id_from = create_KF_if_new(g2o_id_from);
        const mola::id_t id_to   = create_KF_if_new(g2o_id_to);

        MRPT_LOG_DEBUG_STREAM(
            "G2O edge from #" << g2o_id_from << " ==> to #" << g2o_id_to
                              << " (MOLA IDs: #" << id_from << " ==> #" << id_to
                              << ")");

        // New SE(3) constraint:
        const mrpt::poses::CPose3DPDFGaussianInf& e = edge.second;
        create_edge(id_from, id_to, e);
    }

    // Advertise new localization:
    {
        BackEndBase::AdvertiseUpdatedLocalization_Input new_loc;
        new_loc.timestamp    = mrpt::Clock::now();
        new_loc.reference_kf = g2o_to_mola_IDs_.at(cur_kf);
        new_loc.pose         = mrpt::math::TPose3D::Identity();

        slam_backend_->advertiseUpdatedLocalization(new_loc);
    }

    MRPT_TRY_END
}

mola::id_t G2ODataset::create_KF_if_new(const mrpt::graphs::TNodeID id)
{
    MRPT_START

    if (auto it_id = g2o_to_mola_IDs_.find(id); it_id != g2o_to_mola_IDs_.end())
        return it_id->second;  // Already done!

    // 1) New KeyFrame
    BackEndBase::ProposeKF_Input kf;

    kf.timestamp = mrpt::Clock::now();

    std::future<BackEndBase::ProposeKF_Output> kf_out_fut;
    kf_out_fut = slam_backend_->addKeyFrame(kf);

    // Wait until it's executed:
    auto kf_out = kf_out_fut.get();

    ASSERT_(kf_out.success);
    ASSERT_(kf_out.new_kf_id && kf_out.new_kf_id.value() != mola::INVALID_ID);

    g2o_to_mola_IDs_[id] = kf_out.new_kf_id.value();

    return kf_out.new_kf_id.value();

    MRPT_END
}

void G2ODataset::create_edge(
    const mola::id_t from, const mola::id_t to,
    const mrpt::poses::CPose3DPDFGaussianInf& edge)
{
    MRPT_START

    std::future<BackEndBase::AddFactor_Output> factor_out_fut;

    MRPT_TODO("Take into account the covariance");

    MRPT_LOG_DEBUG_STREAM(
        "create_edge: from #" << from << " ==> to #" << to
                              << " relPose=" << edge.getMeanVal().asString());

    mola::Factor f;
    // SE(3) transformation:
    mola::FactorRelativePose3 fPose3(from, to, edge.getMeanVal().asTPose());
    f              = std::move(fPose3);
    factor_out_fut = slam_backend_->addFactor(f);

    // Wait until it's executed:
    auto factor_out = factor_out_fut.get();
    ASSERT_(factor_out.success);
    ASSERT_(
        factor_out.new_factor_id &&
        factor_out.new_factor_id != mola::INVALID_FID);

    MRPT_END
}
