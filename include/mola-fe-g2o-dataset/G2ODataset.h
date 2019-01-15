/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   G2ODataset.h
 * @brief  "Fake" SLAM FrontEnd for reading from a G2O dataset file
 * @author Jose Luis Blanco Claraco
 * @date   Jan 15, 2018
 */
#pragma once

#include <mola-kernel/FrontEndBase.h>
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <list>

namespace mola
{
/** "Fake" SLAM FrontEnd for reading from a G2O dataset file
 * Send a new observation at each spinOnce() call.
 * That is, the rate [Hz] at which edges are sent to the SLAM back-end
 * is the execution rate of this module (per the YAML config file).
 *
 * \ingroup mola_fe_g2o_dataset_grp */
class G2ODataset : public FrontEndBase
{
   public:
    G2ODataset();
    virtual ~G2ODataset() override = default;

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;

    void onNewObservation(CObservation::Ptr&) override
    {
        THROW_EXCEPTION(
            "Not an actual front-end! Cannot accept real sensor observations.")
    }

   private:
    using graph_t         = mrpt::graphs::CNetworkOfPoses3DInf;
    using edge_value_type = graph_t::edges_map_t::value_type;
    using sorted_dataset_t =
        std::map<mrpt::graphs::TNodeID, std::list<edge_value_type>>;

    std::string                                 g2o_file_;
    std::map<mrpt::graphs::TNodeID, mola::id_t> g2o_to_mola_IDs_;

    sorted_dataset_t                 dataset_sorted_;
    sorted_dataset_t::const_iterator next_to_publish_;

    mola::id_t create_KF_if_new(const mrpt::graphs::TNodeID id);
    void       create_edge(
              const mola::id_t from, const mola::id_t to,
              const mrpt::poses::CPose3DPDFGaussianInf& edge);
};

}  // namespace mola
