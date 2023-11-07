#include <bs_models/global_mapping/global_map_refinement.h>

#include <fuse_core/transaction.h>
#include <fuse_graphs/hash_graph.h>

#include <beam_utils/filesystem.h>

#include <bs_common/utils.h>
#include <bs_models/lidar/scan_pose.h>
#include <bs_models/reloc/reloc_methods.h>

namespace bs_models::global_mapping {

using namespace reloc;
using namespace beam_matching;

void GlobalMapRefinement::Params::LoadJson(const std::string& config_path) {
  if (config_path.empty()) {
    BEAM_INFO("No config file provided to global map refinement, using default "
              "parameters.");
    return;
  }

  BEAM_INFO("Loading global map refinement config file: {}", config_path);

  nlohmann::json J;
  if (!beam::ReadJson(config_path, J)) {
    BEAM_ERROR("Unable to read global map refinement config");
    throw std::runtime_error{"Unable to read global map refinement config"};
  }

  beam::ValidateJsonKeysOrThrow(
      {"loop_closure", "submap_refinement", "submap_alignment"}, J);

  // load loop closure params
  nlohmann::json J_loop_closure = J["loop_closure"];
  beam::ValidateJsonKeysOrThrow({"candidate_search_config", "refinement_config",
                                 "local_mapper_covariance",
                                 "loop_closure_covariance"},
                                J_loop_closure);

  std::string candidate_search_config_rel =
      J_loop_closure["candidate_search_config"];
  if (!candidate_search_config_rel.empty()) {
    loop_closure.candidate_search_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), candidate_search_config_rel);
  }

  double lc_cov_dia = J_loop_closure["loop_closure_covariance"];
  double lm_cov_dia = J_loop_closure["local_mapper_covariance"];
  loop_closure.loop_closure_covariance =
      Eigen::Matrix<double, 6, 6>::Identity() * lc_cov_dia;
  loop_closure.local_mapper_covariance =
      Eigen::Matrix<double, 6, 6>::Identity() * lm_cov_dia;

  std::string refinement_config_rel = J_loop_closure["refinement_config"];
  if (!refinement_config_rel.empty()) {
    loop_closure.refinement_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), refinement_config_rel);
  }

  // load submap refinement params
  nlohmann::json J_submap_refinement = J["submap_refinement"];
  beam::ValidateJsonKeysOrThrow({"scan_registration_config", "matcher_config"},
                                J_submap_refinement);

  std::string scan_registration_config_rel =
      J_submap_refinement["scan_registration_config"];
  if (!scan_registration_config_rel.empty()) {
    submap_refinement.scan_registration_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), scan_registration_config_rel);
  }

  std::string matcher_config_rel = J_submap_refinement["matcher_config"];
  if (!matcher_config_rel.empty()) {
    submap_refinement.matcher_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), matcher_config_rel);
  }

  // load submap alignment params
  nlohmann::json J_submap_alignment = J["submap_alignment"];
  beam::ValidateJsonKeysOrThrow({"matcher_config"}, J_submap_alignment);
  matcher_config_rel = J_submap_alignment["matcher_config"];
  if (!matcher_config_rel.empty()) {
    submap_alignment.matcher_config = beam::CombinePaths(
        bs_common::GetBeamSlamConfigPath(), matcher_config_rel);
  }
}

GlobalMapRefinement::RegistrationResult::RegistrationResult(
    const Eigen::Matrix4d& Ti, const Eigen::Matrix4d& Tf) {
  Eigen::Matrix4d T_DIFF = beam::InvertTransform(Ti) * Tf;
  Eigen::Matrix3d R_DIFF = T_DIFF.block(0, 0, 3, 3);
  dR = beam::Rad2Deg(std::abs(Eigen::AngleAxis<double>(R_DIFF).angle()));
  dt = T_DIFF.block(0, 3, 3, 1).norm() * 1000;
}

void GlobalMapRefinement::Summary::Save(const std::string& output_path) const {
  nlohmann::json J;
  std::vector<nlohmann::json> J_submap_refinement;
  for (const auto& [stamp, result] : submap_refinement) {
    nlohmann::json J_result;
    J_result["dt_mm"] = result.dt;
    J_result["dR_deg"] = result.dR;
    J_result["sec"] = stamp.sec;
    J_result["nsec"] = stamp.nsec;
    J_submap_refinement.push_back(J_result);
  }
  J["submap_refinement"] = J_submap_refinement;

  std::vector<nlohmann::json> J_submap_alignment;
  for (const auto& [stamp, result] : submap_alignment) {
    nlohmann::json J_result;
    J_result["dt_mm"] = result.dt;
    J_result["dR_deg"] = result.dR;
    J_result["sec"] = stamp.sec;
    J_result["nsec"] = stamp.nsec;
    J_submap_alignment.push_back(J_result);
  }
  J["submap_alignment"] = J_submap_alignment;

  std::string summary_path = beam::CombinePaths(output_path, "summary.json");
  std::ofstream file(summary_path);
  file << std::setw(4) << J << std::endl;
}

GlobalMapRefinement::GlobalMapRefinement(const std::string& global_map_data_dir,
                                         const Params& params)
    : params_(params) {
  // load global map to get submaps
  BEAM_INFO("Loading global map data from: {}", global_map_data_dir);
  global_map_ = std::make_shared<GlobalMap>(global_map_data_dir);
  BEAM_INFO("Done loading global map data");
  Setup();
}

GlobalMapRefinement::GlobalMapRefinement(const std::string& global_map_data_dir,
                                         const std::string& config_path) {
  // load params & setup
  params_.LoadJson(config_path);

  // load global map to get submaps
  BEAM_INFO("Loading global map data from: {}", global_map_data_dir);
  global_map_ = std::make_shared<GlobalMap>(global_map_data_dir);
  Setup();
}

GlobalMapRefinement::GlobalMapRefinement(std::shared_ptr<GlobalMap>& global_map,
                                         const Params& params)
    : global_map_(global_map), params_(params) {
  Setup();
}

GlobalMapRefinement::GlobalMapRefinement(std::shared_ptr<GlobalMap>& global_map,
                                         const std::string& config_path)
    : global_map_(global_map) {
  params_.LoadJson(config_path);
  Setup();
}

void GlobalMapRefinement::Setup() {
  // setup submap alignment
  const auto& m_conf = params_.submap_alignment.matcher_config;
  auto matcher_type = GetTypeFromConfig(m_conf);
  if (matcher_type == MatcherType::LOAM) {
    std::string ceres_config =
        bs_common::GetAbsoluteConfigPathFromJson(m_conf, "ceres_config");
    matcher_loam_ =
        std::make_unique<LoamMatcher>(LoamParams(m_conf, ceres_config));
  } else if (matcher_type == MatcherType::ICP) {
    matcher_ = std::make_unique<IcpMatcher>(IcpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::GICP) {
    matcher_ = std::make_unique<GicpMatcher>(GicpMatcher::Params(m_conf));
  } else if (matcher_type == MatcherType::NDT) {
    matcher_ = std::make_unique<NdtMatcher>(NdtMatcher::Params(m_conf));
  } else {
    BEAM_ERROR("Invalid matcher type");
    throw std::invalid_argument{"invalid json"};
  }

  loop_closure_candidate_search_ = reloc::RelocCandidateSearchBase::Create(
      params_.loop_closure.candidate_search_config);
  loop_closure_refinement_ = reloc::RelocRefinementBase::Create(
      params_.loop_closure.refinement_config);
}

bool GlobalMapRefinement::RunSubmapRefinement(const std::string& output_path) {
  std::vector<SubmapPtr> submaps = global_map_->GetSubmaps();
  for (uint16_t i = 0; i < submaps.size(); i++) {
    BEAM_INFO("Refining submap No. {}", i);
    if (!RefineSubmap(submaps.at(i), output_path)) {
      BEAM_ERROR("Submap refinement failed, exiting.");
      return false;
    }
  }
  return true;
}

bool GlobalMapRefinement::RunSubmapAlignment(const std::string& output_path) {
  std::vector<SubmapPtr> submaps = global_map_->GetSubmaps();

  if (submaps.size() < 2) {
    BEAM_WARN(
        "Not enough submaps to run submap alignment, at least two are needed");
    return true;
  }

  for (uint16_t i = 1; i < submaps.size(); i++) {
    BEAM_INFO("Aligning submap No. {}", i);
    if (!AlignSubmaps(submaps.at(i - 1), submaps.at(i), output_path)) {
      BEAM_ERROR("Submap alignment failed, exiting.");
      return false;
    }
  }
  return true;
}

bool GlobalMapRefinement::RefineSubmap(SubmapPtr& submap,
                                       const std::string& output_path) {
  if (!output_path.empty() && !std::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path for submap refinement: {}", output_path);
    throw std::runtime_error{"invalid path"};
  }
  std::string dir = "submap_" + std::to_string(submap->Stamp().toSec());
  std::string submap_output =
      output_path.empty() ? output_path : beam::CombinePaths(output_path, dir);
  std::filesystem::create_directory(submap_output);

  // Create optimization graph
  std::shared_ptr<fuse_graphs::HashGraph> graph =
      fuse_graphs::HashGraph::make_shared();
  std::unique_ptr<sr::ScanRegistrationBase> scan_registration =
      sr::ScanRegistrationBase::Create(
          params_.submap_refinement.scan_registration_config,
          params_.submap_refinement.matcher_config, submap_output, true);

  // clear lidar map
  scan_registration->GetMapMutable().Clear();

  // iterate through stored scan poses and add scan registration factors to the
  // graph
  BEAM_INFO("Registering scans");
  for (auto scan_iter = submap->LidarKeyframesBegin();
       scan_iter != submap->LidarKeyframesEnd(); scan_iter++) {
    const bs_models::ScanPose& scan_pose = scan_iter->second;
    auto transaction =
        scan_registration->RegisterNewScan(scan_pose).GetTransaction();
    if (transaction) { graph->update(*transaction); }
  }

  // TODO: Add visual BA constraints

  // Optimize graph and update data
  BEAM_INFO("Optimizing graph");
  graph->optimize();

  BEAM_INFO("updating scan poses");
  for (auto scan_iter = submap->LidarKeyframesBegin();
       scan_iter != submap->LidarKeyframesEnd(); scan_iter++) {
    auto& scan_pose = scan_iter->second;
    Eigen::Matrix4d T_W_B_init = scan_pose.T_REFFRAME_BASELINK();
    scan_pose.UpdatePose(graph);
    Eigen::Matrix4d T_W_B_after = scan_pose.T_REFFRAME_BASELINK();
    RegistrationResult result(T_W_B_init, T_W_B_after);
    summary_.submap_refinement.emplace(scan_pose.Stamp(), result);
  }

  // TODO: update visual data (just frame poses?)

  return true;
}

bool GlobalMapRefinement::AlignSubmaps(const SubmapPtr& submap_ref,
                                       SubmapPtr& submap_tgt,
                                       const std::string& output_path) {
  if (!output_path.empty() && !std::filesystem::exists(output_path)) {
    BEAM_ERROR("Invalid output path for submap alignment: {}", output_path);
    throw std::runtime_error{"invalid path"};
  }
  std::string dir = "submap_" + std::to_string(submap_tgt->Stamp().toSec());
  std::string submap_output =
      output_path.empty() ? output_path : beam::CombinePaths(output_path, dir);
  std::filesystem::create_directory(submap_output);

  Eigen::Matrix4d T_World_SubmapRef = submap_ref->T_WORLD_SUBMAP();
  Eigen::Matrix4d T_World_SubmapRef_Init = submap_ref->T_WORLD_SUBMAP_INIT();
  Eigen::Matrix4d T_World_SubmapTgt_Init = submap_tgt->T_WORLD_SUBMAP_INIT();

  // get initial relative pose
  Eigen::Matrix4d T_SubmapRef_SubmapTgt_Init =
      beam::InvertTransform(T_World_SubmapRef_Init) * T_World_SubmapTgt_Init;

  const auto T_W_B_i = submap_tgt->T_WORLD_SUBMAP();
  const bool use_initials = true;
  if (matcher_loam_) {
    // first get maps in their initial world frame
    LoamPointCloudPtr ref_in_ref_submap_frame =
        std::make_shared<LoamPointCloud>(
            submap_ref->GetLidarLoamPointsInWorldFrame(use_initials));
    LoamPointCloudPtr tgt_in_ref_submap_frame =
        std::make_shared<LoamPointCloud>(
            submap_tgt->GetLidarLoamPointsInWorldFrame(use_initials));

    // then transform to the reference submap frame
    Eigen::Matrix4d T_SubmapRef_WorldInit =
        beam::InvertTransform(T_World_SubmapRef_Init);
    ref_in_ref_submap_frame->TransformPointCloud(T_SubmapRef_WorldInit);
    tgt_in_ref_submap_frame->TransformPointCloud(T_SubmapRef_WorldInit);

    // align
    matcher_loam_->SetRef(ref_in_ref_submap_frame);
    matcher_loam_->SetTarget(tgt_in_ref_submap_frame);
    bool match_success = matcher_loam_->Match();
    Eigen::Matrix4d T_SubmapRef_SubmapTgt =
        matcher_loam_->ApplyResult(T_SubmapRef_SubmapTgt_Init);
    if (!output_path.empty()) {
      matcher_loam_->SaveResults(submap_output, "submap_cloud_");
    }

    // set new submap pose
    Eigen::Matrix4d T_World_SubmapTgt =
        T_World_SubmapRef * T_SubmapRef_SubmapTgt;
    submap_tgt->UpdatePose(T_World_SubmapTgt);
  } else {
    // first get maps in their initial world frame
    PointCloud ref_in_world =
        submap_ref->GetLidarPointsInWorldFrameCombined(use_initials);
    PointCloud tgt_in_world =
        submap_tgt->GetLidarPointsInWorldFrameCombined(use_initials);

    // then transform to the reference submap frame
    Eigen::Matrix4d T_SubmapRef_WorldInit =
        beam::InvertTransform(T_World_SubmapRef_Init);
    PointCloudPtr ref_in_ref_submap_frame = std::make_shared<PointCloud>();
    PointCloudPtr tgt_in_ref_submap_frame = std::make_shared<PointCloud>();
    pcl::transformPointCloud(ref_in_world, *ref_in_ref_submap_frame,
                             T_SubmapRef_WorldInit.cast<float>());
    pcl::transformPointCloud(tgt_in_world, *tgt_in_ref_submap_frame,
                             T_SubmapRef_WorldInit.cast<float>());

    // align
    matcher_->SetRef(ref_in_ref_submap_frame);
    matcher_->SetTarget(tgt_in_ref_submap_frame);
    bool match_success = matcher_->Match();
    Eigen::Matrix4d T_SubmapRef_SubmapTgt =
        matcher_->ApplyResult(T_SubmapRef_SubmapTgt_Init);
    if (!output_path.empty()) {
      matcher_->SaveResults(submap_output, "submap_cloud_");
    }

    // set new submap pose
    Eigen::Matrix4d T_World_SubmapTgt =
        T_World_SubmapRef * T_SubmapRef_SubmapTgt;
    submap_tgt->UpdatePose(T_World_SubmapTgt);
  }

  const auto T_W_B_f = submap_tgt->T_WORLD_SUBMAP();
  RegistrationResult result(T_W_B_i, T_W_B_f);
  summary_.submap_alignment.emplace(submap_tgt->Stamp(), result);

  return true;
}

bool GlobalMapRefinement::RunPoseGraphOptimization(
    const std::string& output_path) {
  std::vector<SubmapPtr> submaps = global_map_->GetSubmaps();
  size_t num_submaps = submaps.size();
  if (num_submaps <= pgo_skip_first_n_submaps_) {
    BEAM_ERROR("Global map size {} not large enough to run PGO, must have at "
               "least {} submaps",
               num_submaps, pgo_skip_first_n_submaps_);
  }

  std::string lc_results_path_refinement =
      beam::CombinePaths(output_path, "refinement");
  std::string lc_results_path_candidate_search =
      beam::CombinePaths(output_path, "candidate_search");
  std::filesystem::create_directory(lc_results_path_refinement);
  std::filesystem::create_directory(lc_results_path_candidate_search);

  BEAM_INFO("Running pose-graph optimization on submaps");
  std::shared_ptr<fuse_graphs::HashGraph> graph =
      fuse_graphs::HashGraph::make_shared();

  // add first pose prior
  {
    const SubmapPtr& first_submap = submaps.at(0);
    bs_constraints::Pose3DStampedTransaction prior_transaction(
        first_submap->Stamp());
    prior_transaction.AddPoseVariables(first_submap->Position(),
                                     first_submap->Orientation(),
                                     first_submap->Stamp());
    prior_transaction.AddPosePrior(
        first_submap->Position(), first_submap->Orientation(),
        pose_prior_noise_, "GlobalMapRefinement::RunPoseGraphOptimization");
    graph->update(*prior_transaction.GetTransaction());
  }

  // add all relative pose transactions
  fuse_core::Transaction::SharedPtr transaction =
      std::make_shared<fuse_core::Transaction>();
  for (int i = 1; i < num_submaps; i++) {
    const SubmapPtr& current_submap = submaps.at(i);
    bs_constraints::Pose3DStampedTransaction new_transaction(
        current_submap->Stamp());
    new_transaction.AddPoseVariables(current_submap->Position(),
                                     current_submap->Orientation(),
                                     current_submap->Stamp());

    // If not first submap add constraint to previous
    if (i < 1) { continue; }
    const SubmapPtr& previous_submap = submaps.at(i - 1);

    Eigen::Matrix4d T_PREVIOUS_CURRENT =
        beam::InvertTransform(previous_submap->T_WORLD_SUBMAP()) *
        current_submap->T_WORLD_SUBMAP();
    new_transaction.AddPoseConstraint(
        previous_submap->Position(), current_submap->Position(),
        previous_submap->Orientation(), current_submap->Orientation(),
        bs_common::TransformMatrixToVectorWithQuaternion(T_PREVIOUS_CURRENT),
        params_.loop_closure.local_mapper_covariance,
        "GlobalMap::InitiateNewSubmapPose");
    graph->update(*new_transaction.GetTransaction());
  }

  // now iterate through all submaps, check if loop closures can be run, and if
  // so, update graph after each loop closure
  for (int query_index = pgo_skip_first_n_submaps_;
       query_index < num_submaps - 1; query_index++) {
    std::vector<int> matched_indices;
    std::vector<Eigen::Matrix4d, beam::AlignMat4d> Ts_MATCH_QUERY;
    // ignore the current empty submap, and the last full submap (the query)
    static bool use_initial_poses = false;

    // ignore all submaps equal to or after the submap before query.
    // I.e. if query is 3, ignore 2, 3, 4, ...
    int ignore_last_n_submaps = submaps.size() - query_index + 1;
    loop_closure_candidate_search_->FindRelocCandidates(
        submaps, submaps.at(query_index), matched_indices, Ts_MATCH_QUERY,
        ignore_last_n_submaps, use_initial_poses,
        lc_results_path_candidate_search);
    std::string candidates;
    for (const auto& id : matched_indices) {
      candidates += std::to_string(id) + " ";
    }
    if (matched_indices.size() == 0) { continue; }

    BEAM_INFO(
        "Found {} loop closure candidates for query index {}. Candidates: {}",
        matched_indices.size(), query_index, candidates);

    auto transaction = std::make_shared<fuse_core::Transaction>();
    for (int i = 0; i < matched_indices.size(); i++) {
      if (matched_indices[i] >= query_index - 1) {
        BEAM_ERROR("Error in candidate search implementation, please fix!");
        continue;
      }

      const auto& matched_submap = submaps.at(matched_indices[i]);
      const auto& query_submap = submaps.at(query_index);
      RelocRefinementResults results = loop_closure_refinement_->RunRefinement(
          matched_submap, query_submap, Ts_MATCH_QUERY[i],
          lc_results_path_refinement);

      if (!results.successful) { continue; }

      bs_constraints::Pose3DStampedTransaction new_transaction(
          query_submap->Stamp());
      new_transaction.AddPoseConstraint(
          matched_submap->Position(), query_submap->Position(),
          matched_submap->Orientation(), query_submap->Orientation(),
          bs_common::TransformMatrixToVectorWithQuaternion(
              results.T_MATCH_QUERY),
          params_.loop_closure.loop_closure_covariance,
          "GlobalMap::RunLoopClosure");

      transaction->merge(*(new_transaction.GetTransaction()));
    }
    graph->update(*transaction);
    graph->optimize();
    global_map_->UpdateSubmapPoses(graph, ros::Time::now());
  }
  return true;
}

void GlobalMapRefinement::SaveResults(const std::string& output_path,
                                      bool save_initial) {
  // create results directory
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("Output directory does not exist, not outputting global map "
               "refinement "
               "results. Input: {}",
               output_path);
    return;
  }

  // save
  summary_.Save(output_path);
  global_map_->SaveTrajectoryFile(output_path, save_initial);
  global_map_->SaveTrajectoryClouds(output_path, save_initial);
  global_map_->SaveSubmapFrames(output_path, save_initial);
  global_map_->SaveLidarSubmaps(output_path, save_initial);
  global_map_->SaveKeypointSubmaps(output_path, save_initial);
}

void GlobalMapRefinement::SaveGlobalMapData(const std::string& output_path) {
  // create results directory
  if (!boost::filesystem::exists(output_path)) {
    BEAM_ERROR("Output directory does not exist, not outputting global map "
               "refinement "
               "results. Input: {}",
               output_path);
    return;
  }

  std::string save_dir =
      beam::CombinePaths(output_path, "global_map_data_refined");
  boost::filesystem::create_directory(save_dir);

  // save
  global_map_->SaveData(save_dir);
}

} // namespace bs_models::global_mapping