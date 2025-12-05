#include "broad_phase.hpp"

#include <ccd/linear/broad_phase.hpp>
#include <ccd/rigid/rigid_body_bvh.hpp>
#include <ccd/rigid/rigid_body_hash_grid.hpp>
#include <logger.hpp>
#include <profiler.hpp>
#include <utils/type_name.hpp>

namespace ipc::rigid {

///////////////////////////////////////////////////////////////////////////////
// Broad-Phase Discrete Collision Detection
// NOTE: Yes, this is inside the CCD directory.
///////////////////////////////////////////////////////////////////////////////

void detect_collision_candidates_rigid(
    const RigidBodyAssembler& bodies,
    const PosesD& poses,
    const int collision_types,
    Candidates& candidates,
    DetectionMethod method,
    const double inflation_radius)
{
    if (bodies.m_rbs.size() <= 1) {
        return;
    }

    PROFILE_POINT("detect_discrete_collision_candidates_rigid");
    PROFILE_START();

    switch (method) {
    case BRUTE_FORCE:
        detect_collision_candidates_brute_force(
            bodies.world_vertices(poses), bodies.m_edges, bodies.m_faces,
            bodies.group_ids(), collision_types, candidates);
        break;
    case HASH_GRID:
        detect_collision_candidates_rigid_hash_grid(
            bodies, poses, collision_types, candidates, inflation_radius);
        break;
    case BVH:
        detect_collision_candidates_rigid_bvh(
            bodies, poses, collision_types, candidates, inflation_radius);
        break;
    }

    PROFILE_END();
}

// Find all edge-vertex collisions in one time step using spatial-hashing to
// only compare points and edge in the same cells.
void detect_collision_candidates_rigid_hash_grid(
    const RigidBodyAssembler& bodies,
    const PosesD& poses,
    const int collision_types,
    Candidates& candidates,
    const double inflation_radius)
{
    std::vector<std::pair<int, int>> body_pairs =
        bodies.close_bodies(poses, poses, inflation_radius);

    if (body_pairs.size() == 0) {
        return;
    }

    RigidBodyHashGrid hashgrid;
    hashgrid.resize(bodies, poses, body_pairs, inflation_radius);
    hashgrid.addBodies(bodies, poses, body_pairs, inflation_radius);

    const Eigen::VectorXi& group_ids = bodies.group_ids();
    auto can_vertices_collide = [&group_ids](size_t vi, size_t vj) {
        return group_ids[vi] != group_ids[vj];
    };

    if (collision_types & CollisionType::EDGE_VERTEX) {
        hashgrid.getVertexEdgePairs(
            bodies.m_edges, candidates.ev_candidates, can_vertices_collide);
    }
    if (collision_types & CollisionType::EDGE_EDGE) {
        hashgrid.getEdgeEdgePairs(
            bodies.m_edges, candidates.ee_candidates, can_vertices_collide);
    }
    if (collision_types & CollisionType::FACE_VERTEX) {
        hashgrid.getFaceVertexPairs(
            bodies.m_faces, candidates.fv_candidates, can_vertices_collide);
    }
}

// Use a BVH to create a set of all candidate collisions.
void detect_collision_candidates_rigid_bvh(
    const RigidBodyAssembler& bodies,
    const PosesD& poses,
    const int collision_types,
    Candidates& candidates,
    const double inflation_radius)
{
    std::vector<std::pair<int, int>> body_pairs =
        bodies.close_bodies(poses, poses, inflation_radius);

    // Use interval arithmetic to conservativly capture all distance candidates
    auto posesI = cast<Interval>(poses);

    Candidates local_candidates;
    for (size_t i = 0; i < body_pairs.size(); ++i) {
        detect_body_pair_collision_candidates_bvh(
            bodies, posesI, body_pairs[i].first, body_pairs[i].second,
            collision_types, local_candidates,
            inflation_radius);
    }

    candidates = local_candidates;
}

///////////////////////////////////////////////////////////////////////////////
// Broad-Phase Continous Collision Detection
///////////////////////////////////////////////////////////////////////////////

void detect_collision_candidates_rigid(
    const RigidBodyAssembler& bodies,
    const PosesD& poses_t0,
    const PosesD& poses_t1,
    const int collision_types,
    Candidates& candidates,
    DetectionMethod method,
    const double inflation_radius)
{
    if (bodies.m_rbs.size() <= 1) {
        return;
    }

    PROFILE_POINT("detect_continuous_collision_candidates_rigid");
    PROFILE_START();

    switch (method) {
    case BRUTE_FORCE:
        detect_collision_candidates_brute_force(
            bodies.world_vertices(poses_t0), bodies.m_edges, bodies.m_faces,
            bodies.group_ids(), collision_types, candidates);
        break;
    case HASH_GRID:
        detect_collision_candidates_rigid_hash_grid(
            bodies, poses_t0, poses_t1, collision_types, candidates,
            inflation_radius);
        break;
    case BVH:
        detect_collision_candidates_rigid_bvh(
            bodies, poses_t0, poses_t1, collision_types, candidates,
            inflation_radius);
        break;
    }

    PROFILE_END();
}

// Find all edge-vertex collisions in one time step using spatial-hashing to
// only compare points and edge in the same cells.
void detect_collision_candidates_rigid_hash_grid(
    const RigidBodyAssembler& bodies,
    const PosesD& poses_t0,
    const PosesD& poses_t1,
    const int collision_types,
    Candidates& candidates,
    const double inflation_radius)
{
    std::vector<std::pair<int, int>> body_pairs =
        bodies.close_bodies(poses_t0, poses_t1, inflation_radius);

    if (body_pairs.size() == 0) {
        return;
    }

    RigidBodyHashGrid hashgrid;
    hashgrid.resize(bodies, poses_t0, poses_t1, body_pairs, inflation_radius);
    hashgrid.addBodies(
        bodies, poses_t0, poses_t1, body_pairs, inflation_radius);

    const Eigen::VectorXi& group_ids = bodies.group_ids();
    auto can_vertices_collide = [&group_ids](size_t vi, size_t vj) {
        return group_ids[vi] != group_ids[vj];
    };

    if (collision_types & CollisionType::EDGE_VERTEX) {
        hashgrid.getVertexEdgePairs(
            bodies.m_edges, candidates.ev_candidates, can_vertices_collide);
    }
    if (collision_types & CollisionType::EDGE_EDGE) {
        hashgrid.getEdgeEdgePairs(
            bodies.m_edges, candidates.ee_candidates, can_vertices_collide);
    }
    if (collision_types & CollisionType::FACE_VERTEX) {
        hashgrid.getFaceVertexPairs(
            bodies.m_faces, candidates.fv_candidates, can_vertices_collide);
    }
}

// Use a BVH to create a set of all candidate collisions.
void detect_collision_candidates_rigid_bvh(
    const RigidBodyAssembler& bodies,
    const PosesD& poses_t0,
    const PosesD& poses_t1,
    const int collision_types,
    Candidates& candidates,
    const double inflation_radius)
{
    std::vector<std::pair<int, int>> body_pairs =
        bodies.close_bodies(poses_t0, poses_t1, inflation_radius);

    Poses<Interval> poses = interpolate(
        cast<Interval>(poses_t0), cast<Interval>(poses_t1), Interval(0, 1));

    Candidates local_candidates;
    for (size_t i = 0; i < body_pairs.size(); ++i) {
        detect_body_pair_collision_candidates_bvh(
            bodies, poses, body_pairs[i].first, body_pairs[i].second,
            collision_types, local_candidates,
            inflation_radius);
    }

    candidates = local_candidates;
}

///////////////////////////////////////////////////////////////////////////////
// Broad-Phase Intersection Detection
///////////////////////////////////////////////////////////////////////////////

// Use a BVH to create a set of all candidate intersections.
void detect_intersection_candidates_rigid_bvh(
    const RigidBodyAssembler& bodies,
    const PosesD& poses,
    std::vector<EdgeFaceCandidate>& ef_candidates)
{
    std::vector<std::pair<int, int>> body_pairs =
        bodies.close_bodies(poses, poses, /*inflation_radius=*/0);

    auto posesI = cast<Interval>(poses);

    std::vector<EdgeFaceCandidate> local_candidates;
    for (size_t i = 0; i < body_pairs.size(); ++i) {
        int bodyA_id = body_pairs[i].first;
        int bodyB_id = body_pairs[i].second;
        sort_body_pair(bodies, bodyA_id, bodyB_id);

        const auto RA = poses[bodyA_id].construct_rotation_matrix();
        const auto RB = poses[bodyB_id].construct_rotation_matrix();
        const auto& pA = poses[bodyA_id].position;
        const auto& pB = poses[bodyB_id].position;
        const MatrixXI VA =
            ((bodies[bodyA_id].vertices * RA.transpose()).rowwise()
             + (pA - pB).transpose())
            * RB;

        std::vector<AABB> aabbs = vertex_aabbs(VA);

        detect_body_pair_intersection_candidates_from_aabbs(
            bodies, aabbs, bodyA_id, bodyB_id,
            local_candidates);
    }

    ef_candidates = local_candidates;
}

///////////////////////////////////////////////////////////////////////////////
// Helper functions
///////////////////////////////////////////////////////////////////////////////

} // namespace ipc::rigid
