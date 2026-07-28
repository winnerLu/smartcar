/**
 * Copyright 2025 Suchetan Saravanan.
 *
 * Licensed under the Apache License, Version 2.0.
 */

#ifndef ROADMAP_EXPLORER__TRANSIENT_FRONTIER_BLACKLIST_HPP_
#define ROADMAP_EXPLORER__TRANSIENT_FRONTIER_BLACKLIST_HPP_

#include <cstdint>
#include <vector>

#include <nav2_costmap_2d/costmap_2d.hpp>

#include "roadmap_explorer/Frontier.hpp"

namespace roadmap_explorer
{

/**
 * @brief Return a compact revision for the complete costmap state.
 *
 * SLAM occupancy grids do not expose a monotonically increasing version to
 * Costmap2D consumers. Hashing the metadata and cell bytes gives frontier
 * dispatch code an equivalent revision: a resize, origin shift, or occupancy
 * update changes the value and makes a previously temporary rejection eligible
 * for validation again.
 *
 * Callers must hold the Costmap2D mutex while this function is running.
 */
inline uint64_t costmapRevision(const nav2_costmap_2d::Costmap2D & costmap)
{
  constexpr uint64_t fnv_offset = 1469598103934665603ULL;
  uint64_t revision = fnv_offset;

  const auto append_bytes =
    [&revision](const void * value, size_t size)
    {
      constexpr uint64_t local_fnv_prime = 1099511628211ULL;
      const auto * bytes = static_cast<const unsigned char *>(value);
      for (size_t index = 0; index < size; ++index) {
        revision ^= static_cast<uint64_t>(bytes[index]);
        revision *= local_fnv_prime;
      }
    };

  const auto size_x = costmap.getSizeInCellsX();
  const auto size_y = costmap.getSizeInCellsY();
  const auto resolution = costmap.getResolution();
  const auto origin_x = costmap.getOriginX();
  const auto origin_y = costmap.getOriginY();
  append_bytes(&size_x, sizeof(size_x));
  append_bytes(&size_y, sizeof(size_y));
  append_bytes(&resolution, sizeof(resolution));
  append_bytes(&origin_x, sizeof(origin_x));
  append_bytes(&origin_y, sizeof(origin_y));

  const auto cell_count = static_cast<size_t>(size_x) * size_y;
  const auto * cells = costmap.getCharMap();
  if (cells != nullptr && cell_count > 0) {
    append_bytes(cells, cell_count);
  }
  return revision;
}

/**
 * @brief Frontiers rejected only because of the current map snapshot.
 *
 * Unlike a real Nav2 execution failure, a missing known-free projection may
 * become valid as soon as SLAM expands or updates the map. These entries are
 * therefore prohibited only while the costmap revision remains unchanged.
 */
class TransientFrontierBlacklist
{
public:
  bool releaseIfMapChanged(uint64_t revision)
  {
    if (!has_revision_) {
      revision_ = revision;
      has_revision_ = true;
      return false;
    }
    if (revision_ == revision) {
      return false;
    }
    revision_ = revision;
    const bool released = !frontiers_.empty();
    frontiers_.clear();
    return released;
  }

  void add(const FrontierPtr & frontier, uint64_t revision)
  {
    releaseIfMapChanged(revision);
    frontiers_.push_back(frontier);
  }

  const std::vector<FrontierPtr> & frontiers() const
  {
    return frontiers_;
  }

  uint64_t revision() const
  {
    return revision_;
  }

private:
  bool has_revision_{false};
  uint64_t revision_{0};
  std::vector<FrontierPtr> frontiers_;
};

}  // namespace roadmap_explorer

#endif  // ROADMAP_EXPLORER__TRANSIENT_FRONTIER_BLACKLIST_HPP_
