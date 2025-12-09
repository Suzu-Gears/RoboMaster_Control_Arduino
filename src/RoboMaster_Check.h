#pragma once

#include <cstddef>
#include <cstdint>

#include "RoboMaster_Types.h"

namespace robomaster {
namespace Check {

template<typename... AliasSpecs>
constexpr bool has_rx_conflict() {
  constexpr size_t N = sizeof...(AliasSpecs);
  if constexpr (N < 2) return false;
  constexpr uint32_t rxs[] = { AliasSpecs::spec::rx_id... };
  for (size_t i = 0; i < N; ++i) {
    for (size_t j = i + 1; j < N; ++j) {
      if (rxs[i] == rxs[j]) return true;
    }
  }
  return false;
}

template<typename... AliasSpecs>
constexpr bool has_tx_conflict() {
  constexpr size_t N = sizeof...(AliasSpecs);
  if constexpr (N < 2) return false;
  constexpr uint32_t tx_ids[] = { AliasSpecs::spec::tx_id... };
  constexpr uint8_t tx_idxs[] = { AliasSpecs::spec::tx_buf_idx... };
  for (size_t i = 0; i < N; ++i) {
    for (size_t j = i + 1; j < N; ++j) {
      if (tx_ids[i] == tx_ids[j] && tx_idxs[i] == tx_idxs[j]) return true;
    }
  }
  return false;
}

}  // namespace Check
}  // namespace robomaster
