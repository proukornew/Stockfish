/*
  Stockfish, a UCI chess playing engine derived from Glaurung 2.1
  Copyright (C) 2004-2021 The Stockfish developers (see AUTHORS file)

  Stockfish is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Stockfish is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <cstring>   // For std::memset
#include <iostream>
#include <thread>

#include "bitboard.h"
#include "misc.h"
#include "thread.h"
#include "tt.h"
#include "uci.h"

namespace Stockfish {

TranspositionTable TT; // Our global transposition table

/// TTEntry::save() populates the TTEntry with a new node's data, possibly
/// overwriting an old position. Update is not atomic and can be racy.

void TTEntry::save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev) {

  // Preserve any existing move for the same position
  if (m || (uint16_t)k != key16)
      move16 = (uint16_t)m;

  // Overwrite less valuable entries (cheapest checks first)
  if (b == BOUND_EXACT
      || (uint16_t)k != key16
      || d - DEPTH_OFFSET > depth8 - 4)
  {
      assert(d > DEPTH_OFFSET);
      assert(d < 256 + DEPTH_OFFSET);

      key16     = (uint16_t)k;
      depth8    = (uint8_t)(d - DEPTH_OFFSET);
      genBound8 = (uint8_t)(TT.generation8 | uint8_t(pv) << 2 | b);
      value16   = (int16_t)v;
      eval16    = (int16_t)ev;
  }
}


/// TranspositionTable::resize() sets the size of the transposition table,
/// measured in megabytes. Transposition table consists of a power of 2 number
/// of clusters and each cluster consists of ClusterSize number of TTEntry.

void TranspositionTable::resize(size_t mbSize) {

  Threads.main()->wait_for_search_finished();

  aligned_large_pages_free(table);

  clusterCount = mbSize * 1024 * 1024 / sizeof(Cluster);

  table = static_cast<Cluster*>(aligned_large_pages_alloc(clusterCount * sizeof(Cluster)));
  if (!table)
  {
      std::cerr << "Failed to allocate " << mbSize
                << "MB for transposition table." << std::endl;
      exit(EXIT_FAILURE);
  }

  clear();
}


/// TranspositionTable::clear() initializes the entire transposition table to zero,
//  in a multi-threaded way.

void TranspositionTable::clear() {

  std::vector<std::thread> threads;

  for (size_t idx = 0; idx < Options["Threads"]; ++idx)
  {
      threads.emplace_back([this, idx]() {

          // Thread binding gives faster search on systems with a first-touch policy
          if (Options["Threads"] > 8)
              WinProcGroup::bindThisThread(idx);

          // Each thread will zero its part of the hash table
          const size_t stride = size_t(clusterCount / Options["Threads"]),
                       start  = size_t(stride * idx),
                       len    = idx != Options["Threads"] - 1 ?
                                stride : clusterCount - start;

          std::memset(&table[start], 0, len * sizeof(Cluster));
      });
  }

  for (std::thread& th : threads)
      th.join();
}


/// TranspositionTable::probe() looks up the current position in the transposition
/// table. It returns true and a pointer to the TTEntry if the position is found.
/// Otherwise, it returns false and a pointer to an empty or least valuable TTEntry
/// to be replaced later. The replace value of an entry is calculated as its depth
/// minus 8 times its relative age. TTEntry t1 is considered more valuable than
/// TTEntry t2 if its replace value is greater than that of t2.

__attribute__ ((hot)) TTEntry* TranspositionTable::probe(const Key key, bool& found) const {
	Cluster* const cluster = first_entry(key);
	const uint16_t key16 = (uint16_t)key;
	//int nline = ((uint32_t)key >> 16) & 0x3;

	auto match = _mm_set1_epi16(key16);
	auto metadata = _mm_load_si128((__m128i const*)cluster);
	auto mask = _mm_movemask_epi8(_mm_cmpeq_epi16(match, metadata));
	int index;
	if (mask) {
		found = true;
		index = __builtin_ctz(mask) >> 1;
	}
	else {
		found = false;
		auto depth8genBound8 = _mm_load_si128((__m128i const*)&cluster->depth8genBound8[0]);
		auto andmask = _mm_set1_epi16((uint16_t)0xFF00);
		auto cmpmask = _mm_set1_epi16(0x0000);
		auto depth8 = _mm_and_si128(depth8genBound8, andmask);
		auto mask2 = _mm_movemask_epi8(_mm_cmpeq_epi16(depth8, cmpmask));
		if (mask2) {
			index = __builtin_ctz(mask2) >> 1;
		}
		else {
			auto andmask2 = _mm_set1_epi16(0x00F8);
			auto curGen = _mm_set1_epi16((uint16_t)generation8 << 8);
			auto gen8 = _mm_slli_epi16(_mm_and_si128(depth8genBound8, andmask2), 8);
			auto diffgen = _mm_subs_epu16(curGen, gen8);
			auto res = _mm_subs_epu16(depth8, diffgen);
			index = _mm_extract_epi32(_mm_minpos_epu16(res), 0) >> 16;
		}
	}

	return (TTEntry*)&(cluster->key16[index]);
}


/// TranspositionTable::hashfull() returns an approximation of the hashtable
/// occupation during a search. The hash is x permill full, as per UCI protocol.

int TranspositionTable::hashfull() const {

  int cnt = 0;
  for (int i = 0; i < 125; ++i) {
      for (int j = 0; j < ClusterSize; ++j) {
		  TTEntry* tte = (TTEntry*)&(table[i].key16[j]);
          cnt += tte->depth8 && (tte->genBound8 & GENERATION_MASK) == generation8;
	  }
  }
  return cnt;
}

} // namespace Stockfish
