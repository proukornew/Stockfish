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

#ifndef TT_H_INCLUDED
#define TT_H_INCLUDED

#include "misc.h"
#include "types.h"

namespace Stockfish {

/// TTEntry struct is the 10 bytes transposition table entry, defined as below:
///
/// key        16 bit
/// depth       8 bit
/// generation  5 bit
/// pv node     1 bit
/// bound type  2 bit
/// move       16 bit
/// value      16 bit
/// eval value 16 bit

struct TTEntry {

  Move  move()  const { return (Move )fields.move16; }
  Value value() const { return (Value)fields.value16; }
  Value eval()  const { return (Value)fields.eval16; }
  Depth depth() const { return (Depth)fields.depth8 + DEPTH_OFFSET; }
  bool is_pv()  const { return (bool)fields.pv; }
  Bound bound() const { return (Bound)fields.bound; }
  void save(Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev);

private:
  friend class TranspositionTable;

  struct __attribute__ ((__packed__, gcc_struct)) { //gcc_struct for mingw
	uint16_t move16;
	int16_t  value16;
	int16_t  eval16;
	uint8_t  depth8;
	uint8_t  pv:      1;
	uint8_t  bound:   2;
	uint8_t  gen:     5;
	uint32_t key24:  24;
  } fields;
};


/// A TranspositionTable is an array of Cluster, of size clusterCount. Each
/// cluster consists of ClusterSize number of TTEntry. Each non-empty TTEntry
/// contains information on exactly one position. The size of a Cluster should
/// divide the size of a cache line for best performance, as the cacheline is
/// prefetched when possible.

class TranspositionTable {

  static constexpr int ClusterSize = 64;

  struct Cluster {
    TTEntry entry[ClusterSize]; // 11x64=704
  };

  static_assert(sizeof(Cluster) == 704, "Unexpected Cluster size");

  // Constants used to refresh the hash table periodically
  static constexpr unsigned GENERATION_BITS  = 3;                                // nb of bits reserved for other things
  static constexpr int      GENERATION_DELTA = (1 << GENERATION_BITS);           // increment for generation field
  static constexpr int      GENERATION_CYCLE = 255 + (1 << GENERATION_BITS);     // cycle length
  static constexpr int      GENERATION_MASK  = (0xFF << GENERATION_BITS) & 0xFF; // mask to pull out generation number

public:
 ~TranspositionTable() { aligned_large_pages_free(table); }
  void new_search() { generation8 += 1; }
  TTEntry* probe(const Key key, bool& found) const;
  int hashfull() const;
  void resize(size_t mbSize);
  void clear();

  TTEntry* first_entry(const Key key) const {
    return &table[mul_hi64(key, clusterCount)].entry[0];
  }

private:
  friend struct TTEntry;

  size_t clusterCount;
  Cluster* table;
  uint8_t generation8; // Size must be not bigger than TTEntry::genBound8
};

extern TranspositionTable TT;

} // namespace Stockfish

#endif // #ifndef TT_H_INCLUDED
