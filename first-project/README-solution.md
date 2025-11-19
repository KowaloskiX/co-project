# High-Level Explanation of the Code

This code solves a **complex pathfinding problem** (likely a competitive programming challenge) with a twist: the cost of edges changes based on their **position in the path**.

## Problem Overview

- **Graph traversal** from `startNode` to `endNode`
- **Dynamic edge costs**: Each edge has two weights (`w1`, `w2`)
  - On **prime-numbered steps** (1st, 2nd, 3rd, 5th, 7th...), the cost is `3 * w2`
  - On **non-prime steps**, the cost is `w1`
- **Goal**: Find the path with minimum total cost (called "Blackie length")

## Solution Strategy (3 Phases)

### **Phase 1: Quick Seeding (First ~5 seconds)**

- Uses **multiple Dijkstra variants** with different heuristics to find a "good enough" initial solution quickly
- Tries different α-weighted combinations of `w1` and `w2` to explore the solution space
- Establishes `global_best_cost` as an initial upper bound

### **Phase 2: Heuristic Calculation**

- Computes **reverse Dijkstra** from target node
- Creates `h_cost_to_target[]` - an admissible heuristic (minimum possible remaining cost from any node to target)
- This powers the A\* search in Phase 3

### **Phase 3: Iterative Weighted A\* (Remaining ~14 seconds)**

- **Main optimization engine** that iteratively improves the solution
- Uses **weighted A\*** with decreasing weights (1.30 → 1.00)
  - Higher weights = faster, less optimal
  - Lower weights = slower, more optimal
- Employs aggressive pruning:
  - **Time limits** (19s total, 1s safety buffer)
  - **Bound pruning** (discard paths worse than current best)
  - **Dominance pruning** (O(1) hash map checks if we've reached same state cheaper)
  - **Bounded cycle detection** (prevents revisiting recent nodes)

## Key Optimizations

1. **State encoding**: Packs `(node, steps)` into single `long long` for fast lookups
2. **Path reconstruction**: Stores compressed parent pointers to avoid copying vectors
3. **Tie-breaking**: Prefers deeper states (more progress) when costs are equal
4. **Sieve of Eratosthenes**: Pre-computes all primes up to max path length
5. **Iterative deepening**: Starts aggressive, becomes more precise as time allows

## Output

Prints the best path found within the time limit (size + node sequence).

---

This is essentially a **highly optimized A\* variant** that handles position-dependent costs through careful state management and aggressive pruning strategies.
