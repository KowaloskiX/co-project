#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <tuple>
#include <limits>
#include <unordered_map>

using namespace std;

// --- Structs ---

struct Edge
{
  int to;
  int w1;
  int w2;
};

// State for standard Dijkstra (alpha-weighted)
struct StateAlpha
{
  int node;
  double dist;
  bool operator>(const StateAlpha &o) const { return dist > o.dist; }
};

// State for heuristic Dijkstra (static cost)
struct StateStatic
{
  int node;
  long long dist;
  bool operator>(const StateStatic &o) const { return dist > o.dist; }
};

// State for the main Iterative Bounded A* search (Phase 3)
struct StateAStar
{
  long long f_cost; // Estimated total cost (g + w*h) - priority in PQ
  long long g_cost; // True cost so far (g)
  int node;         // Current node ID
  int steps;        // Number of edges taken (1-based index of next edge)
  int parent_idx;   // Index into the path_storage vector for reconstruction

  // TIE-BREAKER: Prioritize states with lower f_cost.
  // On ties, prioritize the state that has taken MORE steps (depth-first exploration).
  bool operator>(const StateAStar &o) const
  {
    if (f_cost != o.f_cost)
      return f_cost > o.f_cost;
    return steps < o.steps; // Prioritize greater depth
  }
};

// --- Globals ---
int N, M, startNode, endNode;
vector<vector<Edge>> graph;
vector<bool> is_prime;

// Heuristics
vector<long long> h_cost_to_target;

// Best solution found so far
vector<int> global_best_path;
long long global_best_cost = numeric_limits<long long>::max();

// Path reconstruction storage (used by iterative A*)
vector<tuple<int, int>> path_storage; // Stores (node, parent_idx) for reconstruction

// --- CRITICAL OPTIMIZATION (O(1) State Pruning) ---
// Key: ((long long)node << 32) | steps. Value: min_cost_to_reach
unordered_map<long long, long long> min_cost_for_state;

const long long INF = numeric_limits<long long>::max() / 3;
auto t_start = chrono::steady_clock::now();
const double TIME_LIMIT_TOTAL = 174.0;  // 1.0s safety buffer
const double TIME_LIMIT_SEEDING = 140.0; // 5.0s for a strong initial bound
long long loop_counter = 0;
bool time_limit_exceeded = false;

// --- Utilities ---

/**
 * @brief Sieves primes up to n.
 */
void sieve_primes(int n)
{
  is_prime.assign(n + 1, true);
  is_prime[0] = is_prime[1] = false;
  for (int i = 2; (long long)i * i <= n; ++i)
    if (is_prime[i])
      for (int j = i * i; j <= n; j += i)
        is_prime[j] = false;
}

/**
 * @brief Returns the minimum *possible* cost of an edge, ignoring position.
 */
inline long long static_cost(const Edge &e)
{
  return min((long long)e.w1, 3LL * e.w2);
}

/**
 * @brief Calculates the true, position-dependent cost of an edge.
 */
inline long long step_cost(const Edge &e, int step_index)
{
  if (step_index < (int)is_prime.size() && is_prime[step_index])
  {
    return 3LL * e.w2;
  }
  return (long long)e.w1;
}

/**
 * @brief Calculates the full Blackie cost of a given path.
 */
long long calculate_blackie_length(const vector<int> &path)
{
  long long res = 0;
  for (size_t i = 0; i + 1 < path.size(); ++i)
  {
    int u = path[i], v = path[i + 1];
    bool found = false;
    for (const auto &e : graph[u])
    {
      if (e.to == v)
      {
        res += step_cost(e, (int)i + 1);
        found = true;
        break;
      }
    }
    if (!found)
      return INF;
  }
  return res;
}

/**
 * @brief Reconstructs the path from the storage given the final state's index.
 */
vector<int> reconstruct_path(int final_parent_idx)
{
  vector<int> path;
  int current_idx = final_parent_idx;
  while (current_idx != -1)
  {
    path.push_back(get<0>(path_storage[current_idx]));
    current_idx = get<1>(path_storage[current_idx]);
  }
  reverse(path.begin(), path.end());
  return path;
}

/**
 * @brief Encodes the (node, steps) state into a single long long key.
 */
inline long long encode_state(int node, int steps)
{
  return ((long long)node << 32) | (steps);
}

/**
 * @brief Bounded Cycle Check: Ensures a node v is not present in the last K steps.
 * This is O(K) complexity, making it fast and preventing short cycles (WA fix).
 */
bool BoundedCheckCycle(int v, int parent_idx, int max_depth = 20)
{
  int current_idx = parent_idx;
  int depth = 0;
  while (current_idx != -1 && depth < max_depth)
  {
    if (get<0>(path_storage[current_idx]) == v)
    {
      return true;
    }
    current_idx = get<1>(path_storage[current_idx]);
    depth++;
  }
  return false;
}

// --- Phase 1: Heuristics & Seeding ---

vector<int> dijkstra_alpha(int src, int dst, double alpha)
{
  vector<double> dist(N, 1e18);
  vector<int> parent(N, -1);
  priority_queue<StateAlpha, vector<StateAlpha>, greater<StateAlpha>> pq;

  dist[src] = 0.0;
  pq.push(StateAlpha{src, 0.0});

  while (!pq.empty())
  {
    auto [u, d] = pq.top();
    pq.pop();

    if (d > dist[u] + 1e-9)
      continue;
    if (u == dst)
      break;

    for (auto &e : graph[u])
    {
      double cost = e.w1 + alpha * e.w2;
      if (dist[e.to] > d + cost)
      {
        dist[e.to] = d + cost;
        parent[e.to] = u;
        pq.push(StateAlpha{e.to, dist[e.to]});
      }
    }
  }

  if (dist[dst] >= 1e17)
    return {};
  vector<int> path;
  for (int v = dst; v != -1; v = parent[v])
    path.push_back(v);
  reverse(path.begin(), path.end());
  return path;
}

vector<int> dijkstra_static_path(int src, int dst)
{
  vector<long long> dist(N, INF);
  vector<int> parent(N, -1);
  priority_queue<StateStatic, vector<StateStatic>, greater<StateStatic>> pq;

  dist[src] = 0;
  pq.push(StateStatic{src, 0});

  while (!pq.empty())
  {
    auto [u, d] = pq.top();
    pq.pop();

    if (d > dist[u])
      continue;
    if (u == dst)
      break;

    for (auto &e : graph[u])
    {
      long long cost = static_cost(e);
      if (dist[e.to] > d + cost)
      {
        dist[e.to] = d + cost;
        parent[e.to] = u;
        pq.push(StateStatic{e.to, dist[e.to]});
      }
    }
  }

  if (dist[dst] == INF)
    return {};
  vector<int> path;
  for (int v = dst; v != -1; v = parent[v])
    path.push_back(v);
  reverse(path.begin(), path.end());
  return path;
}

void calculate_reverse_dijkstra_heuristic(int target)
{
  h_cost_to_target.assign(N, INF);
  priority_queue<StateStatic, vector<StateStatic>, greater<StateStatic>> pq;

  h_cost_to_target[target] = 0;
  pq.push(StateStatic{target, 0});

  while (!pq.empty())
  {
    auto [u, d] = pq.top();
    pq.pop();

    if (d > h_cost_to_target[u])
      continue;

    for (auto &e : graph[u])
    {
      int v = e.to;
      long long cost = static_cost(e);
      if (h_cost_to_target[v] > d + cost)
      {
        h_cost_to_target[v] = d + cost;
        pq.push(StateStatic{v, h_cost_to_target[v]});
      }
    }
  }
}

// --- Phase 3: Iterative Bounded A* Search (Optimized) ---

void find_path_iterative(double heuristic_weight)
{
  if (global_best_cost == INF || time_limit_exceeded)
    return;

  priority_queue<StateAStar, vector<StateAStar>, greater<StateAStar>> pq;

  // Reset path storage and dominance map for this run
  path_storage.clear();
  path_storage.reserve(N * 2);
  min_cost_for_state.clear();
  loop_counter = 0;

  // Initial state setup
  path_storage.emplace_back(startNode, -1); // Index 0

  long long h0 = h_cost_to_target[startNode];
  if (h0 == INF)
    return;

  pq.push(StateAStar{h0, 0LL, startNode, 0, 0}); // f=h0, g=0, steps=0, parent_idx=0
  min_cost_for_state[encode_state(startNode, 0)] = 0LL;

  while (!pq.empty())
  {
    auto [f, g, u, steps, parent_idx] = pq.top();
    pq.pop();

    // 1. Time Pruning (Efficient Check)
    if (++loop_counter % 2048 == 0)
    {
      if (chrono::duration<double>(chrono::steady_clock::now() - t_start).count() > TIME_LIMIT_TOTAL)
      {
        time_limit_exceeded = true;
        return;
      }
    }

    // 2. Cost Pruning (Bound) - Check against global best path
    if (f >= global_best_cost)
      continue;

    // 3. Dominance Pruning (O(1) Check)
    auto it = min_cost_for_state.find(encode_state(u, steps));
    if (it != min_cost_for_state.end() && g > it->second)
      continue;

    // 4. Target Check
    if (u == endNode)
    {
      if (g < global_best_cost)
      {
        global_best_cost = g;
        global_best_path = reconstruct_path(parent_idx);
      }
      continue;
    }

    // 5. Explore Neighbors
    int next_steps = steps + 1;
    // CRITICAL FIX: Ensure the path length does not exceed the sieve size due to cycles.
    if (next_steps >= (int)is_prime.size())
      continue;

    for (const auto &e : graph[u])
    {
      int v = e.to;

      // CORRECTNESS FIX: Bounded Cycle Check (O(20))
      if (BoundedCheckCycle(v, parent_idx))
        continue;

      // Calculate costs
      long long h_cost = h_cost_to_target[v];
      if (h_cost == INF)
        continue;

      long long next_edge_cost = step_cost(e, next_steps);
      long long ng = g + next_edge_cost;

      // --- Weighted Heuristic Calculation ---
      long long nf = ng + (long long)(h_cost * heuristic_weight);

      // 7. A* / Bound Pruning
      if (nf >= global_best_cost)
        continue;

      // 8. Dominance Pruning (Check if this new state is dominated)
      long long next_state_key = encode_state(v, next_steps);
      auto it_next = min_cost_for_state.find(next_state_key);

      if (it_next != min_cost_for_state.end() && ng >= it_next->second)
        continue;

      // This state is promising! Update dominance map and push.
      min_cost_for_state[next_state_key] = ng;

      // Store path info before pushing to PQ
      int new_parent_idx = path_storage.size();
      path_storage.emplace_back(v, parent_idx);

      // Push to PQ
      pq.push(StateAStar{nf, ng, v, next_steps, new_parent_idx});
    }
  }
}

int main()
{
  ios::sync_with_stdio(false);
  cin.tie(nullptr);

  t_start = chrono::steady_clock::now();
  cin >> N >> M;
  cin >> startNode >> endNode;

  const int MAX_SIMPLE_PATH_LEN = N + 5;

  graph.assign(N, {});
  for (int i = 0; i < M; ++i)
  {
    int u, v, w1, w2;
    cin >> u >> v >> w1 >> w2;
    graph[u].push_back({v, w1, w2});
    graph[v].push_back({u, w1, w2});
  }

  sieve_primes(MAX_SIMPLE_PATH_LEN);

  // --- Phase 1: Heuristics & Seeding ---
  calculate_reverse_dijkstra_heuristic(endNode);

  // Run aggressive seeding for 5.0s max
  vector<double> alphas = {0.1, 0.5, 0.75, 1.0, 1.5, 2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 3.5, 3.75, 4.0, 4.5, 5.0, 6.0, 7.5, 10.0, 15.0};

  auto path_static = dijkstra_static_path(startNode, endNode);
  if (!path_static.empty())
  {
    global_best_path = path_static;
    global_best_cost = calculate_blackie_length(path_static);
  }

  for (double alpha : alphas)
  {
    if (chrono::duration<double>(chrono::steady_clock::now() - t_start).count() > TIME_LIMIT_SEEDING)
    {
      break;
    }
    auto path = dijkstra_alpha(startNode, endNode, alpha);
    if (!path.empty())
    {
      long long cost = calculate_blackie_length(path);
      if (cost < global_best_cost)
      {
        global_best_cost = cost;
        global_best_path = path;
      }
    }
  }

  if (global_best_path.empty())
  {
    if (startNode == endNode)
    {
      cout << "1\n"
           << startNode << "\n";
    }
    else
    {
      cout << "0\n\n";
    }
    return 0;
  }

  // --- Phase 3: Run the Main Iterative Weighted A* Search ---

  // Iterate from aggressive (1.30) down to optimal (1.00)
  vector<double> heuristic_weights = {1.30, 1.25, 1.20, 1.15, 1.10, 1.07, 1.04, 1.02, 1.01, 1.00};

  for (double w : heuristic_weights)
  {
    if (time_limit_exceeded)
    {
      break;
    }

    find_path_iterative(w);
  }

  // --- Final Output ---
  cout << global_best_path.size() << "\n";
  for (size_t i = 0; i < global_best_path.size(); ++i)
  {
    if (i)
      cout << " ";
    cout << global_best_path[i];
  }
  cout << "\n";

  return 0;
}