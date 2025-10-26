#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <bitset> // Konieczny do unikania cykli i MLX
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <climits>
#include <algorithm>
#include <map>
#include <memory>
#include <limits> // Dla infinity

using namespace std;

vector<bool> sieve_primes(int n) {
    vector<bool> prime(n + 1, true);
    prime[0] = prime[1] = false;
    if (n >= 2) {
        for (int i = 4; i <= n; i += 2) prime[i] = false;
        for (int i = 3; i * i <= n; i += 2) {
            if (prime[i]) {
                for (int j = i * i; j <= n; j += 2 * i) {
                    prime[j] = false;
                }
            }
        }
    }
    return prime;
}

struct PairHash {
    size_t operator()(const pair<int, int>& p) const {
        return ((size_t)p.first << 20) ^ p.second;
    }
};
// --- Stałe ---
const int MAX_N = 100005;
// Szerokość wiązki - kompromis między jakością a wydajnością
const int BEAM_WIDTH = 4000;
const double TIME_LIMIT = 18.0;
const int MAX_SIEVE_LEN = 200000;
const long long INF = numeric_limits<long long>::max();

// --- Struktury danych ---

struct Edge
{
  int to;
  int w1;
  int w2;
};

// Krawędź dla Dijkstry (odwróconej)
struct DijkstraEdge
{
  int to;
  long long static_cost;
};

vector<bool> is_prime_sieve;
vector<long long> min_cost_to_target; // NOWA HEURYSTYKA A*

struct Path
{
  int current_node;
  long long cost;
  int path_len;
  bitset<MAX_N> visited;
  shared_ptr<Path> parent;

  Path(int start_node)
      : current_node(start_node), cost(0), path_len(0), parent(nullptr)
  {
    visited[start_node] = 1;
  }

  Path(const shared_ptr<Path> &parent_ptr, const Edge &edge, long long edge_cost)
  {
    this->current_node = edge.to;
    this->path_len = parent_ptr->path_len + 1;
    this->cost = parent_ptr->cost + edge_cost;
    this->visited = parent_ptr->visited; // Kopia bitsetu
    this->visited[edge.to] = 1;
    this->parent = parent_ptr;
  }
};

using PathPtr = shared_ptr<Path>;

// --- Heurystyka A* ---

/**
 * @brief Oblicza heurystykę h(v) - koszt dotarcia do celu.
 */
inline long long heuristic(int node)
{
  return min_cost_to_target[node]; // Zwraca pre-obliczony koszt Dijkstry
}

/**
 * @brief Oblicza f(p) = g(p) + h(p) = path->cost + heuristic(path->node)
 */
inline long long get_f_cost(const PathPtr &p)
{
  long long h_cost = heuristic(p->current_node);
  if (h_cost == INF)
    return INF; // Nieosiągalny
  // Unikaj przepełnienia
  if (p->cost > INF - h_cost)
    return INF;
  return p->cost + h_cost;
}

/**
 * @brief Komparator dla Max-Heap (nasza "ograniczona" wiązka).
 * Sortuje po 'f_cost'.
 */
struct ComparePathPtrMax
{
  bool operator()(const PathPtr &a, const PathPtr &b)
  {
    long long f_a = get_f_cost(a);
    long long f_b = get_f_cost(b);
    if (f_a != f_b)
    {
      return f_a < f_b; // Standardowy Max-Heap (gorszy = większy f_cost)
    }
    // TIE-BREAKER: f_cost jest taki sam.
    // Preferuj ścieżkę, która ma mniej pracy do wykonania (mniejsze h).
    // (Większe h jest "gorsze", więc idzie wyżej w Max-Heap)
    return heuristic(a->current_node) > heuristic(b->current_node);
  }
};

// --- Funkcje pomocnicze ---

bool is_prime_slow(int n)
{
  if (n <= 1)
    return false;
  for (long long i = 2; i * i <= n; ++i)
  {
    if (n % i == 0)
      return false;
  }
  return true;
}

void sieve(int max_n)
{
  is_prime_sieve.assign(max_n + 1, true);
  is_prime_sieve[0] = is_prime_sieve[1] = false;
  for (int p = 2; p * p <= max_n; p++)
  {
    if (is_prime_sieve[p])
    {
      for (int i = p * p; i <= max_n; i += p)
        is_prime_sieve[i] = false;
    }
  }
}

long long get_edge_cost(int w1, int w2, int path_index)
{
  bool is_p;
  if (path_index < is_prime_sieve.size())
  {
    is_p = is_prime_sieve[path_index];
  }
  else
  {
    is_p = is_prime_slow(path_index);
  }
  return is_p ? (3LL * w2) : (1LL * w1);
}

void print_path(PathPtr solution)
{
  if (solution == nullptr)
  {
    cout << "0\n\n";
    return;
  }
  vector<int> nodes;
  PathPtr current = solution;
  while (current != nullptr)
  {
    nodes.push_back(current->current_node);
    current = current->parent;
  }
  reverse(nodes.begin(), nodes.end());

  cout << nodes.size() << "\n";
  for (size_t i = 0; i < nodes.size(); ++i)
  {
    cout << nodes[i] << (i == nodes.size() - 1 ? "" : " ");
  }
  cout << "\n";
}

void print_bfs_path(const vector<int> &path)
{
  if (path.empty())
  {
    cout << "0\n\n";
    return;
  }
  cout << path.size() << "\n";
  for (size_t i = 0; i < path.size(); ++i)
  {
    cout << path[i] << (i == path.size() - 1 ? "" : " ");
  }
  cout << "\n";
}

// --- Funkcje Obliczeniowe (Heurystyka i Fallback) ---

/**
 * @brief Uruchamia Dijkstrę od celu, aby wypełnić 'min_cost_to_target' dla A*.
 */
void precalculate_heuristic_dijkstra(int N, int end_node, const vector<vector<DijkstraEdge>> &adj_rev)
{
  min_cost_to_target.assign(N, INF);

  // Min-Heap dla Dijkstry: <koszt, węzeł>
  priority_queue<pair<long long, int>, vector<pair<long long, int>>, greater<pair<long long, int>>> pq;

  min_cost_to_target[end_node] = 0;
  pq.push({0, end_node});

  while (!pq.empty())
  {
    long long d = pq.top().first;
    int u = pq.top().second;
    pq.pop();

    if (d > min_cost_to_target[u])
    {
      continue; // Już znaleziono lepszą ścieżkę
    }

    for (const auto &edge : adj_rev[u])
    {
      int v = edge.to;
      long long new_dist = d + edge.static_cost;

      if (new_dist < min_cost_to_target[v])
      {
        min_cost_to_target[v] = new_dist;
        pq.push({new_dist, v});
      }
    }
  }
}

vector<int> find_path_bfs_fallback(int start, int dest, const vector<vector<Edge>> &adj)
{
  queue<int> q;
  map<int, int> parent;
  vector<bool> visited(adj.size(), false);

  q.push(start);
  visited[start] = true;
  parent[start] = -1;

  while (!q.empty())
  {
    int u = q.front();
    q.pop();
    if (u == dest)
    {
      vector<int> path;
      int curr = dest;
      while (curr != -1)
      {
        path.push_back(curr);
        curr = parent[curr];
      }
      reverse(path.begin(), path.end());
      return path;
    }
    for (const auto &edge : adj[u])
    {
      if (!visited[edge.to])
      {
        visited[edge.to] = true;
        parent[edge.to] = u;
        q.push(edge.to);
      }
    }
  }
  return {};
}

// --- Główna funkcja ---

int main()
{
  ios_base::sync_with_stdio(false);
  cin.tie(NULL);

  int N, M;
  cin >> N >> M;

  if(N<50000){  
  int start_node, end_node;
  cin >> start_node >> end_node;

  vector<vector<Edge>> adj(N);
  vector<vector<DijkstraEdge>> adj_rev(N); // Graf odwrócony dla Dijkstry

  for (int i = 0; i < M; ++i)
  {
    int u, v, w1, w2;
    cin >> u >> v >> w1 >> w2;

    adj[u].push_back({v, w1, w2});
    adj[v].push_back({u, w1, w2});

    // Oblicz 'static_cost' dla heurystyki
    long long static_cost = min((long long)w1, 3LL * w2);

    // Zbuduj graf odwrócony
    adj_rev[v].push_back({u, static_cost});
    adj_rev[u].push_back({v, static_cost});
  }

  // 1. Pre-obliczenia
  sieve(MAX_SIEVE_LEN);
  precalculate_heuristic_dijkstra(N, end_node, adj_rev); // NOWA heurystyka
  auto start_time = chrono::high_resolution_clock::now();

  // 2. Inicjalizacja LBS
  vector<PathPtr> current_beams;
  current_beams.push_back(make_shared<Path>(start_node));

  PathPtr best_solution = nullptr;
  long long min_solution_cost = -1;

  using CappedPQ = priority_queue<PathPtr, vector<PathPtr>, ComparePathPtrMax>;

  // 3. Pętla Beam Search
  while (true)
  {
    auto now = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = now - start_time;
    if (elapsed.count() > TIME_LIMIT)
    {
      break;
    }
    if (current_beams.empty())
    {
      break;
    }

    CappedPQ next_beam_candidates;

    for (const auto &parent_path : current_beams)
    {
      int u = parent_path->current_node;

      // Pruning (przycinanie): Jeśli koszt f(p) jest już gorszy niż
      // najlepsze znalezione rozwiązanie, nie eksploruj tej gałęzi.
      long long f_p = get_f_cost(parent_path);
      if (f_p == INF || (best_solution != nullptr && f_p >= min_solution_cost))
      {
        continue;
      }

      for (const auto &edge : adj[u])
      {
        int v = edge.to;

        // Sprawdzanie cykli
        if (parent_path->visited[v] == 0)
        {

          int new_path_len = parent_path->path_len + 1;
          long long edge_cost = get_edge_cost(edge.w1, edge.w2, new_path_len);

          PathPtr next_path = make_shared<Path>(parent_path, edge, edge_cost);

          if (v == end_node)
          {
            if (min_solution_cost == -1 || next_path->cost < min_solution_cost)
            {
              min_solution_cost = next_path->cost;
              best_solution = next_path;
            }
          }
          else
          {
            // Logika "Ograniczonej Kolejki" (Capped Heap)
            long long f_next = get_f_cost(next_path);
            if (f_next == INF)
              continue; // Ta ścieżka nie prowadzi do celu

            if (next_beam_candidates.size() < BEAM_WIDTH)
            {
              next_beam_candidates.push(next_path);
            }
            else if (f_next < get_f_cost(next_beam_candidates.top()))
            {
              next_beam_candidates.pop();
              next_beam_candidates.push(next_path);
            }
          }
        }
      }
    }

    current_beams.clear();
    while (!next_beam_candidates.empty())
    {
      current_beams.push_back(next_beam_candidates.top());
      next_beam_candidates.pop();
    }
  }

  // 4. Wypisanie wyniku i plan awaryjny
  if (best_solution != nullptr)
  {
    print_path(best_solution);
  }
  else
  {
    vector<int> bfs_path = find_path_bfs_fallback(start_node, end_node, adj);
    print_bfs_path(bfs_path.empty() ? vector<int>{start_node} : bfs_path);
  }
}else{
        int source, dest;
    cin >> source >> dest;

    vector<vector<tuple<int, int, int>>> graph(N);

    for (int i = 0; i < M; i++) {
        int u, v, w1, w2;
        cin >> u >> v >> w1 >> w2;
        graph[u].emplace_back(v, w1, w2);
        graph[v].emplace_back(u, w1, w2);
    }

    int max_depth = min(N - 1, max(1000, N / 10));
    if (max_depth < 0) max_depth = 0;
    vector<bool> prime = sieve_primes(max_depth + 5);

    priority_queue<tuple<long long, int, int>, vector<tuple<long long, int, int>>, greater<>> pq;

    unordered_map<pair<int, int>, long long, PairHash> dist;
    unordered_map<pair<int, int>, pair<int, int>, PairHash> parent;

    pq.push({0, source, 0});
    dist[{source, 0}] = 0;

    bitset<100001> visited;
    visited[source] = true;

    pair<int, int> best_state = {-1, -1};
    long long best_cost = LLONG_MAX;

    while (!pq.empty()) {
        auto [cost, node, edge_count] = pq.top();
        pq.pop();

        if (cost >= best_cost) continue;

        if (node == dest) {
            if (cost < best_cost) {
                best_cost = cost;
                best_state = {node, edge_count};
            }
            continue;
        }

        for (const auto& [neighbor, w1, w2] : graph[node]) {
            int new_edge_count = edge_count + 1;

            long long edge_cost = (new_edge_count < (int)prime.size() && prime[new_edge_count]) ? 3LL * w2 : (long
long)w1;
            long long new_cost = cost + edge_cost;

            if (visited[neighbor]) continue;

            pair<int, int> new_state = {neighbor, new_edge_count};
            if (!dist.count(new_state) || dist[new_state] > new_cost) {
                dist[new_state] = new_cost;
                parent[new_state] = {node, edge_count};
                pq.push({new_cost, neighbor, new_edge_count});
            }

            visited[neighbor] = true;
        }
    }

    if (best_state.first == -1) {
        priority_queue<pair<long long, int>, vector<pair<long long, int>>, greater<>> simple_pq;
        vector<long long> simple_dist(N, LLONG_MAX);
        vector<int> simple_parent(N, -1);

        simple_pq.push({0, source});
        simple_dist[source] = 0;

        while (!simple_pq.empty()) {
            auto [cost, node] = simple_pq.top();
            simple_pq.pop();

            if (cost > simple_dist[node]) continue;
            if (node == dest) break;

            for (const auto& [neighbor, w1, w2] : graph[node]) {
                long long new_cost = cost + min(w1, 3 * w2);
                if (new_cost < simple_dist[neighbor]) {
                    simple_dist[neighbor] = new_cost;
                    simple_parent[neighbor] = node;
                    simple_pq.push({new_cost, neighbor});
                }
            }
        }

        if (simple_dist[dest] == LLONG_MAX) {
            cout << 0 << "\n";
            return 0;
        }

        vector<int> path;
        int curr = dest;
        while (curr != -1) {
            path.push_back(curr);
            curr = simple_parent[curr];
        }
        reverse(path.begin(), path.end());

        cout << path.size() << "\n";
        for (size_t i = 0; i < path.size(); i++) {
            if (i > 0) cout << " ";
            cout << path[i];
        }
        cout << "\n";
        return 0;
    }

    vector<int> path;
    pair<int, int> current = best_state;

    while (current.first != -1) {
        path.push_back(current.first);
        if (parent.count(current)) {
            current = parent[current];
        } else {
            break;
        }
    }

    reverse(path.begin(), path.end());

    cout << path.size() << "\n";
    for (size_t i = 0; i < path.size(); i++) {
        if (i > 0) cout << " ";
        cout << path[i];
    }
    cout << "\n";

}

  return 0;
}