#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <bitset>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <map>
#include <memory>
#include <limits>

using namespace std;

// --- Stałe ---
const int MAX_N = 100005;

// Zmniejszamy wiązkę, aby rozwiązać problem MLX
const int BEAM_WIDTH = 5000;
const double TIME_LIMIT = 18.5;
const int MAX_SIEVE_LEN = 200000;
const long long INF = numeric_limits<long long>::max();
const double HEURISTIC_WEIGHT = 1.0;

// --- Struktury danych ---

struct Edge
{
    int to;
    int w1;
    int w2;
};

struct DijkstraEdge
{
    int to;
    long long static_cost;
};

vector<bool> is_prime_sieve;
vector<long long> min_cost_to_target; // Heurystyka A* (h)

struct Path
{
    int current_node;
    long long cost;        // 'g' RZECZYWISTE (g_real)
    long long heuristic_g; // 'g' HEURYSTYCZNE (g_optimistic) - dla TIE-BREAKER
    int path_len;
    shared_ptr<Path> parent;
    bitset<MAX_N> visited;

    Path(int start_node)
        : current_node(start_node), cost(0), heuristic_g(0), path_len(0), parent(nullptr)
    {
        visited[start_node] = true;
    }

    Path(const shared_ptr<Path> &parent_ptr, const Edge &edge, long long edge_cost, long long static_edge_cost)
    {
        this->current_node = edge.to;
        this->path_len = parent_ptr->path_len + 1;
        this->parent = parent_ptr;
        this->cost = parent_ptr->cost + edge_cost;
        this->heuristic_g = parent_ptr->heuristic_g + static_edge_cost;
        this->visited = parent_ptr->visited;
        this->visited[edge.to] = true;
    }
};

using PathPtr = shared_ptr<Path>;

// --- Heurystyka A* ---

inline long long heuristic(int node)
{
    return min_cost_to_target[node]; // h
}

/**
 * @brief Oblicza f(p) = g_real + h
 */
inline long long get_f_cost(const PathPtr &p)
{
    long long h_cost = heuristic(p->current_node);
    if (h_cost == INF)
        return INF;

    // POWRÓT DO LEPSZEJ FORMUŁY: f = g_real + h
    if (p->cost > INF - h_cost)
        return INF;
    return p->cost + h_cost;
}

/**
 * @brief Komparator dla Max-Heap z wielopoziomowym TIE-BREAKEREM.
 */
struct ComparePathPtrMax
{
    bool operator()(const PathPtr &a, const PathPtr &b)
    {
        // Poziom 1: Sortuj po f = g_real + h
        long long f_a = get_f_cost(a);
        long long f_b = get_f_cost(b);
        if (f_a != f_b)
        {
            return f_a < f_b; // Gorszy = większy f_cost
        }

        // Poziom 2 (Tie-Breaker): f jest równe. Sortuj po g_optimistic.
        // Wolimy ścieżki o niższym koszcie optymistycznym (lepszy potencjał).
        if (a->heuristic_g != b->heuristic_g)
        {
            return a->heuristic_g > b->heuristic_g; // Gorszy = większy g_optimistic
        }

        // Poziom 3 (Tie-Breaker): Oba koszty równe. Sortuj po h.
        // Wolimy ścieżki bliżej celu (niższe h).
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

// Zwraca optymistyczny koszt krawędzi (dla heurystyki i tie-breakera)
inline long long get_static_cost(int w1, int w2)
{
    return min((long long)w1, 3LL * w2);
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

void precalculate_heuristic_dijkstra(int N, int end_node, const vector<vector<DijkstraEdge>> &adj_rev)
{
    min_cost_to_target.assign(N, INF);
    priority_queue<pair<long long, int>, vector<pair<long long, int>>, greater<pair<long long, int>>> pq;
    min_cost_to_target[end_node] = 0;
    pq.push({0, end_node});
    while (!pq.empty())
    {
        long long d = pq.top().first;
        int u = pq.top().second;
        pq.pop();
        if (d > min_cost_to_target[u])
            continue;
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

vector<int> find_path_bfs_fallback(int N, int start, int dest, const vector<vector<Edge>> &adj)
{
    queue<int> q;
    map<int, int> parent;
    vector<bool> visited_bfs(N, false);
    q.push(start);
    visited_bfs[start] = true;
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
            if (!visited_bfs[edge.to])
            {
                visited_bfs[edge.to] = true;
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
    int start_node, end_node;
    cin >> start_node >> end_node;

    vector<vector<Edge>> adj(N);
    vector<vector<DijkstraEdge>> adj_rev(N);

    for (int i = 0; i < M; ++i)
    {
        int u, v, w1, w2;
        cin >> u >> v >> w1 >> w2;
        adj[u].push_back({v, w1, w2});
        adj[v].push_back({u, w1, w2});

        long long static_cost = get_static_cost(w1, w2);

        adj_rev[v].push_back({u, static_cost});
        adj_rev[u].push_back({v, static_cost});
    }

    // 1. Pre-obliczenia
    sieve(min(MAX_N - 1, MAX_SIEVE_LEN));
    precalculate_heuristic_dijkstra(N, end_node, adj_rev);
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
            break;
        if (current_beams.empty())
            break;

        CappedPQ next_beam_candidates;

        for (const auto &parent_path : current_beams)
        {
            int u = parent_path->current_node;

            // Używamy f = g_real + h do przycinania
            long long f_p = get_f_cost(parent_path);
            if (f_p == INF || (best_solution != nullptr && f_p >= min_solution_cost))
            {
                continue;
            }

            for (const auto &edge : adj[u])
            {
                int v = edge.to;

                if (parent_path->visited[v] == 0)
                {

                    int new_path_len = parent_path->path_len + 1;

                    long long real_edge_cost = get_edge_cost(edge.w1, edge.w2, new_path_len);
                    long long static_edge_cost = get_static_cost(edge.w1, edge.w2);

                    PathPtr next_path = make_shared<Path>(parent_path, edge, real_edge_cost, static_edge_cost);

                    if (v == end_node)
                    {
                        // "Sędzia": Użyj RZECZYWISTEGO kosztu (next_path->cost)
                        if (min_solution_cost == -1 || next_path->cost < min_solution_cost)
                        {
                            min_solution_cost = next_path->cost;
                            best_solution = next_path;
                        }
                    }
                    else
                    {
                        // "Przewodnik": Użyj f = g_real + h
                        long long f_next = get_f_cost(next_path);
                        if (f_next == INF)
                            continue;

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
        vector<int> bfs_path = find_path_bfs_fallback(N, start_node, end_node, adj);
        print_bfs_path(bfs_path.empty() ? vector<int>{start_node} : bfs_path);
    }

    return 0;
}