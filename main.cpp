#include <algorithm>
#include <chrono>
#include <cstdint>
#include <climits>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

// ---------------------- Problem types ----------------------
struct Edge
{
    int to;
    int w1;
    int w2;
};

static inline long long static_cost(int w1, int w2)
{
    return min<long long>(w1, 3LL * w2);
}

static const long long INF = (1LL << 62);

// ---------------------- Prime helper ----------------------
struct Prime
{
    vector<char> is; // is[n] == 1 if prime
    void ensure(int n)
    {
        if (n < 2)
        {
            is.assign(3, 0);
            is[2] = 1;
            return;
        }
        if ((int)is.size() > n)
            return;
        vector<char> tmp(n + 1, true);
        tmp[0] = tmp[1] = false;
        for (long long p = 2; p * p <= n; ++p)
        {
            if (tmp[(int)p])
            {
                for (long long x = p * p; x <= n; x += p)
                    tmp[(int)x] = false;
            }
        }
        is.swap(tmp);
    }
    inline bool operator()(int x)
    {
        if (x < 0)
            return false;
        if (x >= (int)is.size())
            ensure(x);
        return is[x];
    }
};

// ---------------------- Heuristics ----------------------
vector<long long> reverse_dijkstra(int N, int target, const vector<vector<Edge>> &adj)
{
    vector<vector<pair<int, long long>>> rev(N);
    for (int u = 0; u < N; ++u)
    {
        for (const auto &e : adj[u])
        {
            rev[e.to].push_back({u, static_cost(e.w1, e.w2)});
        }
    }
    vector<long long> dist(N, INF);
    priority_queue<pair<long long, int>, vector<pair<long long, int>>, greater<pair<long long, int>>> pq;
    dist[target] = 0;
    pq.push({0, target});
    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (d != dist[u])
            continue;
        for (auto &pr : rev[u])
        {
            int v = pr.first;
            long long w = pr.second;
            if (dist[v] > d + w)
            {
                dist[v] = d + w;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}

vector<int> bfs_hops_to_target(int N, int target, const vector<vector<Edge>> &adj)
{
    vector<int> hops(N, -1);
    queue<int> q;
    q.push(target);
    hops[target] = 0;
    while (!q.empty())
    {
        int u = q.front();
        q.pop();
        for (const auto &e : adj[u])
        {
            int v = e.to;
            if (hops[v] == -1)
            {
                hops[v] = hops[u] + 1;
                q.push(v);
            }
        }
    }
    return hops;
}

// ---------------------- Path validation & utilities ----------------------
static inline bool edge_exists(const vector<vector<Edge>> &adj, int u, int v)
{
    for (const auto &e : adj[u])
        if (e.to == v)
            return true;
    return false;
}

static bool validate_path_simple(const vector<int> &path, int S, int T, const vector<vector<Edge>> &adj)
{
    if (path.empty())
        return false;
    if (path.front() != S)
        return false;
    if (path.back() != T)
        return false;
    unordered_set<int> seen;
    seen.reserve(path.size() * 2);
    for (size_t i = 0; i < path.size(); ++i)
    {
        int u = path[i];
        if (u < 0 || u >= (int)adj.size())
            return false;
        if (!seen.insert(u).second)
            return false; // repeated vertex -> not simple
        if (i)
        {
            if (!edge_exists(adj, path[i - 1], u))
                return false;
        }
    }
    return true;
}

static long long true_blackie_cost(const vector<int> &path, const vector<vector<Edge>> &adj, Prime &prime)
{
    if (path.size() <= 1)
        return 0;
    long long sum = 0;
    for (size_t i = 1; i < path.size(); ++i)
    {
        int u = path[i - 1], v = path[i];
        int step = (int)i; // e1 is step 1, etc.
        long long best_edge = INF;
        for (const auto &e : adj[u])
        {
            if (e.to != v)
                continue;
            long long c = prime(step) ? (3LL * e.w2) : (long long)e.w1;
            if (c < best_edge)
                best_edge = c;
        }
        // graph guaranteed to have an edge u->v (validated before)
        sum += best_edge;
    }
    return sum;
}

// ---------------------- Fallback path (static-cost Dijkstra) ----------------------
vector<int> dijkstra_static_path(int N, int s, int t, const vector<vector<Edge>> &adj)
{
    vector<long long> dist(N, INF);
    vector<int> parent(N, -1);
    priority_queue<pair<long long, int>, vector<pair<long long, int>>, greater<pair<long long, int>>> pq;
    dist[s] = 0;
    pq.push({0, s});
    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (d != dist[u])
            continue;
        if (u == t)
            break;
        for (const auto &e : adj[u])
        {
            long long w = static_cost(e.w1, e.w2);
            if (dist[e.to] > d + w)
            {
                dist[e.to] = d + w;
                parent[e.to] = u;
                pq.push({dist[e.to], e.to});
            }
        }
    }
    vector<int> path;
    if (dist[t] == INF)
        return path;
    for (int v = t; v != -1; v = parent[v])
        path.push_back(v);
    reverse(path.begin(), path.end());
    return path;
}

// ---------------------- A* over layered states (node, steps) ----------------------
struct AStarResult
{
    vector<int> path;
    long long cost = -1;
};
struct KeyHash
{
    size_t operator()(const uint64_t &k) const noexcept { return (size_t)(k ^ (k >> 33)); }
};

// Label skyline per node: (steps, g) non-dominated
static inline bool dominated_and_update(vector<vector<pair<int, long long>>> &lab, int u, int s, long long g)
{
    auto &vec = lab[u];
    // dominated by existing?
    for (auto &p : vec)
    {
        if (p.first <= s && p.second <= g)
            return true; // dominated
    }
    // remove labels dominated by (s,g)
    int w = 0;
    for (int i = 0; i < (int)vec.size(); ++i)
    {
        auto &p = vec[i];
        if (s <= p.first && g <= p.second)
            continue; // dominated -> skip
        vec[w++] = p;
    }
    vec.resize(w);
    vec.push_back({s, g});
    return false;
}

AStarResult astar_limited(
    int N, int s, int t,
    const vector<vector<Edge>> &adj,
    const vector<long long> &h,        // admissible heuristic (static reverse Dijkstra)
    const vector<int> &hops_to_target, // to cap length window
    Prime &prime,
    int Lmax,
    double weight_h,
    chrono::steady_clock::time_point hard_deadline,
    long long incumbent_upper_bound // use a real, valid UB to prune hard
)
{
    struct StateRec
    {
        int node;
        int steps;
        int parent_id;
    };
    vector<StateRec> states;
    states.reserve(1 << 20);
    states.push_back({s, 0, -1}); // id 0

    vector<long long> gval;
    gval.reserve(1 << 20);
    gval.push_back(0);

    struct PQN
    {
        long long f, g;
        int id;
        bool operator<(PQN const &o) const
        {
            if (f != o.f)
                return f > o.f;
            return g > o.g;
        }
    };
    priority_queue<PQN> open;
    auto f0 = (long long)(gval[0] + weight_h * (h[s] == INF ? (long long)4e18 : h[s]));
    open.push({f0, 0, 0});

    // skyline labels per node
    vector<vector<pair<int, long long>>> labels(N); // typically very small
    labels[s].push_back({0, 0});

    long long incumbent_cost = incumbent_upper_bound; // start with real UB
    int incumbent_id = -1;

    // helper: simple-path enforcement (no repeated vertices)
    auto repeats_on_chain = [&](int id, int v) -> bool
    {
        for (int cur = id; cur != -1; cur = states[cur].parent_id)
        {
            if (states[cur].node == v)
                return true;
        }
        return false;
    };

    while (!open.empty())
    {
        if (chrono::steady_clock::now() > hard_deadline)
            break;

        auto top = open.top();
        open.pop();
        long long g = top.g;
        int id = top.id;
        if (gval[id] != g)
            continue; // stale

        int u = states[id].node;
        int steps = states[id].steps;

        // global B&B
        if (g >= incumbent_cost)
            continue;

        // f-based early exit (if proven optimal within this Lmax & weight)
        if (u == t)
        {
            // we keep only if improves the UB; but record parent for reconstruction too
            if (g < incumbent_cost)
            {
                incumbent_cost = g;
                incumbent_id = id;
            }
            if (!open.empty() && open.top().f >= incumbent_cost)
                break;
            continue;
        }

        // neighbors (already presorted by static cost)
        for (const auto &e : adj[u])
        {
            int v = e.to;
            if (v == u)
                continue; // self loop
            // avoid immediate backtrack
            if (states[id].parent_id != -1 && states[states[id].parent_id].node == v)
                continue;
            // enforce simple path
            if (repeats_on_chain(id, v))
                continue;

            // unreachable tail from v?
            int ht = hops_to_target[v];
            if (ht == -1)
                continue;
            int next_steps = steps + 1;
            if (next_steps + ht > Lmax)
                continue;
            if (h[v] == INF)
                continue;

            long long edge_cost = prime(next_steps) ? (3LL * e.w2) : (long long)e.w1;
            long long ng = g + edge_cost;
            if (ng >= incumbent_cost)
                continue;

            // skyline dominance at v
            if (dominated_and_update(labels, v, next_steps, ng))
                continue;

            states.push_back({v, next_steps, id});
            gval.push_back(ng);
            long long hh = h[v];
            long long ff = ng + (long long)(weight_h * (hh == INF ? (long long)4e18 : hh));
            open.push({ff, ng, (int)states.size() - 1});
        }
    }

    AStarResult res;
    if (incumbent_id == -1)
    {
        res.cost = -1;
        return res;
    }
    vector<int> nodes;
    for (int cur = incumbent_id; cur != -1; cur = states[cur].parent_id)
        nodes.push_back(states[cur].node);
    reverse(nodes.begin(), nodes.end());
    res.path = std::move(nodes);
    res.cost = incumbent_cost;
    return res;
}

// ---------------------- Output ----------------------
static inline void print_path(const vector<int> &path)
{
    if (path.empty())
    {
        cout << "0\n\n";
        return;
    }
    cout << path.size() << "\n";
    for (size_t i = 0; i < path.size(); ++i)
    {
        if (i)
            cout << ' ';
        cout << path[i];
    }
    cout << "\n";
}

// ---------------------- Main ----------------------
int main()
{
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int N, M;
    if (!(cin >> N >> M))
        return 0;
    int S, T;
    cin >> S >> T;

    vector<vector<Edge>> adj(N);
    adj.reserve(N);
    for (int i = 0; i < M; ++i)
    {
        int u, v, w1, w2;
        cin >> u >> v >> w1 >> w2;
        // undirected
        adj[u].push_back({v, w1, w2});
        adj[v].push_back({u, w1, w2});
    }

    // Presort adjacency by static-cost ascending => better incumbents early
    for (int u = 0; u < N; ++u)
    {
        auto &vec = adj[u];
        stable_sort(vec.begin(), vec.end(), [](const Edge &a, const Edge &b)
                    {
            long long ca = static_cost(a.w1, a.w2);
            long long cb = static_cost(b.w1, b.w2);
            if (ca != cb) return ca < cb;
            return a.to < b.to; });
    }

    const double TIME_LIMIT = 18.5; // leave headroom under 20s
    auto start_tp = chrono::steady_clock::now();
    auto deadline = start_tp + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                   std::chrono::duration<double>(TIME_LIMIT));

    vector<long long> h = reverse_dijkstra(N, T, adj);
    vector<int> hops_to_target = bfs_hops_to_target(N, T, adj);
    Prime prime;

    // Always-valid fallback (static-cost Dijkstra)
    vector<int> best_path = dijkstra_static_path(N, S, T, adj);
    if (!validate_path_simple(best_path, S, T, adj))
    {
        cout << "0\n\n"; // graph disconnected or malformed
        return 0;
    }

    // Strong UB: true Blackie cost of fallback
    prime.ensure((int)best_path.size() + 10);
    long long best_cost = true_blackie_cost(best_path, adj, prime);

    int min_hops = hops_to_target[S];
    if (min_hops == -1)
    { // disconnected
        print_path(best_path);
        return 0;
    }

    // Adaptive Lmax schedule based on min_hops and fallback length
    int Lfb = (int)best_path.size() - 1;
    vector<int> Ls;
    Ls.push_back(min_hops);
    if (Lfb > min_hops)
    {
        Ls.push_back(min(Lfb, min_hops + 50));
        Ls.push_back(min(Lfb, min_hops + 120));
        Ls.push_back(min(Lfb, min_hops + 240));
        Ls.push_back(min(Lfb, min_hops + 400));
    }
    // A little beyond fallback if time allows:
    Ls.push_back(min(Lfb + 20, min_hops + 600));
    Ls.push_back(min(Lfb + 60, min_hops + 800));

    // Phase 1: fast good incumbents
    const double W1[] = {1.6, 1.25, 1.1};
    for (int Lmax : Ls)
    {
        if (chrono::steady_clock::now() > deadline)
            break;
        prime.ensure(Lmax + 10);
        for (double w : W1)
        {
            if (chrono::steady_clock::now() > deadline)
                break;
            AStarResult ar = astar_limited(N, S, T, adj, h, hops_to_target, prime, Lmax, w, deadline, best_cost);
            if (ar.cost != -1 && validate_path_simple(ar.path, S, T, adj))
            {
                // compute true cost of returned path to avoid any rounding surprises
                long long tc = true_blackie_cost(ar.path, adj, prime);
                if (tc < best_cost)
                {
                    best_cost = tc;
                    best_path = std::move(ar.path);
                }
            }
        }
    }

    // Phase 2: targeted refine (w=1.0) around best length discovered so far
    if (chrono::steady_clock::now() < deadline)
    {
        int Lbest = (int)best_path.size() - 1;
        vector<int> Lref = {max(min_hops, Lbest), min(Lbest + 20, min_hops + 800)};
        for (int Lmax : Lref)
        {
            if (chrono::steady_clock::now() > deadline)
                break;
            prime.ensure(Lmax + 10);
            AStarResult ar = astar_limited(N, S, T, adj, h, hops_to_target, prime, Lmax, 1.0, deadline, best_cost);
            if (ar.cost != -1 && validate_path_simple(ar.path, S, T, adj))
            {
                long long tc = true_blackie_cost(ar.path, adj, prime);
                if (tc < best_cost)
                {
                    best_cost = tc;
                    best_path = std::move(ar.path);
                }
            }
        }
    }

    print_path(best_path);
    return 0;
}
