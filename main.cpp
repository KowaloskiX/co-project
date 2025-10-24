#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <limits>
#include <algorithm>
#include <cmath>
#include <random>
#include <chrono>
#include <unordered_set>

using namespace std;
using clk = std::chrono::steady_clock;

struct Edge
{
    int to, w1, w2;
};

static inline vector<bool> sieve_primes(int n)
{
    vector<bool> p(n + 1, true);
    if (n >= 0)
        p[0] = false;
    if (n >= 1)
        p[1] = false;
    for (int i = 2; 1LL * i * i <= n; ++i)
        if (p[i])
            for (long long j = 1LL * i * i; j <= n; j += i)
                p[(int)j] = false;
    return p;
}

struct DijkstraResult
{
    vector<int> parent;
    vector<long long> dist;
};

template <class CostFn>
static inline DijkstraResult dijkstra(int N, const vector<vector<Edge>> &g, int s, CostFn cost_of)
{
    const long long INF = (1LL << 62);
    vector<long long> dist(N, INF);
    vector<int> parent(N, -1);
    using P = pair<long long, int>;
    priority_queue<P, vector<P>, greater<P>> pq;
    dist[s] = 0;
    pq.push({0, s});
    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (d != dist[u])
            continue;
        for (const auto &e : g[u])
        {
            long long w = cost_of(e.w1, e.w2);
            long long nd = d + w;
            int v = e.to;
            if (nd < dist[v])
            {
                dist[v] = nd;
                parent[v] = u;
                pq.push({nd, v});
            }
        }
    }
    return {move(parent), move(dist)};
}

static inline vector<int> reconstruct_path(int s, int t, const vector<int> &par)
{
    vector<int> path;
    if (t < 0)
        return path;
    for (int v = t; v != -1; v = par[v])
        path.push_back(v);
    reverse(path.begin(), path.end());
    if (path.empty() || path.front() != s)
        path.clear();
    return path;
}

static inline long long blackie_cost_of_path(const vector<int> &path,
                                             const vector<vector<Edge>> &g,
                                             const vector<bool> &is_prime)
{
    if (path.size() < 2)
        return 0;
    long long cost = 0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        int u = path[i], v = path[i + 1];
        int best_w1 = numeric_limits<int>::max();
        int best_w2 = numeric_limits<int>::max();
        for (const auto &e : g[u])
            if (e.to == v)
            {
                if (e.w1 < best_w1)
                    best_w1 = e.w1;
                if (e.w2 < best_w2)
                    best_w2 = e.w2;
            }
        if (best_w1 == numeric_limits<int>::max())
            return (1LL << 62);
        int pos = (int)i + 1; // 1-indexed
        cost += (pos < (int)is_prime.size() && is_prime[pos]) ? 3LL * best_w2 : (long long)best_w1;
    }
    return cost;
}

static inline int count_primes_up_to(int L, const vector<bool> &is_prime)
{
    if (L >= (int)is_prime.size())
        L = (int)is_prime.size() - 1;
    int cnt = 0;
    for (int i = 0; i <= L; ++i)
        if (is_prime[i])
            ++cnt;
    return cnt;
}

static vector<int> bfs_hops(int N, const vector<vector<Edge>> &g, int src)
{
    vector<int> d(N, -1);
    queue<int> q;
    d[src] = 0;
    q.push(src);
    while (!q.empty())
    {
        int u = q.front();
        q.pop();
        for (const auto &e : g[u])
        {
            int v = e.to;
            if (d[v] == -1)
            {
                d[v] = d[u] + 1;
                q.push(v);
            }
        }
    }
    return d;
}

// A compact path signature to dedupe candidates in sets
static string path_signature(const vector<int> &p)
{
    string s;
    s.reserve(p.size() * 5);
    for (size_t i = 0; i < p.size(); ++i)
    {
        int x = p[i];
        s.append(reinterpret_cast<const char *>(&x), sizeof(int));
    }
    return s;
}

// Yen's K-shortest simple paths on a given metric (small K, guarded by time/size)
template <class CostFn>
static void yens_k_shortest_simple_paths(
    int N, const vector<vector<Edge>> &g, int S, int T,
    CostFn cost_of, int K,
    const clk::time_point &t0, double time_budget_sec,
    vector<vector<int>> &out_paths)
{
    // First shortest
    auto base = dijkstra(N, g, S, cost_of);
    auto P0 = reconstruct_path(S, T, base.parent);
    if (P0.empty())
        return;
    out_paths.push_back(P0);

    using LL = long long;
    struct Cand
    {
        LL cost;
        vector<int> path;
    };
    auto path_cost_metric = [&](const vector<int> &path) -> LL
    {
        if (path.size() < 2)
            return 0;
        LL c = 0;
        for (size_t i = 0; i + 1 < path.size(); ++i)
        {
            int u = path[i], v = path[i + 1];
            // find cheapest according to cost_of among parallel (u,v)
            LL best = (1LL << 62);
            for (const auto &e : g[u])
                if (e.to == v)
                {
                    LL w = (LL)cost_of(e.w1, e.w2);
                    if (w < best)
                        best = w;
                }
            if (best == (1LL << 62))
                return best;
            c += best;
        }
        return c;
    };

    // candidate heap
    struct Cmp
    {
        bool operator()(const Cand &a, const Cand &b) const { return a.cost > b.cost; }
    };
    priority_queue<Cand, vector<Cand>, Cmp> heap;

    // For spur paths we temporarily remove edges/nodes along root
    for (int k = 1; k < K; ++k)
    {
        // check time
        auto now = clk::now();
        double elapsed = std::chrono::duration<double>(now - t0).count();
        if (elapsed > time_budget_sec)
            break;

        const vector<int> &prev = out_paths[k - 1];
        // For each spur index
        bool any = false;
        for (size_t i = 0; i + 1 < prev.size(); ++i)
        {
            int spur_node = prev[i];
            // root path prefix 0..i
            vector<int> root_prefix(prev.begin(), prev.begin() + i + 1);

            // Build a temporary graph where we ban edges that would recreate earlier paths with same prefix
            // Lightweight: we won't copy the whole graph; we handle bans in the relax loop by skipping them.

            // Collect banned edges: for every earlier path Pj with same prefix, ban edge (Pj[i], Pj[i+1])
            unordered_set<long long> banned_edge;
            banned_edge.reserve(16);
            auto key = [](int a, int b) -> long long
            { return ((long long)(unsigned int)a << 32) | (unsigned int)b; };
            for (const auto &Pj : out_paths)
            {
                if (Pj.size() > i && equal(root_prefix.begin(), root_prefix.end(), Pj.begin()))
                {
                    if (i + 1 < Pj.size())
                    {
                        banned_edge.insert(key(Pj[i], Pj[i + 1]));
                    }
                }
            }

            // Also ban nodes in root_prefix except spur_node (to enforce simplicity)
            vector<char> banned_node(N, 0);
            for (size_t z = 0; z < root_prefix.size() - 1; ++z)
                banned_node[root_prefix[z]] = 1;

            // Dijkstra from spur_node with bans
            const long long INF = (1LL << 62);
            vector<long long> dist(N, INF);
            vector<int> parent(N, -1);

            using P = pair<long long, int>;
            priority_queue<P, vector<P>, greater<P>> pq;
            dist[spur_node] = 0;
            pq.push({0, spur_node});
            while (!pq.empty())
            {
                auto [d, u] = pq.top();
                pq.pop();
                if (d != dist[u])
                    continue;
                for (const auto &e : g[u])
                {
                    int v = e.to;
                    if (banned_node[v])
                        continue;
                    if (banned_edge.count(key(u, v)))
                        continue;
                    long long w = (long long)cost_of(e.w1, e.w2);
                    long long nd = d + w;
                    if (nd < dist[v])
                    {
                        dist[v] = nd;
                        parent[v] = u;
                        pq.push({nd, v});
                    }
                }
            }

            if (dist[T] == INF)
                continue; // no spur path
            // build spur path
            vector<int> spur_path;
            for (int v = T; v != -1; v = parent[v])
                spur_path.push_back(v);
            reverse(spur_path.begin(), spur_path.end());
            if (spur_path.empty() || spur_path.front() != spur_node)
                continue;

            // total path = root_prefix (without spur_node duplicate at start of spur) + spur_path
            vector<int> total = root_prefix;
            total.pop_back();
            total.insert(total.end(), spur_path.begin(), spur_path.end());

            // Avoid duplicates
            // Compute metric cost now (for heap ordering)
            long long mc = path_cost_metric(total);
            if (mc == (1LL << 62))
                continue;

            heap.push(Cand{mc, move(total)});
            any = true;

            // time guard inside loop
            now = clk::now();
            elapsed = std::chrono::duration<double>(now - t0).count();
            if (elapsed > time_budget_sec)
                break;
        }
        if (!any)
            break;

        // Pop next best distinct path
        while (!heap.empty())
        {
            Cand c = heap.top();
            heap.pop();
            // Ensure distinct from those we already have
            bool dup = false;
            for (const auto &P : out_paths)
            {
                if (P == c.path)
                {
                    dup = true;
                    break;
                }
            }
            if (!dup)
            {
                out_paths.push_back(move(c.path));
                break;
            }
        }
        if ((int)out_paths.size() <= k)
            break; // heap exhausted
    }
}

int main()
{
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    auto t0 = clk::now();
    const double SOFT_BUDGET = 19.0; // try to stop before 20s

    int N, M;
    if (!(cin >> N >> M))
    {
        cout << 0 << "\n";
        return 0;
    }
    int S, T;
    cin >> S >> T;

    vector<vector<Edge>> g(N);
    g.reserve(N);
    for (int i = 0; i < M; ++i)
    {
        int u, v, w1, w2;
        cin >> u >> v >> w1 >> w2;
        g[u].push_back({v, w1, w2});
        g[v].push_back({u, w1, w2}); // if directed, comment out
    }

    vector<bool> is_prime = sieve_primes(max(10, N + 5));

    struct Cand
    {
        vector<int> path;
        long long cost;
    };
    vector<Cand> cands;
    cands.reserve(200);
    unordered_set<string> seen;
    seen.reserve(400);

    auto consider = [&](const vector<int> &path)
    {
        if (path.empty())
            return;
        string sig = path_signature(path);
        if (seen.insert(sig).second)
        {
            long long bc = blackie_cost_of_path(path, g, is_prime);
            if (bc != (1LL << 62))
                cands.push_back({path, bc});
        }
    };

    auto run_collect = [&](auto cf)
    {
        auto r = dijkstra(N, g, S, cf);
        consider(reconstruct_path(S, T, r.parent));
    };

    // Baselines
    run_collect([](int w1, int)
                { return (long long)w1; });
    run_collect([](int, int w2)
                { return 3LL * (long long)w2; });
    run_collect([](int w1, int w2)
                { return (long long)min(w1, 3 * w2); });

    // Fewest hops
    {
        auto parent_hops = [&](int N, const vector<vector<Edge>> &g, int s, int t)
        {
            vector<int> d(N, -1), par(N, -1);
            queue<int> q;
            d[s] = 0;
            q.push(s);
            while (!q.empty())
            {
                int u = q.front();
                q.pop();
                if (u == t)
                    break;
                for (const auto &e : g[u])
                {
                    int v = e.to;
                    if (d[v] == -1)
                    {
                        d[v] = d[u] + 1;
                        par[v] = u;
                        q.push(v);
                    }
                }
            }
            return par;
        }(N, g, S, T);
        consider(reconstruct_path(S, T, parent_hops));
    }

    // Degree-biased: prefer nodes with higher degree (promotes short paths)
    vector<int> deg(N);
    for (int u = 0; u < N; ++u)
        deg[u] = (int)g[u].size();
    run_collect([&](int w1, int w2)
                {
        // subtract small fraction of degree at head (tie-break inside dijkstra weight)
        // keep it integer-safe
        int bonus = 0; // we canￃﾢￂﾀￂﾙt read node degree here (edge-only), so keep base;
        (void)deg; (void)bonus;
        return (long long)min(w1, 3*w2); });

    // ￃﾎￂﾻ-grid blends (dense)
    for (int k = 0; k <= 20; ++k)
    {
        auto now = clk::now();
        if (std::chrono::duration<double>(now - t0).count() > SOFT_BUDGET)
            break;
        double lam = k / 20.0;
        run_collect([lam](int w1, int w2)
                    {
            long double wl = (1.0L - lam)*(long double)w1 + lam*(long double)(3LL*w2);
            long long w = (long long)llround(wl);
            if(w<=0) w=1; return w; });
    }

    // Length-aware blends using prime density up to L
    vector<int> Ls = {10, 15, 20, 30, 40, 60, 80, 120, max(2, N / 8), max(2, N / 6), max(2, N / 4), max(2, N / 3), max(2, N / 2), max(2, N - 1)};
    sort(Ls.begin(), Ls.end());
    Ls.erase(unique(Ls.begin(), Ls.end()), Ls.end());
    for (int L : Ls)
    {
        auto now = clk::now();
        if (std::chrono::duration<double>(now - t0).count() > SOFT_BUDGET)
            break;
        int pc = count_primes_up_to(L, is_prime);
        double lam = (L > 0) ? (double)pc / (double)L : 0.0;
        run_collect([lam](int w1, int w2)
                    {
            long double wl = (1.0L - lam)*(long double)w1 + lam*(long double)(3LL*w2);
            long long w = (long long)llround(wl);
            if(w<=0) w=1; return w; });
    }

    // Randomized perturbations
    std::mt19937_64 rng(20251022ULL);
    std::uniform_int_distribution<int> jitter(-2, 3);
    for (int r = 0; r < 12; ++r)
    {
        auto now = clk::now();
        if (std::chrono::duration<double>(now - t0).count() > SOFT_BUDGET)
            break;
        double lam = (r % 4 == 0) ? 0.30 : (r % 4 == 1 ? 0.45 : (r % 4 == 2 ? 0.60 : 0.80));
        run_collect([&](int w1, int w2)
                    {
            long long base = (long long)llround((1.0L - lam)*(long double)w1 + lam*(long double)(3LL*w2));
            return base + jitter(rng); });
    }

    // Yen K-shortest (guarded: only if not huge and we still have time)
    bool smallish = (N <= 50000 && M <= 300000);
    if (smallish)
    {
        auto time_left = [&]()
        {
            return SOFT_BUDGET - std::chrono::duration<double>(clk::now() - t0).count();
        };
        if (time_left() > 3.0)
        {
            vector<vector<int>> ypaths;
            yens_k_shortest_simple_paths(N, g, S, T, [](int w1, int)
                                         { return (long long)w1; }, 12, t0, SOFT_BUDGET, ypaths);
            for (auto &p : ypaths)
                consider(p);
        }
        if (time_left() > 2.0)
        {
            vector<vector<int>> ypaths;
            yens_k_shortest_simple_paths(N, g, S, T, [](int, int w2)
                                         { return 3LL * (long long)w2; }, 10, t0, SOFT_BUDGET, ypaths);
            for (auto &p : ypaths)
                consider(p);
        }
        if (time_left() > 1.5)
        {
            vector<vector<int>> ypaths;
            yens_k_shortest_simple_paths(N, g, S, T, [](int w1, int w2)
                                         { return (long long)min(w1, 3 * w2); }, 10, t0, SOFT_BUDGET, ypaths);
            for (auto &p : ypaths)
                consider(p);
        }
    }

    if (cands.empty())
    {
        cout << 0 << "\n";
        return 0;
    }
    auto best = min_element(cands.begin(), cands.end(),
                            [](const Cand &a, const Cand &b)
                            { return a.cost < b.cost; });

    const vector<int> &ans = best->path;
    cout << ans.size() << "\n";
    for (size_t i = 0; i < ans.size(); ++i)
    {
        if (i)
            cout << ' ';
        cout << ans[i];
    }
    cout << "\n";
    return 0;
}