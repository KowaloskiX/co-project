#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <algorithm>
#include <climits>
#include <bitset>
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

vector<long long> compute_heuristic(int dest, const vector<vector<tuple<int, int, int>>>& graph) {
    int N = graph.size();
    vector<long long> h(N, LLONG_MAX);
    priority_queue<pair<long long, int>, vector<pair<long long, int>>, greater<>> pq;

    pq.push({0, dest});
    h[dest] = 0;

    int visited_count = 0;
    int limit = max(1, N / 4);

    while (!pq.empty() && visited_count < limit) {
        auto [cost, node] = pq.top();
        pq.pop();

        if (cost > h[node]) continue;
        visited_count++;

        for (const auto& [nbr, w1, w2] : graph[node]) {
            long long new_cost = cost + w1;
            if (new_cost < h[nbr]) {
                h[nbr] = new_cost;
                pq.push({new_cost, nbr});
            }
        }
    }

    for (int i = 0; i < N; i++) {
        if (h[i] == LLONG_MAX)
            h[i] = LLONG_MAX / 4;
    }

    return h;
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);

    int N, M;
    cin >> N >> M;

    int source, dest;
    cin >> source >> dest;

    vector<vector<tuple<int, int, int>>> graph(N);

    for (int i = 0; i < M; i++) {
        int u, v, w1, w2;
        cin >> u >> v >> w1 >> w2;
        graph[u].emplace_back(v, w1, w2);
        graph[v].emplace_back(u, w1, w2);
    }

    int max_depth = min(N - 1, N);
    if (max_depth < 0) max_depth = 0;
    vector<bool> prime = sieve_primes(max_depth + 5);

    vector<long long> h = compute_heuristic(dest, graph);

    using State = tuple<long long, int, int>;
    priority_queue<State, vector<State>, greater<>> pq;

    unordered_map<pair<int, int>, long long, PairHash> dist;
    unordered_map<pair<int, int>, pair<int, int>, PairHash> parent;

    pq.push({h[source], source, 0});
    dist[{source, 0}] = 0;

    pair<int, int> best_state = {-1, -1};
    long long best_cost = LLONG_MAX;

    while (!pq.empty()) {
        auto [f_cost, node, edge_count] = pq.top();
        pq.pop();

        long long g_cost = dist[{node, edge_count}];
        if (g_cost >= best_cost) continue;

        if (node == dest) {
            best_cost = g_cost;
            best_state = {node, edge_count};
            continue;
        }

        for (const auto& [neighbor, w1, w2] : graph[node]) {
            int new_edge_count = edge_count + 1;

            long long edge_cost = (new_edge_count < (int)prime.size() && prime[new_edge_count])
                                    ? 3LL * w2
                                    : (long long)w1;

            long long new_g = g_cost + edge_cost;
            pair<int, int> new_state = {neighbor, new_edge_count};

            if (!dist.count(new_state) || dist[new_state] > new_g) {
                dist[new_state] = new_g;
                parent[new_state] = {node, edge_count};
                long long new_f = (h[neighbor] == LLONG_MAX / 4) ? new_g : new_g + h[neighbor];
                pq.push({new_f, neighbor, new_edge_count});
            }
        }
    }

    if (best_state.first == -1) {
        cout << 0 << "\n";
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

    return 0;
}
