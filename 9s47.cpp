#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <algorithm>
#include <climits>
using namespace std;

struct PairHash {
    size_t operator()(const pair<int, int>& p) const {
        return ((size_t)p.first << 20) ^ p.second;
    }
};

void dijkstra(vector<vector<tuple<int, int>>> &graph, vector<long long> &dist, vector<pair<int, int>> &parent,
              int source, int dest) {
    priority_queue<pair<long long, int>, vector<pair<long long, int>>, greater<>> pq;

    pq.push({0, source});
    dist[source] = 0;

    while (!pq.empty()) {
        auto [cost, node] = pq.top();
        pq.pop();

        if (cost > dist[node]) continue;

        for (const auto& [neighbor, weight] : graph[node]) {
            long long new_cost = cost + weight;
            if (new_cost < dist[neighbor]) {
                dist[neighbor] = new_cost;
                parent[neighbor] = {node, weight};
                pq.push({new_cost, neighbor});
            }
        }
    }
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);

    int N, M;
    cin >> N >> M;

    int source, dest;
    cin >> source >> dest;

    vector<vector<tuple<int, int>>> graph(N);

    for (int i = 0; i < M; i++) {
        int u, v, w1, w2;
        cin >> u >> v >> w1 >> w2;
        graph[u].emplace_back(v, w1);
        graph[v].emplace_back(u, w2);
    }

    vector<long long> best_dist(N, LLONG_MAX);
    vector<pair<int, int>> best_parent(N, {-1, -1});

    dijkstra(graph, best_dist, best_parent, source, dest);

    if (best_dist[dest] == LLONG_MAX) {
        cout << 0 << "\n";
        return 0;
    }

    vector<int> path;
    int curr = dest;
    while (curr != -1) {
        path.push_back(curr);
        curr = best_parent[curr].first;
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