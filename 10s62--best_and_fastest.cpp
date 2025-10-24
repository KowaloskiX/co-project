#include <bits/stdc++.h>
using namespace std;

vector<bool> sieve_primes(int n) {
    vector<bool> prime(n + 1, true);
    prime[0] = prime[1] = false;
    for (int i = 4; i <= n; i += 2) prime[i] = false;
    for (int i = 3; i * i <= n; i += 2) {
        if (prime[i]) {
            for (int j = i * i; j <= n; j += 2 * i) prime[j] = false;
        }
    }
    return prime;
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    int N, M;
    cin >> N >> M;
    int source, dest;
    cin >> source >> dest;

    vector<vector<tuple<int,int,int>>> graph(N);
    for (int i = 0; i < M; i++) {
        int u, v, w1, w2;
        cin >> u >> v >> w1 >> w2;
        graph[u].emplace_back(v, w1, w2);
        graph[v].emplace_back(u, w1, w2);
    }

    vector<long long> dist(N, LLONG_MAX);
    vector<int> parent(N, -1);
    priority_queue<pair<long long,int>, vector<pair<long long,int>>, greater<>> pq;

    dist[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        auto [cost, node] = pq.top(); pq.pop();
        if (cost > dist[node]) continue;
        for (auto &[neighbor, w1, w2] : graph[node]) {
            long long new_cost = cost + (long long)w1 + 3LL*w2 * 0.3;
            if (new_cost < dist[neighbor]) {
                dist[neighbor] = new_cost;
                parent[neighbor] = node;
                pq.push({new_cost, neighbor});
            }
        }
    }

    if (dist[dest] == LLONG_MAX) {
        cout << 0 << "\n";
        return 0;
    }

    vector<int> path;
    int curr = dest;
    while (curr != -1) {
        path.push_back(curr);
        curr = parent[curr];
    }
    reverse(path.begin(), path.end());

    int max_len = (int)path.size();
    vector<bool> prime = sieve_primes(max_len + 5);

    long long total_cost = 0;
    for (int i = 0; i + 1 < (int)path.size(); i++) {
        int u = path[i];
        int v = path[i+1];
        long long w1 = LLONG_MAX, w2 = LLONG_MAX;
        for (auto &[neighbor, ww1, ww2] : graph[u]) {
            if (neighbor == v) {
                w1 = ww1;
                w2 = ww2;
                break;
            }
        }
        int edge_pos = i + 1;
        total_cost += (edge_pos < (int)prime.size() && prime[edge_pos]) ? 3LL * w2 : w1;
    }

    cout << path.size() << "\n";
    for (size_t i = 0; i < path.size(); i++) {
        if (i > 0) cout << " ";
        cout << path[i];
    }
    cout << "\n";

    return 0;
}