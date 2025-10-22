#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <algorithm>
#include <climits>
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
    
    vector<bool> prime = sieve_primes(N);
    
    priority_queue<tuple<long long, int, int>, 
                   vector<tuple<long long, int, int>>,
                   greater<>> pq;
    
    unordered_map<pair<int, int>, long long, PairHash> dist;
    unordered_map<pair<int, int>, pair<int, int>, PairHash> parent;
    
    pq.push({0, source, 0});
    dist[{source, 0}] = 0;
    
    pair<int, int> best_state = {-1, -1};
    long long best_cost = LLONG_MAX;
    
    int max_depth = min(N - 1, max(1000, N / 10));
    
    vector<long long> node_best(N, LLONG_MAX);
    node_best[source] = 0;
    
    int states_explored = 0;
    const int MAX_STATES = 5000000;
    
    while (!pq.empty() && states_explored < MAX_STATES) {
        auto [cost, node, edge_count] = pq.top();
        pq.pop();
        states_explored++;
        
        pair<int, int> state = {node, edge_count};
        
        if (dist.count(state) && dist[state] < cost) continue;
        
        if (cost >= best_cost) continue;
        
        if (node == dest) {
            if (cost < best_cost) {
                best_cost = cost;
                best_state = state;
            }
            continue;
        }
        
        if (edge_count >= max_depth) continue;
        
        for (const auto& [neighbor, w1, w2] : graph[node]) {
            int new_edge_count = edge_count + 1;
            
            long long edge_cost = prime[new_edge_count] ? 3LL * w2 : (long long)w1;
            long long new_cost = cost + edge_cost;
            
            if (new_cost >= best_cost) continue;
            
            if (new_cost > node_best[neighbor] * 1.5) continue;
            
            pair<int, int> new_state = {neighbor, new_edge_count};
            
            if (!dist.count(new_state) || dist[new_state] > new_cost) {
                dist[new_state] = new_cost;
                parent[new_state] = state;
                pq.push({new_cost, neighbor, new_edge_count});
                
                if (new_cost < node_best[neighbor]) {
                    node_best[neighbor] = new_cost;
                }
            }
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
    
    return 0;
}