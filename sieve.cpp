#include <bits/stdc++.h>
using namespace std;

#pragma GCC optimize("Ofast")
#pragma GCC target("popcnt")

struct Edge {
    int to;
    int w1, w2;
};

inline int fast_read_int() {
    int x = 0, c = getchar_unlocked();
    while (c < '0' || c > '9')  
        c = getchar_unlocked();
    for (; c >= '0' && c <= '9'; c = getchar_unlocked())
        x = x * 10 + (c - '0');
    return x;
}


class PrimeChecker {
    bitset<500000> is_prime;
public:
    PrimeChecker(int m) {
        int n = (m - 1) / 2;
        is_prime.set();
        int sqrt_m = (int)sqrt((double)m);
        for (int i = 1; (2 * i + 1) <= sqrt_m; ++i) {
            if (is_prime[i]) {
                int p = 2 * i + 1;
                for (int j = ((p * p) - 1) / 2; j <= n; j += p)
                    is_prime[j] = 0;
            }
        }
    }

    inline bool isPrime(int n) const {
        if (n < 2) return false;
        if (n == 2) return true;
        if (n % 2 == 0) return false;
        return is_prime[(n - 1) / 2];
    }
};

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    
    int s,t,n,m;
    cin >> n >> m;
    cin >> s >> t;
    PrimeChecker prime(m);
    vector<vector<Edge>> adj(n + 1);
    for(int i = 0; i < m; i++){
        int u,v,w1,w2;
        u = fast_read_int();
        v = fast_read_int();
        w1 = fast_read_int();
        w2 = fast_read_int();
        adj[u].push_back({v, w1, w2});
        adj[v].push_back({u, w1, w2});
    }

    const int INF = INT_MAX / 4;
    const int MAX_STEPS = n;

    vector<vector<int>> dist(n + 1, vector<int>(MAX_STEPS + 2, INF));
    vector<vector<pair<int, int>>> parent(n + 1, vector<pair<int, int>>(MAX_STEPS + 2, {-1, -1}));

    using State = tuple<int, int, int>;
    priority_queue<State, vector<State>, greater<>> pq;

    dist[s][0] = 0;
    pq.push({0, s, 0});

    int end_step = -1;

    while (!pq.empty()) {
        auto [d, u, step] = pq.top();
        pq.pop();

        if (d != dist[u][step]) continue;
        if (u == t) { end_step = step; break; }

        if (step + 1 > MAX_STEPS) continue;

        for (auto &e : adj[u]) {
            int v = e.to;
            int nextStep = step + 1;
            int cost = prime.isPrime(nextStep) ? (1LL * e.w2 * 3) : e.w1;
            int nd = d + cost;

            if (nd < dist[v][nextStep]) {
                dist[v][nextStep] = nd;
                parent[v][nextStep] = {u, step};
                pq.push({nd, v, nextStep});
            }
        }
    }

    int ans = INF;
    int best_step = -1;
    for (int k = 0; k <= MAX_STEPS; ++k) {
        if (dist[t][k] < ans) {
            ans = dist[t][k];
            best_step = k;
        }
    }

    if (ans == INF) {
        cout << "-1\n";
        return 0;
    }

    vector<int> path;
    int cur = t, cur_step = best_step;
    while (cur != -1 && cur_step >= 0) {
        path.push_back(cur);
        auto [pnode, pstep] = parent[cur][cur_step];
        cur = pnode;
        cur_step = pstep;
    }

    reverse(path.begin(), path.end());

    cout << path.size() << "\n";
    for (int i = 0; i < (int)path.size(); i++) {
        cout << path[i] << (i + 1 == (int)path.size() ? '\n' : ' ');
    }

    return 0;
}
