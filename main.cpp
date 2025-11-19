#include <bits/stdc++.h>
using namespace std;
typedef pair<int, int> pii;
const int MAXN = 1e5 + 10, INF = 0x3f3f3f3f;
vector<pii> adj[MAXN];
int dis[MAXN], vis[MAXN];

int main() {
    ios_base::sync_with_stdio(0), cin.tie(0);
    int n, m;
    cin >> n >> m;
    while (m--) {
        int u, v;
        cin >> u >> v;
        adj[u].emplace_back(v, 1);
        adj[v].emplace_back(u, 1);
    }
    priority_queue<pii> pq;
    pq.push({0, n - 1});
    memset(dis, 0x3f, sizeof dis);
    vector<int> path;
    path.push_back(n - 1);
    dis[n - 1] = 0;
    while (!pq.empty()) {
        pii u = pq.top();
        pq.pop();
        if (vis[u.second]) continue;
        vis[u.second] = 1;
        for (auto& v : adj[u.second]) {
            if (!vis[v.first] && u.first + v.second < dis[v.first]) {
                dis[v.first] = u.first + v.second;
                pq.push({dis[v.first], v.first});
                path.push_back(v.first);
            }
        }
    }
    int k = 0, maxDis = 0;
    for (auto& node : path) {
        if (maxDis < dis[node]) {
            k = 1;
            maxDis = dis[node];
        } else if (maxDis == dis[node]) {
            ++k;
        }
    }
    cout << k << '\n';
    for (auto& node : path) {
        if (maxDis == dis[node]) cout << node << ' ';
    }
    return 0;
}