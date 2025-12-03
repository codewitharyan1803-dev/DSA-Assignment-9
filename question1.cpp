#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>

using namespace std;

struct Edge {
    int u, v, w;
};

struct DSU {
    vector<int> parent, rank;
    DSU(int n) {
        parent.resize(n);
        rank.assign(n, 0);
        for (int i = 0; i < n; i++) parent[i] = i;
    }
    int find(int x) {
        if (parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    }
    bool unite(int a, int b) {
        a = find(a);
        b = find(b);
        if (a == b) return false;
        if (rank[a] < rank[b]) swap(a, b);
        parent[b] = a;
        if (rank[a] == rank[b]) rank[a]++;
        return true;
    }
};

void BFS(const vector<vector<pair<int,int>>>& adj, int start) {
    int n = adj.size();
    vector<bool> visited(n, false);
    queue<int> q;
    visited[start] = true;
    q.push(start);

    cout << "BFS from " << start << ": ";
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        cout << u << " ";
        for (auto &p : adj[u]) {
            int v = p.first;
            if (!visited[v]) {
                visited[v] = true;
                q.push(v);
            }
        }
    }
    cout << endl;
}

void DFSUtil(const vector<vector<pair<int,int>>>& adj, int u, vector<bool>& visited) {
    visited[u] = true;
    cout << u << " ";
    for (auto &p : adj[u]) {
        int v = p.first;
        if (!visited[v]) DFSUtil(adj, v, visited);
    }
}

void DFS(const vector<vector<pair<int,int>>>& adj, int start) {
    int n = adj.size();
    vector<bool> visited(n, false);
    cout << "DFS from " << start << ": ";
    DFSUtil(adj, start, visited);
    cout << endl;
}

void KruskalMST(int n, vector<Edge>& edges) {
    sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
        return a.w < b.w;
    });

    DSU dsu(n);
    int mst_weight = 0;

    cout << "Kruskal MST edges (u v w):" << endl;
    for (auto &e : edges) {
        if (dsu.unite(e.u, e.v)) {
            cout << e.u << " " << e.v << " " << e.w << endl;
            mst_weight += e.w;
        }
    }
    cout << "Total weight of Kruskal MST: " << mst_weight << endl;
}

void PrimMST(const vector<vector<pair<int,int>>>& adj) {
    int n = adj.size();
    vector<int> key(n, numeric_limits<int>::max());
    vector<bool> inMST(n, false);
    vector<int> parent(n, -1);

    using pii = pair<int,int>;
    priority_queue<pii, vector<pii>, greater<pii>> pq;

    int start = 0;
    key[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (inMST[u]) continue;
        inMST[u] = true;

        for (auto &p : adj[u]) {
            int v = p.first;
            int w = p.second;
            if (!inMST[v] && w < key[v]) {
                key[v] = w;
                parent[v] = u;
                pq.push({key[v], v});
            }
        }
    }

    int mst_weight = 0;
    cout << "Prim MST edges (u v w):" << endl;
    for (int v = 1; v < n; v++) {
        if (parent[v] != -1) {
            int u = parent[v];
            int w = 0;
            for (auto &p : adj[u]) {
                if (p.first == v) {
                    w = p.second;
                    break;
                }
            }
            cout << parent[v] << " " << v << " " << w << endl;
            mst_weight += w;
        }
    }
    cout << "Total weight of Prim MST: " << mst_weight << endl;
}

void Dijkstra(const vector<vector<pair<int,int>>>& adj, int src) {
    int n = adj.size();
    using pii = pair<int,int>;
    vector<int> dist(n, numeric_limits<int>::max());
    dist[src] = 0;

    priority_queue<pii, vector<pii>, greater<pii>> pq;
    pq.push({0, src});

    while (!pq.empty()) {
        int u = pq.top().second;
        int d = pq.top().first;
        pq.pop();

        if (d > dist[u]) continue;

        for (auto &p : adj[u]) {
            int v = p.first;
            int w = p.second;
            if (dist[u] != numeric_limits<int>::max() && dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
    }

    cout << "Dijkstra shortest distances from " << src << ":" << endl;
    for (int i = 0; i < n; i++) {
        cout << "Vertex " << i << " : ";
        if (dist[i] == numeric_limits<int>::max())
            cout << "INF";
        else
            cout << dist[i];
        cout << endl;
    }
}

int main() {
    int n, m;
    cout << "Enter number of vertices and edges: ";
    cin >> n >> m;

    vector<vector<pair<int,int>>> adj(n);
    vector<Edge> edges;

    cout << "Enter edges (u v w) for undirected weighted graph (0-based vertices):" << endl;
    for (int i = 0; i < m; i++) {
        int u, v, w;
        cin >> u >> v >> w;
        adj[u].push_back({v, w});
        adj[v].push_back({u, w});
        edges.push_back({u, v, w});
    }

    int start = 0;

    BFS(adj, start);
    DFS(adj, start);
    KruskalMST(n, edges);
    PrimMST(adj);
    Dijkstra(adj, start);

    return 0;
}
