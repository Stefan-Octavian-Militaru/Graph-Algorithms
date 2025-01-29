#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <utility>
#include <bits/ranges_algo.h>
#include <set>
using namespace std;
class vertice;
class graph;
class edge {
    vertice* start;
    vertice* end;
    int weight;
    int capacity;
public:
    edge(vertice* v1, vertice* v2) : start(v1), end(v2) {}
    edge(vertice* v1, vertice* v2, int w) : start(v1), end(v2), weight(w) {}
    edge(vertice* v1, vertice* v2, int w, int c) : start(v1), end(v2), weight(w), capacity(c) {}
    void set_capacity(int cap) {capacity = cap;}
    void set_weight(int wgt) {weight = wgt;}
    int get_capacity() {return capacity;}
    int get_weight() {return weight;}
    friend ostream& operator<<(ostream& out, const graph& g);
    friend class graph;
    friend class oriented_graph;
    friend class vertice;
    friend void euler(int vertice, vector<int>& cycle, graph& g);
};
class vertice {
protected:
    int id;
    vector<edge*> edges;
public:
    vertice(int id) : id(id){}
    void add_edge(edge* e) {edges.push_back(e);}
    int get_id() {return id;}
    friend ostream& operator<<(ostream& out, const graph& g);
    friend class graph;
    friend class oriented_graph;
    friend class edge;
    friend void euler(int vertice, vector<int>& cycle, graph& g);
};
class oriented_vertice : public vertice{
    vector<edge*> in_edges;
    vector<edge*> all_edges;
public:
    oriented_vertice(int id) : vertice(id) {}
    void add_in_edge(edge* e) {in_edges.push_back(e);}
    friend class graph;
    friend class oriented_graph;
    friend class edge;
};
class graph {
protected:
    int nr_vertices;
    int nr_edges;
    vector<vertice*> vertices;
    vector<edge*> edges;
    vector<vector<int>> adjacency;
public:
    graph() {vertices.push_back(new vertice(0));}
    graph(int nr_vertices, int nr_edges) : nr_vertices(nr_vertices), nr_edges(nr_edges) {vertices.push_back(new vertice(0));}
    virtual void add_vertice(vertice* v) {
        vertices.push_back(v);
        nr_vertices++;
    }
    virtual void add_edge(vertice* v1, vertice* v2) {
        edge* e = new edge(v1, v2);
        v1->add_edge(e);
        v2->add_edge(e);
        edges.push_back(e);
        nr_edges++;
    }
    virtual bool edge_exists(int start, int end);
    virtual void DFS(int start, vector<int>& visited);
    virtual void DFS_start(int start);
    virtual void BFS(int start);
    void DFMC(int start, vector<int>& visited, vector<int>& nivel, vector<int>& niv_min);
    void muchii_critice();
    virtual void read_weighted_graph();
    vector<edge*> kruskal();
    vector<int> hierzholer();
    void coloration(vector<pair<int, int>>& vertice_degrees, vector<int>& color);
    void six_coloration();
    friend istream& operator>>(istream& in, graph& g);
    friend ostream& operator<<(ostream& out, const graph& g);
    friend graph* havel_hakimi(vector<int>& degs);
    friend void euler(int vertice, vector<int>& cycle, graph& g);
    friend class vertice;
    friend class edge;
};
class oriented_graph : public graph {
    vector<oriented_vertice*> vertices;
    vector<vector<edge*> > edge_matrix;
public:
    oriented_graph() : graph() {vertices.push_back(new oriented_vertice(0));}
    oriented_graph(int nr_vertices, int nr_edges) : graph(nr_vertices, nr_edges) {vertices.push_back(new oriented_vertice(0));}
    void add_vertice(oriented_vertice* v) {
        vertices.push_back(v);
        nr_vertices++;
    }
    void add_edge(oriented_vertice* v1, oriented_vertice* v2) {
        edge* e = new edge(v1, v2);
        v1->add_edge(e);
        v2->add_in_edge(e);
        edges.push_back(e);
        nr_edges++;
    }
    bool edge_exists(int start, int end);
    void DFS(int start, vector<int>& visited);
    void DFS_start(int start);
    void BFS(int start);
    vector<int> topo_sort();
    void read_weighted_graph();
    void read_flow_graph();
    void djikstra(int s);
    void bellman_ford(int s);
    void acyclic_shortest_path(int s);
    void floyd_warshall();
    bool find_and_improve(int s, int t);
    int edmonds_karp(int s, int t);
    friend istream& operator>>(istream& in, oriented_graph& og);
};
bool graph :: edge_exists(int start, int end) {
    for (int i = 0; i < vertices[start]->edges.size(); i++) {
        if ((vertices[start]->edges[i]->end->get_id() == end) || (vertices[start]->edges[i]->start->get_id() == end)){
            return true;
        }
    }
    return false;
}
bool oriented_graph :: edge_exists(int start, int end) {
    for (int i = 0; i < vertices[start]->edges.size(); i++) {
        if (vertices[start]->edges[i]->end->get_id() == end) {
            return true;
        }
    }
    return false;
}
void graph :: DFS(int start, vector<int>& visited) {
    visited[start] = 1;
    cout << start << " ";
    for (int i = 0; i < vertices[start]->edges.size(); i++){
        if (visited[vertices[start]->edges[i]->start->get_id()] == 0) {
            DFS(vertices[start]->edges[i]->start->get_id(), visited);
        }
        else {
            if (visited[vertices[start]->edges[i]->end->get_id()] == 0) {
                DFS(vertices[start]->edges[i]->end->get_id(), visited);
            }
        }
    }
}
void oriented_graph :: DFS(int start, vector<int>& visited) {
    visited[start] = 1;
    cout << start << " ";
    for (int i = 0; i < vertices[start]->edges.size(); i++){
        if (visited[vertices[start]->edges[i]->end->get_id()] == 0) {
            DFS(vertices[start]->edges[i]->end->get_id(), visited);
        }
    }
}
void graph :: DFS_start(int start) {
    vector<int> visited;
    visited.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        visited[i] = 0;
    }
    visited[start] = 1;
    DFS(start, visited);
}
void oriented_graph :: DFS_start(int start) {
    vector<int> visited;
    visited.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        visited[i] = 0;
    }
    visited[start] = 1;
    DFS(start, visited);
}
void graph::BFS(int start) {
    queue<int> vqueue;
    vector<int> visited;
    visited.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        visited[i] = 0;
    }
    visited[start] = 1;
    vqueue.push(start);
    while (!vqueue.empty()) {
        int curr = vqueue.front();
        cout << curr << " ";
        for (int i = 0; i < vertices[curr]->edges.size();i++) {
            if (visited[vertices[curr]->edges[i]->start->get_id()] == 0) {
                vqueue.push(vertices[curr]->edges[i]->start->get_id());
                visited[vertices[curr]->edges[i]->start->get_id()] = 1;
            }
            else {
                if (visited[vertices[curr]->edges[i]->end->get_id()] == 0) {
                    vqueue.push(vertices[curr]->edges[i]->end->get_id());
                    visited[vertices[curr]->edges[i]->end->get_id()] = 1;
                }
            }
        }
        vqueue.pop();
    }
}
void oriented_graph::BFS(int start) {
    queue<int> vqueue;
    vector<int> visited;
    visited.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        visited[i] = 0;
    }
    visited[start] = 1;
    vqueue.push(start);
    while (!vqueue.empty()) {
        int curr = vqueue.front();
        cout << curr << " ";
        for (int i = 0; i < vertices[curr]->edges.size();i++) {
            if (visited[vertices[curr]->edges[i]->end->get_id()] == 0) {
                vqueue.push(vertices[curr]->edges[i]->end->get_id());
                visited[vertices[curr]->edges[i]->end->get_id()] = 1;
            }
        }
        vqueue.pop();
    }
}
vector<int> oriented_graph :: topo_sort() {
    vector<int> unused_vertices;
    unused_vertices.resize(nr_vertices + 1);
    vector<int> sorted_vertices;
    queue<oriented_vertice*> vqueue;
    for (int i = 1; i <= nr_vertices;  i++) {
        unused_vertices[i] = vertices[i]->in_edges.size();
        if (unused_vertices[i] == 0) {
            vqueue.push(vertices[i]);
            unused_vertices[i] = -1;
        }
    }
    while (!vqueue.empty()) {
        oriented_vertice* curr = vqueue.front();
        sorted_vertices.push_back(curr->get_id());
        for (int i = 0; i < curr->edges.size(); i++) {
            unused_vertices[curr->edges[i]->end->get_id()]--;
        }
        for (int i = 1; i <= nr_vertices;  i++) {
            if (unused_vertices[i] == 0) {
                vqueue.push(vertices[i]);
                unused_vertices[i] = -1;
            }
        }
        vqueue.pop();
    }
    return sorted_vertices;
}
void graph :: DFMC(int start, vector<int>& visited, vector<int>& nivel, vector<int>& niv_min) {
    visited[start] = 1;
    niv_min[start] = nivel[start];
    for (int i = 0; i < vertices[start]->edges.size(); i++) {
        int current_vertice = vertices[start]->edges[i]->start->get_id() + vertices[start]->edges[i]->end->get_id() - start;
        if (visited[current_vertice] == 0) {
            nivel[current_vertice] = nivel[start] + 1;
            DFMC(current_vertice, visited, nivel, niv_min);
            niv_min[start] = min(niv_min[start], niv_min[current_vertice]);
            if (niv_min[current_vertice] > nivel[start]) {
                cout << start << " " << current_vertice << endl;
            }
        }
        else {
            if (nivel[current_vertice] < nivel[start] - 1) {
                niv_min[start] = min(niv_min[start], nivel[current_vertice]);
            }
        }
    }

}
void graph :: muchii_critice() {
    vector<int> visited;
    vector<int> niv_min;
    vector<int> nivel;
    visited.resize(nr_vertices + 1);
    niv_min.resize(nr_vertices + 1);
    nivel.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        visited[i] = 0;
        nivel[i] = 0;
    }
    visited[1] = 1;
    nivel[1] = 1;
    DFMC(1, visited, nivel, niv_min);
}
void graph::read_weighted_graph() {
    ifstream fin("graph_info.txt");
    int nrv, nre;
    fin >> nrv >> nre;
    for (int i = 1; i <= nrv; i++) {
        vertice* v = new vertice(i);
        add_vertice(v);
    }
    int st, en, we;
    for (int i = 0; i < nre; i++) {
        fin >> st >> en >> we;
        edge* e = new edge(vertices[st], vertices[en], we);
        vertices[st]->add_edge(e);
        vertices[en]->add_edge(e);
        edges.push_back(e);
        nr_edges++;
    }
}
void oriented_graph::read_weighted_graph() {
    ifstream fin("graph_info.txt");
    int nrv, nre;
    fin >> nrv >> nre;
    for (int i = 1; i <= nrv; i++) {
        oriented_vertice* v = new oriented_vertice(i);
        add_vertice(v);
    }
    int st, en, we;
    for (int i = 0; i < nre; i++) {
        fin >> st >> en >> we;
        edge* e = new edge(vertices[st], vertices[en], we);
        vertices[st]->add_edge(e);
        vertices[en]->add_in_edge(e);
        edges.push_back(e);
        nr_edges++;
    }
}
void oriented_graph::read_flow_graph() {
    ifstream fin("graph_info.txt");
    int nrv, nre;
    fin >> nrv >> nre;
    edge_matrix.resize(nrv + 1);
    for (int i = 1; i <= nrv; i++) {
        edge_matrix[i].resize(nrv + 1);
        oriented_vertice* v = new oriented_vertice(i);
        add_vertice(v);
    }
    for (int i = 1; i <= nrv; i++) {
        for (int j = 1; j <= nrv; j++) {
            edge_matrix[i][j] = nullptr;
        }
    }
    int st, en, cap;
    for (int i = 0; i < nre; i++) {
        fin >> st >> en >> cap;
        edge* e = new edge(vertices[st], vertices[en], 0, cap);
        vertices[st]->add_edge(e);
        vertices[en]->add_in_edge(e);
        edges.push_back(e);
        edge_matrix[st][en] = e;
        nr_edges++;
    }
}
bool compare_weighted_edges(edge* e1, edge* e2) {
    return e1->get_weight() < e2->get_weight();
}
void init_kruskal(int v, vector<int>& parent, vector<int>& height) {
    parent[v] = 0;
    height[v] = 0;
}
int reprez_kruskal(int v, vector<int>& parent, vector<int>& height) {
    if (parent[v] == 0) {
        return v;
    }
    parent[v] = reprez_kruskal(parent[v], parent, height);
    return parent[v];
}
void union_kruskal(int v1, int v2, vector<int>& parent, vector<int>& height) {
    int rv1, rv2;
    rv1 = reprez_kruskal(v1, parent, height);
    rv2 = reprez_kruskal(v2, parent, height);
    if (height[rv1] > height[rv2]) {
        parent[rv2] = rv1;
    }
    else {
        parent[rv1] = rv2;
        if (height[rv1] == height[rv2]) {
            height[rv2] += 1;
        }
    }
}
vector<edge*> graph::kruskal() {
    vector<edge*> sorted_edges = edges;
    vector<edge*> apcm;
    sort(sorted_edges.begin(), sorted_edges.end(), compare_weighted_edges);
    vector<int> parent;
    vector<int> height;
    parent.resize(nr_vertices + 1);
    height.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        init_kruskal(i, parent, height);
    }
    for (int i = 0; i < sorted_edges.size(); i++) {
        if (reprez_kruskal(sorted_edges[i]->start->get_id(), parent, height) != reprez_kruskal(sorted_edges[i]->end->get_id(), parent, height)) {
            union_kruskal(sorted_edges[i]->start->get_id(), sorted_edges[i]->end->get_id(), parent, height);
            apcm.push_back(sorted_edges[i]);
        }
    }
    return apcm;
}
void queue_repair(priority_queue<pair<int,int>, vector<pair<int, int>>, greater<pair<int,int>>>& pq, int vertice, int dist) {
    pair<int, int> p = pq.top();
    if (p.second == vertice) {
        pq.pop();
        pq.push(make_pair(dist, p.second));
    }
    else {
        pq.pop();
        queue_repair(pq, vertice, dist);
        pq.push(p);
    }
}
void oriented_graph :: djikstra (int s) {
    vector<int> parent;
    vector<int> distance;
    parent.resize(nr_vertices + 1);
    distance.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        parent[i] = 0;
        distance[i] = INT_MAX;
    }
    distance[s] = 0;
    priority_queue<pair<int,int>, vector<pair<int, int>>,  greater<pair<int,int>>> vertice_queue;
    for (int i = 1; i <= nr_vertices; i++) {
        vertice_queue.push(make_pair(distance[i], i));
    }
    while (vertice_queue.empty() == false) {
        int curent = vertice_queue.top().second;
        if (distance[curent] < INT_MAX) {
            for (int contor = 0; contor < vertices[curent]->edges.size(); contor++){
                if (distance[curent] + vertices[curent]->edges[contor]->weight < distance[vertices[curent]->edges[contor]->end->get_id()] ){
                    distance[vertices[curent]->edges[contor]->end->get_id()] = distance[curent] + vertices[curent]->edges[contor]->weight;
                    queue_repair(vertice_queue, vertices[curent]->edges[contor]->end->get_id(), distance[vertices[curent]->edges[contor]->end->get_id()]);
                    parent[vertices[curent]->edges[contor]->end->get_id()] = curent;
                }
            }
        }
        vertice_queue.pop();
    }
    for (int i = 1; i <= nr_vertices; i++) {
        cout << i << " " << distance[i] << endl;
    }
}
void oriented_graph :: bellman_ford (int s) {
    vector<int> distance;
    vector<int> parent;
    distance.resize(nr_vertices + 1);
    parent.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        distance[i] = INT_MAX;
        parent[i] = 0;
    }
    distance[s] = 0;
    bool update_this_cycle;
    for (int i = 1; i < nr_vertices; i++){
        update_this_cycle = false;
        for (int j = 1; j < nr_vertices; j++) {
            if (distance[j] < INT_MAX) {
                for (int contor = 0; contor < vertices[j]->edges.size(); contor++){
                    int endnode = vertices[j]->edges[contor]->end->get_id();
                    if (distance[j] + vertices[j]->edges[contor]->weight < distance[endnode]){
                        distance[endnode] = distance[j] + vertices[j]->edges[contor]->weight;
                        parent[endnode] = j;
                        update_this_cycle = true;
                    }
                }
            }
        }
        if (update_this_cycle == false) {
            break;
        }
    }
    for (int i = 0; i < edges.size(); i++) {
        int stnode = edges[i]->start->get_id();
        int endnode = edges[i]->end->get_id();
        if (distance[stnode] + edges[i]->weight < distance[endnode]) {
            parent[endnode] = stnode;
            vector<bool> visited;
            visited.resize(nr_vertices + 1);
            for (int j = 1; j < nr_vertices; j++) {
                visited[j] = false;
            }
            visited[endnode] = true;
            while (visited[stnode] == false) {
                visited[stnode] = true;
                stnode = parent[stnode];
            }
            vector<int> negcycle;
            negcycle.push_back(stnode);
            endnode = parent[stnode];
            while (endnode != stnode) {
                negcycle.push_back(endnode);
                endnode = parent[endnode];
            }
            cout << "Negative cycle contains " << negcycle.size() << " nodes" << endl;
            for (int i = 0; i < negcycle.size(); i++) {
                cout << negcycle[i] << " ";
            }
            cout << endl;
            break;
        }
    }
    for (int i = 1; i <= nr_vertices; i++) {
        cout << i << " " << distance[i] << endl;
    }
}
void oriented_graph :: acyclic_shortest_path(int s) {
    vector<int> distance;
    distance.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        distance[i] = INT_MAX;
    }
    distance[s] = 0;
    vector<int> verts = topo_sort();
    for (int i = 0; i < verts.size(); i++) {
        int curent = verts[i];
        int curent_distance = distance[verts[i]];
        if (curent_distance < INT_MAX) {
            for (int contor = 0; contor < vertices[curent]->edges.size(); contor++){
                if (curent_distance + vertices[curent]->edges[contor]->weight < distance[vertices[curent]->edges[contor]->end->get_id()] ){
                    distance[vertices[curent]->edges[contor]->end->get_id()] = curent_distance + vertices[curent]->edges[contor]->weight;
                }
            }
        }
    }
    for (int i = 1; i <= nr_vertices; i++) {
        cout << i << " " << distance[i] << endl;
    }
}
void floyd_warshall_path(int i, int j, vector<vector<int>> &parent) {
    if (i != j) {
        floyd_warshall_path(i, parent[i][j], parent);
    }
    cout << j << " ";
}
void oriented_graph::floyd_warshall() {
    vector<vector<int>> distance;
    vector<vector<int>> parent;
    distance.resize(nr_vertices + 1);
    parent.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        distance[i].resize(nr_vertices + 1);
        parent[i].resize(nr_vertices + 1);
        int edge_contor = 0;
        for (int j = 1; j <= nr_vertices; j++) {
            if (vertices[i]->edges[edge_contor]->end->get_id() == j) {
                distance[i][j] = vertices[i]->edges[edge_contor]->weight;
                parent[i][j] = i;
                if (edge_contor == vertices[i]->edges.size() - 1) {
                    edge_contor = 0;
                }
                else {
                    edge_contor++;
                }

            }
            else {
                distance[i][j] = INT_MAX;
                parent[i][j] = 0;
            }
        }
    }
    for (int k=1; k<=nr_vertices; k++) {
        for (int i = 1; i <= nr_vertices; i++) {
            for (int j = 1; j <= nr_vertices; j++) {
                if (distance[i][j] > distance[i][k] + distance[k][j] &&(distance[i][k] < INT_MAX && distance[k][j] < INT_MAX)) {
                    distance[i][j] = distance[i][k] + distance[k][j];
                    parent[i][j] = parent[k][j];
                }
            }
        }
    }
    floyd_warshall_path(4, 3, parent);
}
bool oriented_graph :: find_and_improve(int s, int t) {
    queue<int> verts;
    vector<pair<int,int>> parent;
    vector<int> visited;
    parent.resize(nr_vertices + 1);
    visited.resize(nr_vertices + 1);
    verts.push(s);
    parent[s] = make_pair(0,0);
    bool found_finish = false;
    while (!verts.empty()) {
        int curr = verts.front();
        visited[curr] = 1;
        for (int i = 1; i <= nr_vertices; i++) {
            if (visited[i] != 1) {
                if (edge_matrix[curr][i] != 0 && edge_matrix[curr][i]->weight < edge_matrix[curr][i]->capacity){
                    verts.push(i);
                    parent[i] = make_pair(curr, edge_matrix[curr][i]->capacity - edge_matrix[curr][i]->weight);
                    visited[i] = 1;
                    if (i == t) {
                        found_finish = true;
                        break;
                    }
                }
                if (edge_matrix[i][curr] != 0 && edge_matrix[i][curr]->weight > 0) {
                    verts.push(i);
                    parent[i] = make_pair(-1 * curr,edge_matrix[i][curr]->weight);
                    visited[i] = 1;
                }
            }
        }
        if (found_finish == true)
            break;
        verts.pop();
    }
    if (found_finish == false) {
        return false;
    }
    int improve_amount = INT_MAX;
    int prev = parent[t].first;
    int curr_flow = parent[t].second;
    int curr = t;
    while (prev != 0) {
        if (prev < 0) {
            prev = prev * -1;
        }
        improve_amount = min(improve_amount, curr_flow);
        curr = prev;
        prev = parent[curr].first;
        curr_flow = parent[curr].second;
    }
    prev = parent[t].first;
    curr = t;
    while (prev != 0) {
        if (prev < 0) {
            edge_matrix[curr][prev]->weight -= improve_amount;
            prev = prev * -1;
        }
        else {
            edge_matrix[prev][curr]->weight += improve_amount;
        }
        curr = prev;
        prev = parent[prev].first;
    }
    return true;
}
int oriented_graph :: edmonds_karp(int s, int t) {
    bool val = find_and_improve(s, t);
    while (val == true) {
        val = find_and_improve(s,t);
    }
    int total_flow = 0;
    for (int i = 1; i <= nr_vertices; i++) {
        if (edge_matrix[s][i] != 0)
            total_flow += edge_matrix[s][i]->weight;
    }
    return total_flow;
}
void euler(int vertice, vector<int>& cycle, graph& g) {
    for (int i = 1; i <= g.nr_vertices ; i++) {
        if (g.adjacency[vertice][i] != 0) {
            g.adjacency[vertice][i] = 0;
            g.adjacency[i][vertice] = 0;
            euler(i, cycle, g);
        }
    }
    cycle.push_back(vertice);
}
vector<int> graph::hierzholer() {
    vector<int> cycle;
    graph copy = *this;
    euler(1, cycle, copy);
    for (int i = 0; i < cycle.size(); i++) {
        cout << cycle[i] << " ";
    }
    return cycle;
}
void graph :: coloration(vector<pair<int, int>>& vertice_degrees, vector<int>& color) {
    if (vertice_degrees.size() <= 6) {
        for (int i = 0; i < vertice_degrees.size(); i++) {
            color[vertice_degrees[i].first] = i + 1;
        }
    }
    else {
        int selected_vertice = -1;
        int offset = 0;
        for (int i = 0; i < vertice_degrees.size(); i++) {
            if (vertice_degrees[i].second <= 5) {
                selected_vertice = vertice_degrees[i].first;
                offset = i;
                break;
            }
        }
        for (int i = 0; i < vertice_degrees.size(); i++) {
            if (adjacency[selected_vertice][vertice_degrees[i].first] != 0) {
                vertice_degrees[i].second -= 1;
            }
        }
        vertice_degrees.erase(vertice_degrees.begin() + offset);
        coloration(vertice_degrees, color);
        vector<bool> available_colors = {true, true, true, true, true, true};
        for (int i = 0; i < vertices[selected_vertice]->edges.size(); i++) {
            if (vertices[selected_vertice]->edges[i]->start->get_id() != selected_vertice) {
                available_colors[color[vertices[selected_vertice]->edges[i]->start->get_id()] - 1] = false;
            }
            else {
                available_colors[color[vertices[selected_vertice]->edges[i]->end->get_id()] - 1] = false;
            }
        }
        for (int i = 0; i < 6; i++) {
            if (available_colors[i] == true) {
                color[selected_vertice] = i + 1;
            }
        }
    }
}
void graph::six_coloration() {
    vector<pair<int,int>> vertice_degrees;
    vector<int> color;
    color.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        vertice_degrees.push_back(make_pair(i, vertices[i]->edges.size()));
    }
    coloration(vertice_degrees, color);
    for (int i = 1; i <= nr_vertices; i++) {
        cout << i << " " << color[i] << endl;
    }
}
istream& operator>>(istream& in, graph& g) {
    int nr_edges, nr_vertices, start, end;
    in >> nr_vertices >> nr_edges;
    g = *(new graph(0,0));
    for (int i = 1; i <= nr_vertices; i++) {
        vertice* v = new vertice(i);
        g.add_vertice(v);
    }
    g.adjacency.resize(nr_vertices + 1);
    for (int i = 1; i <= nr_vertices; i++) {
        g.adjacency[i].resize(nr_vertices + 1);
        for (int j = 1; j <= nr_vertices; j++) {
            g.adjacency[i][j] = 0;
        }
    }
    for (int i = 0; i < nr_edges; i++) {
        in >> start >> end;
        g.adjacency[start][end] = 1;
        g.adjacency[end][start] = 1;
        g.add_edge(g.vertices[start], g.vertices[end]);
    }
    return in;
}
istream& operator>>(istream& in, oriented_graph& og) {
    int nr_edges, nr_vertices, start, end;
    in >> nr_vertices >> nr_edges;
    og = *(new oriented_graph(0,0));
    for (int i = 1; i <= nr_vertices; i++) {
        oriented_vertice* v = new oriented_vertice(i);
        og.add_vertice(v);
    }
    for (int i = 0; i < nr_edges; i++) {
        in >> start >> end;
        og.add_edge(og.vertices[start], og.vertices[end]);
    }
    return in;
}
ostream& operator<<(ostream& out, const graph& g) {
    out << g.nr_edges << " " << g.nr_vertices << endl;
    for (int i = 0; i < g.nr_edges; i++) {
        out << g.edges[i]->start->get_id() << " " << g.edges[i]->end->get_id() << endl;
    }
    return out;
}
bool comp_hh(pair<int,int> p1, pair<int, int> p2) {
    return p1.second > p2.second;
}
graph* havel_hakimi(vector<int>& degs) {
    vector<pair<int, int>> vertice_degree;
    for (int i = 0; i < degs.size(); i++) {
        vertice_degree.push_back(make_pair(i + 1, degs[i]));
    }
    sort(vertice_degree.begin(), vertice_degree.end(), comp_hh);
    for (int i = 0; i < vertice_degree.size(); i++) {
        cout << vertice_degree[i].first << " " << vertice_degree[i].second << endl;
    }
    int degsum = 0;
    int nr_vertices = degs.size();

    cout << nr_vertices << endl;
    graph* g = new graph(0, 0);
    for (int i = 0; i < nr_vertices; i++) {
        degsum += degs[i];
        if (degs[i] > nr_vertices - 1) {
            return nullptr;
        }
        g->add_vertice(new vertice(i + 1));
    }
    if (degsum % 2 != 0) {
        return nullptr;
    }
    for (int i = 0; i < nr_vertices; i++) {
        if (vertice_degree[i].second < 0) {
            return nullptr;
        }
        for (int j = i + 1; j <= i + vertice_degree[i].second; j++) {
            vertice_degree[j].second--;
            g->add_edge(g->vertices[vertice_degree[i].first], g->vertices[vertice_degree[j].first]);
            cout << "new edge from " << vertice_degree[i].first << " to " << vertice_degree[j].first<< endl;
        }
        vertice_degree[i].second = 0;
    }
    return g;
}
int main() {
    graph g;
    ifstream fin("graph_info.txt");
    fin >> g;
    g.six_coloration();
}