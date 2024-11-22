// COMSC210 | Lab 34 | Winston Jose
// 

#include <iostream>
#include <vector>
#include <queue>
#include <stack>
using namespace std;

const int SIZE = 11;

struct Edge {
    int src, dest, weight;
};

typedef pair<int, int> Pair;  // Creates alias 'Pair' for the pair<int,int> data type

class Graph {
public:
    // a vector of vectors of Pairs to represent an adjacency list
    vector<vector<Pair>> adjList;

    // Graph Constructor
    Graph(vector<Edge> const &edges) {
        // resize the vector to hold SIZE elements of type vector<Edge>
        adjList.resize(SIZE);

        // add edges to the directed graph
        for (auto &edge: edges) {
            int src = edge.src;
            int dest = edge.dest;
            int weight = edge.weight;

            // insert at the end
            adjList[src].push_back(make_pair(dest, weight));
            // for an undirected graph, add an edge from dest to src also
            adjList[dest].push_back(make_pair(src, weight));
        }
    }

    // Print the graph's adjacency list
    void printGraph() {
        cout << "Graph's adjacency list:" << endl;
        for (int i = 0; i < adjList.size(); i++) {
            cout << i << " --> ";
            for (Pair v : adjList[i])
                cout << "(" << v.first << ", " << v.second << ") ";
            cout << endl;
        }
    }

    // DFS using recursion
    void DFS(int start) {
        vector<bool> visited(SIZE, false);
        cout << "DFS starting from node " << start << ": \n";
        DFSUtil(start, visited);
        cout << endl;
    }

    // Utility function for DFS
    void DFSUtil(int node, vector<bool>& visited) {
        visited[node] = true;
        cout << node << " ";

        // Recur for all adjacent nodes
        for (auto& neighbor : adjList[node]) {
            if (!visited[neighbor.first]) {
                DFSUtil(neighbor.first, visited);
            }
        }
    }

    // BFS using queue
    void BFS(int start) {
        vector<bool> visited(SIZE, false);
        queue<int> q;
        
        visited[start] = true;
        q.push(start);

        cout << "BFS starting from node " << start << ": \n";
        
        while (!q.empty()) {
            int node = q.front();
            q.pop();
            cout << node << " ";

            // Visit all adjacent nodes
            for (auto& neighbor : adjList[node]) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    q.push(neighbor.first);
                }
            }
        }
        cout << endl;
    }
};

int main() {
    // Creates a vector of graph edges/weights
  
    vector<Edge> edges = {
        {0, 1, 15}, {0, 2, 7}, {0, 4, 10}, 
        {1, 4, 12}, {1, 7, 9}, {2, 5, 8}, 
        {3, 5, 10}, {3, 6, 9}, {4, 5, 6}, 
        {4, 8, 14}, {5, 9, 13}, {6, 9, 7}, 
        {7, 8, 11}, {7, 10, 10}, {8, 9, 16}
    };

    // Creates graph
    Graph graph(edges);

    // Prints adjacency list representation of graph
    graph.printGraph();

    // Perform DFS and BFS from node 0
    graph.DFS(0);
    graph.BFS(0);

    return 0;
}
