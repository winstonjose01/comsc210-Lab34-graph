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
        //cout << "Graph's adjacency list:" << endl;
        cout << "City Transportation Network (Bus Stops):\n";
        for (int i = 0; i < adjList.size(); i++) {
            cout << "Stop " << i << " --> ";
            for (Pair v : adjList[i])
                cout << "(Stop " << v.first << ", Time: " << v.second << " mins) ";
            cout << endl;
        }
    }

    void shortestPathFromSource(int start){
        // Min-heap priority queue: (distance, node)
        priority_queue<Pair, vector<Pair>, greater<Pair>> pq;

        vector<int> dist(SIZE, INT_MAX);
        dist[start] = 0;
        pq.push({0, start});

        // Distance vector initialized to infinity
        while (!pq.empty()){
            int currentDist = pq.top().first;
            int currentNode = pq.top().second;
            pq.pop();

            // skip if we encouter a stale distance
            if (currentDist > dist[currentNode]) continue;

            for (auto &neighbor : adjList[currentNode]){
                
                int nextNode = neighbor.first;
                int weight = neighbor.second;
                
                // Relaxation step
                if (dist[currentNode] + weight < dist[nextNode]) {
                    dist[nextNode] = dist[currentNode] + weight;
                    pq.push({dist[nextNode], nextNode});
                }
            }
        }

        // Print shortest distances
        cout << "Shortest paths from Stop " << start << ":\n";
        for (int i = 0; i < SIZE; i++) {
            if (dist[i] == INT_MAX)
                cout << "Stop " << i << ": Unreachable\n";
            else
                cout << "Stop " << i << ": " << dist[i] << " mins\n";
        }
    }

    // DFS using recursion
    void DFS(int start) {
        vector<bool> visited(SIZE, false);
        cout << "DFS - Stops reachable from Stop " << start << ": \n";
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
    void BFS(int start, int target) {
        vector<bool> visited(SIZE, false);
        queue<int> q;
        vector <int> parent (SIZE, -1); // Track paths
        
        visited[start] = true;
        q.push(start);

        cout << "BFS starting from node " << start << ": \n";
        
        while (!q.empty()) {
            int node = q.front();
            q.pop();
            //cout << node << " ";
            if (node == target) break;

            // Visit all adjacent nodes
            for (auto& neighbor : adjList[node]) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    parent[neighbor.first] = node;
                    q.push(neighbor.first);
                }
            }
        }
            // Reconstruct and print path
        cout << "BFS - Shortest path from Stop " << start << " to Stop " << target << ": ";
        if (!visited[target]) {
            cout << "No path found.\n";
            return;
        }
        stack<int> path;
        for (int v = target; v != -1; v = parent[v])
            path.push(v);
        while (!path.empty()) {
            cout << path.top();
            path.pop();
            if (!path.empty()) cout << " -> ";
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
    //graph.printGraph();

    //graph.DFS(0);
    //graph.BFS(0);
        int choice, start, target;
    while (true) {
        cout << "\nCity Transportation Network Menu:\n";
        cout << "1. View Network\n";
        cout << "2. Find Reachable Stops (DFS)\n";
        cout << "3. Find Shortest Path (BFS)\n";
        cout << "4. Find Shortest Paths from Stop 0 (Dijkstra)\n";
        cout << "5. Exit\n";
        cout << "Enter your choice: ";
        cin >> choice;

        switch (choice) {
            case 1:
                graph.printGraph();
                break;
            case 2:
                cout << "Enter starting stop (0-" << SIZE-1 << "): ";
                cin >> start;
                if (start >= 0 && start < SIZE)
                    graph.DFS(start);
                else
                    cout << "Invalid stop number.\n";
                break;
            case 3:
                cout << "Enter starting stop (0-" << SIZE-1 << "): ";
                cin >> start;
                cout << "Enter target stop (0-" << SIZE-1 << "): ";
                cin >> target;
                if (start >= 0 && start < SIZE && target >= 0 && target < SIZE)
                    graph.BFS(start, target);
                else
                    cout << "Invalid stop numbers.\n";
                break;
            case 4:
                graph.shortestPathFromSource(0);
                break;
            case 5:
                cout << "Exiting application.\n";
                return 0;
            default:
                cout << "Invalid choice. Try again.\n";
        }
    }

    return 0;
}
