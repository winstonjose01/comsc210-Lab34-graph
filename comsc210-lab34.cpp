// COMSC210 | Lab 34 | Winston Jose
// IDE used: VS Code:
// Githublink : https://github.com/winstonjose01/comsc210-Lab34-graph

#include <iostream>
#include <vector>
#include <queue>
#include <stack>
using namespace std;

const int SIZE = 11;

// Struct to represent an edge in the graph
struct Edge {
    int src, dest, weight;  // source node, destination node, weight of the edge
};

typedef pair<int, int> Pair;  // Creates alias 'Pair' for the pair<int,int> data type

// Class to represent a graph using adjacency list
class Graph {
public:
    // a vector of vectors of Pairs to represent an adjacency list
    vector<vector<Pair>> adjList;

    // Graph Constructor - initializes the graph with edges
    Graph(vector<Edge> const &edges) {
        // Resize the adjacenty list to hold SIZE elements of type vector<Edge>
        adjList.resize(SIZE);

        // Add edges to the directed graph
        for (auto &edge: edges) {
            int src = edge.src;
            int dest = edge.dest;
            int weight = edge.weight;

            // Add edge from src to dest
            adjList[src].push_back(make_pair(dest, weight));
            // Add edge from dest to src
            adjList[dest].push_back(make_pair(src, weight));
        }
    }

    // Print the graph's adjacency list
    // Arguments: none
    // retursn : no returns
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

    // Computes the Minimum Spanning Tree (MST) using Prim's algorithm
    // Arguments: none
    // retursn : no returns
    void findMST(){

        priority_queue<Pair, vector<Pair>, greater<Pair>> pq; // Min-heap (weight, node)
        vector <bool> inMST(SIZE, false);   // Tracks if a node is in the MST
        vector <Edge> mstEdges;             // Stores edges in the MST
        int totalWeight = 0;                // Total weight of the MST

        pq.push({0,0}); // Start with node 0 (weight, node)

        while (!pq.empty()){
            int weight = pq.top().first;
            int node = pq.top().second;
            pq.pop();

            if (inMST[node]) continue; // Skip if already included
            inMST[node] = true;     // Mark node as included
            totalWeight += weight;  // Add edge weight to MST total

            // Add all valid edges from this node to MST
            for (auto &neighbor : adjList[node]){
                int nextNode = neighbor.first;
                int nextWeight = neighbor.second;
                if (!inMST[nextNode]) {
                    pq.push({nextWeight, nextNode});
                    mstEdges.push_back({node, nextNode, nextWeight});
                }
            }
        }
        // Output MST details
        cout << "Minimum Spanning Tree (MST):\n";
        for (auto &edge : mstEdges) {
            cout << "Edge: Stop " << edge.src << " -> Stop " << edge.dest 
                 << " (Time: " << edge.weight << " mins)\n";
        }
        cout << "Total Weight (Time): " << totalWeight << " mins\n";
}

    //  Finds shortest paths from a source node using Dijkstra's algorithm
    // Arguments: `start`: Starting node
    // retursn : no returns
    void shortestPathFromSource(int start){
        // Min-heap priority queue: (distance, node)
        priority_queue<Pair, vector<Pair>, greater<Pair>> pq;

        vector<int> dist(SIZE, INT_MAX);  // Initialize distances to infinity
        dist[start] = 0;                  // Distance to source is 0
        pq.push({0, start});              // Push the source node to the heap

        // Distance vector initialized to infinity
        while (!pq.empty()){
            int currentDist = pq.top().first;
            int currentNode = pq.top().second;
            pq.pop();

            // Skip if we encouter a stale distance
            if (currentDist > dist[currentNode]) continue;

            for (auto &neighbor : adjList[currentNode]){
                int nextNode = neighbor.first;
                int weight = neighbor.second;
                
                // Relaxation step: Update distances to neighbors
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

    // Performs Depth-First Search (DFS) traversal starting from a node
    // Arguments: `start`: Starting node
    // retursn : no returns
    void DFS(int start) {
        vector<bool> visited(SIZE, false);  // Tracks visited nodes
        cout << "DFS - Stops reachable from Stop " << start << ": \n";
        DFSUtil(start, visited);    // Start DFS from the starting node
        cout << endl;
    }

    // Helper function for DFS
    // Arguments: -`node`: Current node being visited, 
    //            -`visited`: Boolean vector to track visited nodes
    // retursn : no returns
    void DFSUtil(int node, vector<bool>& visited) {
        visited[node] = true;
        cout << node << " ";  // Print the current node

        // Recursively visit all unvisited nodes
        for (auto& neighbor : adjList[node]) {
            if (!visited[neighbor.first]) {
                DFSUtil(neighbor.first, visited);
            }
        }
    }

    // Performs Breadth-First Search (BFS) to find shortest path between two nodes
    // Arguments: -`node`: Current node being visited, 
    //            -`target`: Target node
    // retursn : no returns
    void BFS(int start, int target) {
        vector<bool> visited(SIZE, false);  // Tracks visited nodes
        queue<int> q;                       // Queue for BFS
        vector <int> parent (SIZE, -1);  // Stores parent for path reconstruction
        
        visited[start] = true;
        q.push(start);

        cout << "BFS starting from node " << start << ": \n";
        
        while (!q.empty()) {
            int node = q.front();
            q.pop();

            if (node == target) break; // Stop if target is reached

            // Visit all unvisited neighbors
            for (auto& neighbor : adjList[node]) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    parent[neighbor.first] = node;
                    q.push(neighbor.first);
                }
            }
        }
        // Reconstruct and print the shortest path
        cout << "BFS - Shortest path from Stop " << start << " to Stop " << target << ": ";
        if (!visited[target]) {
            cout << "No path found.\n";
            return;
        }
        stack<int> path;  // Stack to reconstruct path
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
        cout << "[1] Display City Tranportation Network\n";
        cout << "[2] Find Reachable Stops (DFS)\n";
        cout << "[3] Find Shortest Path Between Stops(BFS)\n";
        cout << "[4] Find Shortest Paths from Stop 0 (Dijkstra)\n";
        cout << "[5] Find Minimum Spanning Tree (MST)\n";
        cout << "[6] Exit\n";
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
                graph.findMST();
                break;
            case 6:
                cout << "Exiting application.\n";
                return 0;
            default:
                cout << "Invalid choice. Try again.\n";
        }
    }

    return 0;
}
