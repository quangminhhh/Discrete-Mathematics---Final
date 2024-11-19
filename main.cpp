    #include <iostream>
    #include <vector>
    #include <queue>
    #include <unordered_map>
    #include <cmath>
    #include <limits>
    #include <algorithm>
    #include "termcolor.hpp"

    using namespace std;

    struct Node
    {
        string name;
        double x;
        double y;

        Node(string n, double x_coord, double y_coord) : name(n), x(x_coord), y(y_coord) {}
    };

    struct Edge
    {
        int to;
        double distance;
        double cost;

        Edge(int t, double d, double c) : to(t), distance(d), cost(c) {}
    };


    struct Graph
    {
        vector<Node> nodes;
        unordered_map<int, vector<Edge> > adjList;

        void addNode(const string &name, double x, double y)
        {
            nodes.emplace_back(name, x, y);
        }

        void addEdge(int from, int to, double distance, double cost)
        {
            adjList[from].emplace_back(to, distance, cost);
            adjList[to].emplace_back(from, distance, cost);
        }
    };

    struct PriorityNode
    {
        int node;
        double f; // f = g + h

        bool operator<(const PriorityNode &other) const
        {
            return f > other.f;
        }
    };

    // Function to calculate Euclidean distance between two nodes
    double heuristic(const Node &a, const Node &b)
    {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }


    bool AStar(const Graph &graph, int start, int goal, double alpha, double beta,
               vector<int> &path, double &totalDistance, double &totalCost)
    {
        int n = graph.nodes.size();
        vector<double> g(n, numeric_limits<double>::infinity());
        vector<double> c(n, numeric_limits<double>::infinity());
        vector<int> cameFrom(n, -1);

        priority_queue<PriorityNode> openSet;
        g[start] = 0.0;
        c[start] = 0.0;
        double h = heuristic(graph.nodes[start], graph.nodes[goal]);
        openSet.push(PriorityNode(start, alpha * g[start] + beta * c[start] + h)); // Use parentheses

        while (!openSet.empty())
        {
            PriorityNode current = openSet.top();
            openSet.pop();

            if (current.node == goal)
            {
                path.clear();
                int node = goal;
                while (node != -1)
                {
                    path.push_back(node);
                    node = cameFrom[node];
                }
                reverse(path.begin(), path.end());

                totalDistance = g[goal];
                totalCost = c[goal];
                return true;
            }

            for (const auto &edge : graph.adjList.at(current.node))
            {
                int neighbor = edge.to;
                double tentative_g = g[current.node] + edge.distance;
                double tentative_c = c[current.node] + edge.cost;

                // Calculate f(n) = α * g(n) + β * c(n) + h(n)
                double tentative_f = alpha * tentative_g + beta * tentative_c +
                                     heuristic(graph.nodes[neighbor], graph.nodes[goal]);

                if (tentative_g < g[neighbor] || tentative_c < c[neighbor])
                {
                    cameFrom[neighbor] = current.node;
                    g[neighbor] = tentative_g;
                    c[neighbor] = tentative_c;
                    openSet.push(PriorityNode(neighbor, tentative_f)); // Use parentheses
                }
            }
        }

        return false;
    }

    int main()
    {
        cout << termcolor::bold << termcolor::blue;
        cout << "============================\n";
        cout << " BUS ROUTER PROGRAMMING\n";
        cout << "============================\n";
        cout << termcolor::reset;

        Graph graph;

        // Add nodes (name, x, y)
        graph.addNode("A", 0, 0);
        graph.addNode("B", 2, 3);
        graph.addNode("C", 5, 1);
        graph.addNode("D", 6, 4);
        graph.addNode("E", 8, 0);
        graph.addNode("F", 9, 3);
        graph.addNode("G", 11, 1);
        graph.addNode("H", 12, 4);
        graph.addNode("I", 14, 0);
        graph.addNode("J", 15, 3);

        graph.addEdge(0, 1, 3.6, 2.0); // A - B
        graph.addEdge(0, 2, 5.1, 3.0); // A - C
        graph.addEdge(1, 3, 4.1, 2.5); // B - D
        graph.addEdge(2, 3, 3.0, 2.2); // C - D
        graph.addEdge(2, 4, 3.5, 2.8); // C - E
        graph.addEdge(3, 5, 3.2, 2.4); // D - F
        graph.addEdge(4, 6, 3.0, 2.1); // E - G
        graph.addEdge(5, 7, 3.0, 2.3); // F - H
        graph.addEdge(6, 8, 3.5, 2.7); // G - I
        graph.addEdge(7, 9, 3.2, 2.5); // H - J
        graph.addEdge(8, 9, 3.3, 2.6); // I - J
        graph.addEdge(5, 6, 4.0, 3.0); // F - G

        cout << termcolor::yellow << "Current bus route structure:\n" << termcolor::reset;
        for (const auto &node : graph.nodes)
        {
            cout << termcolor::green << "Stop: " << termcolor::reset
                 << node.name << " (x: " << node.x << ", y: " << node.y << ")\n";
        }

        cout << termcolor::cyan << "\nConnections between stops:\n" << termcolor::reset;
        for (const auto &pair : graph.adjList)
        {
            int from = pair.first;
            for (const auto &edge : pair.second)
            {
                if (from < edge.to)
                {
                    cout << termcolor::blue << graph.nodes[from].name
                         << termcolor::reset << " <-> "
                         << termcolor::blue << graph.nodes[edge.to].name
                         << termcolor::reset << " (Distance: "
                         << edge.distance << " km, Cost: " << edge.cost << " USD)\n";
                }
            }
        }

        cout << termcolor::magenta << "\nPlease enter the starting point and destination:\n" << termcolor::reset;

        cout << termcolor::green << "Bus stops:\n" << termcolor::reset;
        for (size_t i = 0; i < graph.nodes.size(); ++i)
        {
            cout << i << ": " << graph.nodes[i].name << "\n";
        }

        // Get user input
        int start, goal;
        cout << termcolor::yellow << "Enter the number corresponding to the starting point: " << termcolor::reset;
        cin >> start;
        cout << termcolor::yellow << "Enter the number corresponding to the destination: " << termcolor::reset;
        cin >> goal;

        if (start < 0 || start >= graph.nodes.size() || goal < 0 || goal >= graph.nodes.size())
        {
            cout << termcolor::red << "Invalid starting point or destination.\n" << termcolor::reset;
            return 1;
        }

        double alpha = 1.0; // Weight for distance
        double beta = 1.0;  // Weight for cost

        vector<int> path;
        double totalDistance = 0.0;
        double totalCost = 0.0;

        bool found = AStar(graph, start, goal, alpha, beta, path, totalDistance, totalCost);

        if (found)
        {
            cout << termcolor::green << "\nOptimal path from " << graph.nodes[start].name
                 << " to " << graph.nodes[goal].name << ":\n" << termcolor::reset;
            for (size_t i = 0; i < path.size(); ++i)
            {
                cout << termcolor::blue << graph.nodes[path[i]].name << termcolor::reset;
                if (i != path.size() - 1)
                    cout << " -> ";
            }
            cout << "\nTotal distance: " << termcolor::yellow << totalDistance << " km" << termcolor::reset << "\n";
            cout << "Total cost: " << termcolor::yellow << totalCost << " USD" << termcolor::reset << "\n";
        }
        else
        {
            cout << termcolor::red << "No path found from " << graph.nodes[start].name
                 << " to " << graph.nodes[goal].name << ".\n" << termcolor::reset;
        }

        return 0;
    }