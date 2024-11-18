#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <algorithm>

// Định nghĩa không gian tên để dễ quản lý
using namespace std;

// Cấu trúc Node đại diện cho điểm dừng xe buýt
struct Node
{
    string name;
    double x; // Tọa độ x (có thể sử dụng cho hàm heuristic)
    double y; // Tọa độ y

    Node(string n, double x_coord, double y_coord) : name(n), x(x_coord), y(y_coord) {}
};

// Cấu trúc Edge đại diện cho kết nối giữa hai node
struct Edge
{
    int to;          // Chỉ số của node đích trong danh sách nodes
    double distance; // Khoảng cách giữa các node
    double cost;     // Chi phí giữa các node

    Edge(int t, double d, double c) : to(t), distance(d), cost(c) {}
};

// Cấu trúc Graph quản lý danh sách kề
struct Graph
{
    vector<Node> nodes;
    unordered_map<int, vector<Edge> > adjList; // Thêm khoảng trắng giữa các dấu >

    void addNode(const string &name, double x, double y)
    {
        nodes.emplace_back(name, x, y);
    }

    void addEdge(int from, int to, double distance, double cost)
    {
        adjList[from].emplace_back(to, distance, cost);
        adjList[to].emplace_back(from, distance, cost); // Đồ thị vô hướng
    }
};

// Cấu trúc để lưu thông tin trong hàng đợi ưu tiên
struct PriorityNode
{
    int node;
    double f; // f = g + h

    bool operator<(const PriorityNode &other) const
    {
        return f > other.f; // Đảo ngược để hàng đợi ưu tiên lấy nhỏ nhất trước
    }
};

// Hàm tính khoảng cách Euclidean giữa hai node
double heuristic(const Node &a, const Node &b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Hàm A* để tìm đường đi tối ưu
bool AStar(const Graph &graph, int start, int goal, double alpha, double beta,
           vector<int> &path, double &totalDistance, double &totalCost)
{
    int n = graph.nodes.size();
    vector<double> g(n, numeric_limits<double>::infinity()); // Khoảng cách đã đi
    vector<double> c(n, numeric_limits<double>::infinity()); // Chi phí đã đi
    vector<int> cameFrom(n, -1);                             // Node cha

    priority_queue<PriorityNode> openSet;
    g[start] = 0.0;
    c[start] = 0.0;
    double h = heuristic(graph.nodes[start], graph.nodes[goal]);
    openSet.push(PriorityNode(start, alpha * g[start] + beta * c[start] + h)); // Sử dụng parentheses

    while (!openSet.empty())
    {
        PriorityNode current = openSet.top();
        openSet.pop();

        if (current.node == goal)
        {
            // Tái tạo đường đi
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

            // Tính f(n) = α * g(n) + β * c(n) + h(n)
            double tentative_f = alpha * tentative_g + beta * tentative_c +
                                 heuristic(graph.nodes[neighbor], graph.nodes[goal]);

            if (tentative_g < g[neighbor] || tentative_c < c[neighbor])
            {
                cameFrom[neighbor] = current.node;
                g[neighbor] = tentative_g;
                c[neighbor] = tentative_c;
                openSet.push(PriorityNode(neighbor, tentative_f)); // Sử dụng parentheses
            }
        }
    }

    // Không tìm thấy đường đi
    return false;
}

int main()
{
    // Khởi tạo đồ thị
    Graph graph;

    // Thêm các node (tên, x, y)
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

    // Thêm các cạnh (from, to, distance, cost)
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

    // In cấu trúc tuyến đường xe bus hiện tại
    cout << "Cấu trúc tuyến đường xe buýt hiện tại:\n";
    for (const auto &node : graph.nodes)
    {
        cout << "Điểm dừng: " << node.name << " (x: " << node.x << ", y: " << node.y << ")\n";
    }

    cout << "\nCác kết nối giữa các điểm dừng:\n";
    for (const auto &pair : graph.adjList)
    {
        int from = pair.first;
        for (const auto &edge : pair.second)
        {
            // Để tránh in cả hai hướng của mỗi kết nối (do đồ thị vô hướng)
            if (from < edge.to)
            {
                cout << graph.nodes[from].name << " <-> " << graph.nodes[edge.to].name
                     << " (Khoảng cách: " << edge.distance << " km, Chi phí: " << edge.cost << " USD)\n";
            }
        }
    }

    cout << "\nVui lòng nhập điểm xuất phát và điểm đích:\n";

    // Hiển thị danh sách các điểm dừng
    cout << "Các điểm dừng xe buýt:\n";
    for (size_t i = 0; i < graph.nodes.size(); ++i)
    {
        cout << i << ": " << graph.nodes[i].name << "\n";
    }

    // Nhận đầu vào từ người dùng
    int start, goal;
    cout << "Nhập số tương ứng với điểm xuất phát: ";
    cin >> start;
    cout << "Nhập số tương ứng với điểm đích: ";
    cin >> goal;

    // Kiểm tra tính hợp lệ của đầu vào
    if (start < 0 || start >= graph.nodes.size() || goal < 0 || goal >= graph.nodes.size())
    {
        cout << "Điểm xuất phát hoặc điểm đích không hợp lệ.\n";
        return 1;
    }

    // Trọng số cho khoảng cách và chi phí
    double alpha = 1.0; // Trọng số cho khoảng cách
    double beta = 1.0;  // Trọng số cho chi phí

    // Tìm đường đi tối ưu
    vector<int> path;
    double totalDistance = 0.0;
    double totalCost = 0.0;

    bool found = AStar(graph, start, goal, alpha, beta, path, totalDistance, totalCost);

    if (found)
    {
        cout << "\nĐường đi tối ưu từ " << graph.nodes[start].name << " đến " << graph.nodes[goal].name << ":\n";
        for (size_t i = 0; i < path.size(); ++i)
        {
            cout << graph.nodes[path[i]].name;
            if (i != path.size() - 1)
                cout << " -> ";
        }
        cout << "\nTổng khoảng cách: " << totalDistance << " km\n";
        cout << "Tổng chi phí: " << totalCost << " USD\n";
    }
    else
    {
        cout << "Không tìm thấy đường đi từ " << graph.nodes[start].name << " đến " << graph.nodes[goal].name << ".\n";
    }

    return 0;
}
