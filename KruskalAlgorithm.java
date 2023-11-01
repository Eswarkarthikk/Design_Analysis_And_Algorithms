import java.util.*;


class Edge<T> {
    private Vertex<T> vertex1;
    private Vertex<T> vertex2;
    private int weight;

    public Edge(Vertex<T> vertex1, Vertex<T> vertex2, int weight) {
        this.vertex1 = vertex1;
        this.vertex2 = vertex2;
        this.weight = weight;
    }

    public Vertex<T> getVertex1() {
        return vertex1;
    }

    public Vertex<T> getVertex2() {
        return vertex2;
    }

    public int getWeight() {
        return weight;
    }
}

class Vertex<T> {
    private T data;
    private List<Edge<T>> edges;

    public Vertex(T data) {
        this.data = data;
        this.edges = new ArrayList<>();
    }

    public T getData() {
        return data;
    }

    public List<Edge<T>> getEdges() {
        return edges;
    }

    public void addEdge(Edge<T> edge) {
        edges.add(edge);
    }
}

class BinaryMinHeap<T> {
    // Private inner class to hold the edges and their positions
    private class Node {
        Edge<T> edge;
        int position;
    }

    private Map<Edge<T>, Integer> nodePosition;
    private List<Node> heap;

    public BinaryMinHeap() {
        this.nodePosition = new HashMap<>();
        this.heap = new ArrayList<>();
    }

    public void add(Edge<T> edge) {
        Node node = new Node();
        node.edge = edge;
        nodePosition.put(edge, heap.size());
        heap.add(node);
        heapifyUp(heap.size() - 1);
    }

    public Edge<T> extractMin() {
        Node minNode = heap.get(0);
        Edge<T> minEdge = minNode.edge;
        Node lastNode = heap.remove(heap.size() - 1);

        if (!heap.isEmpty()) {
            heap.set(0, lastNode);
            nodePosition.put(lastNode.edge, 0);
            heapifyDown(0);
        }

        return minEdge;
    }

    public boolean isEmpty() {
        return heap.isEmpty();
    }

    private void heapifyUp(int currentIndex) {
        int parentIndex = (currentIndex - 1) / 2;
        while (currentIndex > 0 && heap.get(currentIndex).edge.getWeight() < heap.get(parentIndex).edge.getWeight()) {
            swap(currentIndex, parentIndex);
            currentIndex = parentIndex;
            parentIndex = (currentIndex - 1) / 2;
        }
    }

    private void heapifyDown(int currentIndex) {
        int leftIndex = 2 * currentIndex + 1;
        int rightIndex = 2 * currentIndex + 2;
        int smallest = currentIndex;

        if (leftIndex < heap.size() && heap.get(leftIndex).edge.getWeight() < heap.get(smallest).edge.getWeight()) {
            smallest = leftIndex;
        }
        if (rightIndex < heap.size() && heap.get(rightIndex).edge.getWeight() < heap.get(smallest).edge.getWeight()) {
            smallest = rightIndex;
        }

        if (smallest != currentIndex) {
            swap(currentIndex, smallest);
            heapifyDown(smallest);
        }
    }

    private void swap(int i, int j) {
        Node node1 = heap.get(i);
        Node node2 = heap.get(j);

        nodePosition.put(node1.edge, j);
        nodePosition.put(node2.edge, i);

        heap.set(i, node2);
        heap.set(j, node1);
    }
}

class DisjointSet<T> {
    private Map<Vertex<T>, Vertex<T>> parent;
    private Map<Vertex<T>, Integer> rank;

    public DisjointSet(List<Vertex<T>> vertices) {
        parent = new HashMap<>();
        rank = new HashMap<>();

        for (Vertex<T> vertex : vertices) {
            parent.put(vertex, vertex);
            rank.put(vertex, 0);
        }
    }

    public Vertex<T> find(Vertex<T> vertex) {
        if (vertex != parent.get(vertex)) {
            parent.put(vertex, find(parent.get(vertex)));
        }
        return parent.get(vertex);
    }

    public void union(Vertex<T> x, Vertex<T> y) {
        Vertex<T> rootX = find(x);
        Vertex<T> rootY = find(y);

        if (rootX == rootY) {
            return;
        }

        if (rank.get(rootX) < rank.get(rootY)) {
            parent.put(rootX, rootY);
        } else if (rank.get(rootX) > rank.get(rootY)) {
            parent.put(rootY, rootX);
        } else {
            parent.put(rootY, rootX);
            rank.put(rootX, rank.get(rootX) + 1);
        }
    }
}

class KruskalMinimumSpanningTree<T> {
    public List<Edge<T>> minimumSpanningTree(List<Vertex<T>> vertices, List<Edge<T>> edges) {
        BinaryMinHeap<T> minHeap = new BinaryMinHeap<>();

        for (Edge<T> edge : edges) {
            minHeap.add(edge);
        }

        List<Edge<T>> mst = new ArrayList<>();
        DisjointSet<T> disjointSet = new DisjointSet<>(vertices);

        while (!minHeap.isEmpty()) {
            Edge<T> edge = minHeap.extractMin();
            Vertex<T> root1 = disjointSet.find(edge.getVertex1());
            Vertex<T> root2 = disjointSet.find(edge.getVertex2());

            if (root1 != root2) {
                mst.add(edge);
                disjointSet.union(root1, root2);
            }
        }

        return mst;
    }
}

public class KruskalAlgorithm {
    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        List<Vertex<Character>> vertices = new ArrayList<>();
        List<Edge<Character>> edges = new ArrayList<>();

        System.out.print("Enter the number of vertices (a, b, c, ...): ");
        int numVertices = scanner.nextInt();

        System.out.println("Enter the vertices (character values):");
        for (int i = 0; i < numVertices; i++) {
            char vertexValue = scanner.next().charAt(0);
            Vertex<Character> vertex = new Vertex<>(vertexValue);
            vertices.add(vertex);
        }

        System.out.print("Enter the number of edges: ");
        int numEdges = scanner.nextInt();

        System.out.println("Enter the edges in the format 'vertex1 vertex2 weight':");
        for (int i = 0; i < numEdges; i++) {
            char vertex1 = scanner.next().charAt(0);
            char vertex2 = scanner.next().charAt(0);
            int weight = scanner.nextInt();

            Vertex<Character> v1 = null;
            Vertex<Character> v2 = null;
            for (Vertex<Character> v : vertices) {
                if (v.getData() == vertex1) {
                    v1 = v;
                }
                if (v.getData() == vertex2) {
                    v2 = v;
                }
            }

            if (v1 != null && v2 != null) {
                Edge<Character> edge = new Edge<>(v1, v2, weight);
                edges.add(edge);
            } else {
                System.out.println("Invalid edge vertices!");
            }
        }

        KruskalMinimumSpanningTree<Character> kruskal = new KruskalMinimumSpanningTree<>();
        List<Edge<Character>> minimumSpanningTree = kruskal.minimumSpanningTree(vertices, edges);

        System.out.println("Minimum Spanning Tree Edges:");
        for (Edge<Character> edge : minimumSpanningTree) {
            System.out.println("From " + edge.getVertex1().getData() + " To " + edge.getVertex2().getData() + " Weight: " + edge.getWeight());
        }
    }
}
