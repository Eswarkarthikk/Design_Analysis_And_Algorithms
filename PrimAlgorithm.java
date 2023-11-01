import java.util.Scanner;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.Map;
import java.util.Collections;
import java.util.HashSet;


class BinaryMinHeap<T> {
    private Map<T, Integer> nodePosition;
    private List<Node> heap;
    private int maxSize;

    public BinaryMinHeap(int maxSize) {
        this.nodePosition = new HashMap<>();
        this.heap = new ArrayList<>(maxSize);
        this.maxSize = maxSize;
    }

    public void add(int weight, T key) {
        if (nodePosition.containsKey(key)) {
            decrease(key, weight);
        } else {
            Node node = new Node();
            node.weight = weight;
            node.key = key;
            heap.add(node);
            nodePosition.put(key, heap.size() - 1);
            heapifyUp(heap.size() - 1);
        }
    }

    public void decrease(T key, int newWeight) {
        int position = nodePosition.get(key);
        Node node = heap.get(position);
        node.weight = newWeight;
        heapifyUp(position);
    }

    public boolean containsData(T key) {
        return nodePosition.containsKey(key);
    }

    public boolean empty() {
        return heap.isEmpty();
    }

    public Node extractMinNode() {
        Node minNode = heap.get(0);
        Node lastNode = heap.remove(heap.size() - 1);
        nodePosition.remove(minNode.key);
        if (!heap.isEmpty()) {
            heap.set(0, lastNode);
            nodePosition.put(lastNode.key, 0);
            heapifyDown(0);
        }
        return minNode;
    }

    public int getWeight(T key) {
        Integer position = nodePosition.get(key);
        if (position == null) {
            return Integer.MAX_VALUE;
        } else {
            return heap.get(position).weight;
        }
    }

    private void heapifyUp(int currentIndex) {
        int parentIndex = (currentIndex - 1) / 2;
        while (currentIndex > 0 && heap.get(currentIndex).weight < heap.get(parentIndex).weight) {
            swap(currentIndex, parentIndex);
            currentIndex = parentIndex;
            parentIndex = (currentIndex - 1) / 2;
        }
    }

    private void heapifyDown(int currentIndex) {
        int leftIndex = 2 * currentIndex + 1;
        int rightIndex = 2 * currentIndex + 2;
        int smallest = currentIndex;

        if (leftIndex < heap.size() && heap.get(leftIndex).weight < heap.get(smallest).weight) {
            smallest = leftIndex;
        }
        if (rightIndex < heap.size() && heap.get(rightIndex).weight < heap.get(smallest).weight) {
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

        nodePosition.put(node1.key, j);
        nodePosition.put(node2.key, i);

        heap.set(i, node2);
        heap.set(j, node1);
    }

    public class Node {
        int weight;
        T key;
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

    public void addBidirectionalEdge(Edge<T> edge) {
        edges.add(edge);
        Vertex<T> other = edge.getVertex1() == this ? edge.getVertex2() : edge.getVertex1();
        other.edges.add(edge);
    }

    @Override
    public String toString() {
        return data.toString();
    }
}

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

class Graph<T> {
    private List<Vertex<T>> vertices;

    public Graph() {
        vertices = new ArrayList<>();
    }

    public void addVertex(Vertex<T> vertex) {
        vertices.add(vertex);
    }

    public List<Vertex<T>> getAllVertex() {
        return vertices;
    }
}

class PrimMinimumSpanningTree<T> {
    private Map<Vertex<T>, Vertex<T>> parent;
    private Map<Vertex<T>, Integer> key;
    private Set<Vertex<T>> visitedVertices;
    private BinaryMinHeap<Vertex<T>> minHeap;

    public Map<Vertex<T>, List<Edge<T>>> minimumSpanningTree(Graph<T> graph, Vertex<T> startVertex) {
        this.parent = new HashMap<>();
        this.key = new HashMap<>();
        this.visitedVertices = new HashSet<>();
        this.minHeap = new BinaryMinHeap<>(graph.getAllVertex().size());

        Map<Vertex<T>, List<Edge<T>>> minimumSpanningTree = new HashMap<>();

        for (Vertex<T> vertex : graph.getAllVertex()) {
            key.put(vertex, Integer.MAX_VALUE);
            minHeap.add(Integer.MAX_VALUE, vertex);
        }

        minHeap.decrease(startVertex, 0);
        key.put(startVertex, 0);
        parent.put(startVertex, null);

        while (!minHeap.empty()) {
            BinaryMinHeap<Vertex<T>>.Node heapNode = minHeap.extractMinNode();
            Vertex<T> currentVertex = heapNode.key;
            visitedVertices.add(currentVertex);

            if (parent.get(currentVertex) != null) {
                Vertex<T> start = parent.get(currentVertex);
                Edge<T> minimumSpanningTreeEdge = findEdge(graph, start, currentVertex);
                minimumSpanningTree.computeIfAbsent(start, k -> new ArrayList<>()).add(minimumSpanningTreeEdge);
            }

            for (Edge<T> edge : currentVertex.getEdges()) {
                Vertex<T> adjacentVertex = getOtherVertex(edge, currentVertex);
                if (!visitedVertices.contains(adjacentVertex)) {
                    int weight = edge.getWeight();
                    if (key.get(adjacentVertex) > weight) {
                        minHeap.decrease(adjacentVertex, weight);
                        key.put(adjacentVertex, weight);
                        parent.put(adjacentVertex, currentVertex);
                    }
                }
            }
        }

        return minimumSpanningTree;
    }

    public Vertex<T> getOtherVertex(Edge<T> edge, Vertex<T> currentVertex) {
        return edge.getVertex1().equals(currentVertex) ? edge.getVertex2() : edge.getVertex1();
    }

    private Edge<T> findEdge(Graph<T> graph, Vertex<T> start, Vertex<T> end) {
        for (Edge<T> edge : start.getEdges()) {
            Vertex<T> adjacent = getOtherVertex(edge, start);
            if (adjacent.equals(end)) {
                return edge;
            }
        }
        return null;
    }
}

public class PrimAlgorithm {
    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        Graph<Character> graph = new Graph<>();
        Map<Character, Vertex<Character>> vertexMap = new HashMap<>();

        System.out.print("Enter the number of vertices: ");
        int numVertices = scanner.nextInt();

        System.out.println("Enter the vertices (characters a, b, c, ...):");
        for (int i = 0; i < numVertices; i++) {
            char vertexValue = scanner.next().charAt(0);
            Vertex<Character> vertex = new Vertex<>(vertexValue);
            graph.addVertex(vertex);
            vertexMap.put(vertexValue, vertex);
        }

        System.out.print("Enter the number of edges: ");
        int numEdges = scanner.nextInt();

        System.out.println("Enter the edges in the format 'vertex1 vertex2 weight':");
        for (int i = 0; i < numEdges; i++) {
            char vertex1 = scanner.next().charAt(0);
            char vertex2 = scanner.next().charAt(0);
            int weight = scanner.nextInt();

            Vertex<Character> v1 = vertexMap.get(vertex1);
            Vertex<Character> v2 = vertexMap.get(vertex2);

            Edge<Character> edge = new Edge<>(v1, v2, weight);
            v1.addEdge(edge);
            v2.addEdge(edge);
        }

        PrimMinimumSpanningTree<Character> prim = new PrimMinimumSpanningTree<>();

        System.out.print("Enter the source vertex: ");
        char sourceVertexValue = scanner.next().charAt(0);
        Vertex<Character> sourceVertex = vertexMap.get(sourceVertexValue);

        if (sourceVertex == null) {
            System.out.println("Invalid source vertex.");
            return;
        }

        Map<Vertex<Character>, List<Edge<Character>>> minimumSpanningTree = prim.minimumSpanningTree(graph, sourceVertex);

        System.out.println("Minimum Spanning Tree Edges from " + sourceVertexValue + ":");
        for (Map.Entry<Vertex<Character>, List<Edge<Character>>> entry : minimumSpanningTree.entrySet()) {
            Vertex<Character> startVertex = entry.getKey();
            System.out.println("From " + startVertex.getData() + ":");
            List<Edge<Character>> edges = entry.getValue();
            for (Edge<Character> edge : edges) {
                Vertex<Character> otherVertex = prim.getOtherVertex(edge, startVertex);
                System.out.println("  To " + otherVertex.getData() + " Weight: " + edge.getWeight());
            }
        }
    }
}

