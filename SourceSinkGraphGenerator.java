import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.*;

public class SourceSinkGraphGenerator {

    public static Set<Edge> E;
    public int n,upperCap;
    public float r;

    public static void generateSourceSinkGraph(int n, double r, int upperCap, String filePath) {
        // Step 1: Define a set of vertices V
        Set<Integer> V = new HashSet<>();
        for (int i = 0; i < n; i++) {
            V.add(i);
        }

        // Assign random (x, y) coordinates to each vertex
        double[][] coordinates = new double[n][2];
        Random random = new Random();
        for (int u : V) {
            coordinates[u][0] = random.nextDouble();
            coordinates[u][1] = random.nextDouble();
        }

        // Step 2: Randomly assign edges of length ≤ r without creating parallel edges
        E = new HashSet<>();
        for (int u : V) {
            for (int v : V) {
                if (u != v && (Math.pow(coordinates[u][0] - coordinates[v][0], 2)
                        + Math.pow(coordinates[u][1] - coordinates[v][1], 2) <= Math.pow(r, 2))) {
                    double rand = random.nextDouble();
                    if (rand < 0.5) {
                        if (!E.contains(new Edge(u, v)) && !E.contains(new Edge(v, u))) {
                            E.add(new Edge(u, v));
                          //  System.out.println(u + " " + v);
                        }
                    } else {
                        if (!E.contains(new Edge(u, v)) && !E.contains(new Edge(v, u))) {
                            E.add(new Edge(v, u));
                           // System.out.println(u + " " + v);
                        }
                    }
                }
            }
        }

        // Step 3: Assign randomly selected integer-value capacity in the range [1...upperCap] for each edge
        int[][] capacities = new int[n][n];
        System.out.println(E.size());
        for (Edge edge : E) {
            capacities[edge.u][edge.v] = random.nextInt(upperCap) + 1;
        }


        // step 4: finding source and sink node

        int sourceNode = new Random().nextInt(n);

        int sinkNode = findLongestAcyclicPath(n, E, sourceNode);



        System.out.println("source Node: " + sourceNode +  " sink Node: " + sinkNode);


        // Store the graph and associated edge capacities in an ASCII-readable format
        try (FileWriter writer = new FileWriter(filePath)) {
            writer.write(n + "," + E.size() + "\n");
            for (Edge edge : E) {
               writer.write(edge.u + "," + coordinates[edge.u][0] + "," + coordinates[edge.u][1] + "," + edge.v + "," +
                       coordinates[edge.v][0] + "," + coordinates[edge.v][1] + "," + capacities[edge.u][edge.v] + "\n");
            }
            writer.write(sourceNode + "," + sinkNode + "\n");
            writer.write(n + "," + r + "," + upperCap + "\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }



    static class Edge {
        int u;
        int v;

        Edge(int u, int v) {
            this.u = u;
            this.v = v;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            Edge edge = (Edge) obj;
            return (u == edge.u && v == edge.v) || (u == edge.v && v == edge.u);
        }

        @Override
        public int hashCode() {
            return Integer.hashCode(u) + Integer.hashCode(v);
        }
    }

    public static void main(String[] args) {
        int n=200;
        float r= 0.3F;
        int upperCap=50;
        generateSourceSinkGraph(n, r, upperCap, "source_sink_graph.csv");

    }

    public static int findLongestAcyclicPath(int n, Set<Edge> E, int source) {
        List<Integer>[] adjacencyList = new List[n];
        for (int i = 0; i < n; i++) {
            adjacencyList[i] = new ArrayList<>();
        }

        for (Edge edge : E) {
            adjacencyList[edge.u].add(edge.v);
        }

        List<Integer> topologicalOrder = topologicalSort(adjacencyList);

        int[] distance = new int[n];
        Arrays.fill(distance, Integer.MIN_VALUE);
        distance[source] = 0;

        for (int u : topologicalOrder) {
            for (int v : adjacencyList[u]) {
                if (distance[v] < distance[u] + 1) {
                    distance[v] = distance[u] + 1;
                }
            }
        }

        // Find the node with the maximum distance (sink node)
        int sinkNode = source;
        for (int i = 0; i < n; i++) {
            if (distance[i] > distance[sinkNode]) {
                sinkNode = i;
            }
        }

        return sinkNode;
    }

    private static List<Integer> topologicalSort(List<Integer>[] adjacencyList) {
        int n = adjacencyList.length;
        boolean[] visited = new boolean[n];
        Stack<Integer> stack = new Stack<>();

        for (int i = 0; i < n; i++) {
            if (!visited[i]) {
                topologicalSortUtil(adjacencyList, i, visited, stack);
            }
        }

        List<Integer> result = new ArrayList<>();
        while (!stack.isEmpty()) {
            result.add(stack.pop());
        }

        return result;
    }

    private static void topologicalSortUtil(List<Integer>[] adjacencyList, int v, boolean[] visited, Stack<Integer> stack) {
        visited[v] = true;

        for (int neighbor : adjacencyList[v]) {
            if (!visited[neighbor]) {
                topologicalSortUtil(adjacencyList, neighbor, visited, stack);
            }
        }

        stack.push(v);
    }



}


