import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

public class MaxCapacityDijkstra {
    public static boolean dijkstra_MaxCapacity(int[][] graph, int sourceNode, int sinkNode, int[] parent) {
        int V = graph.length;
        int[] dist = new int[V];
        Arrays.fill(dist, Integer.MIN_VALUE);
        dist[sourceNode] = Integer.MAX_VALUE; // Initialize source with maximum capacity

        PriorityQueue<Vertex> priorityQueue = new PriorityQueue<>(Comparator.comparingInt(node -> node.capacity));
        priorityQueue.add(new Vertex(sourceNode, Integer.MAX_VALUE));

        boolean pathExists = false;

        while (!priorityQueue.isEmpty()) {
            int u = priorityQueue.poll().vertex;

            if (u == sinkNode) {
                // Path from source to sink exists
                pathExists = true;
                break;
            }

            for (int v = 0; v < V; v++) {
                if (graph[u][v] > 0) {
                    int edgeCapacity = graph[u][v];

                    int minCapacity = Math.min(dist[u], edgeCapacity);

                    if (minCapacity > dist[v]) {
                        dist[v] = minCapacity;
                        priorityQueue.add(new Vertex(v, dist[v]));
                        parent[v] = u; // Update the parent of vertex v
                    }
                }
            }
        }


        return pathExists;
    }


    private static class Vertex {
        private int vertex;
        private int capacity;

        public Vertex(int vertex, int capacity) {
            this.vertex = vertex;
            this.capacity = capacity;
        }
    }
}
