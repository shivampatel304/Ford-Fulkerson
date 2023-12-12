import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

public class RandomDFS {

    public static boolean dijkstra_RandomDFS(int[][] graph, int sourceNode, int sinkNode, int[] parent) {
        int V = graph.length;
        int[] dist = new int[V];
        Arrays.fill(dist, Integer.MAX_VALUE);

        PriorityQueue<Vertex> queue = new PriorityQueue<>();
        queue.add(new Vertex(sourceNode, 0));
        dist[sourceNode] = 0;

        boolean pathExists = false;

        while (!queue.isEmpty()) {
            Vertex currentNode = queue.poll();
            int u = currentNode.node;

            if (u == sinkNode) {
                // Path from source to sink exists
                pathExists = true;
                break;
            }

            for (int v = 0; v < V; v++) {
                if (graph[u][v] > 0 && dist[v] == Integer.MAX_VALUE) {
                    int distance = new Random().nextInt();
                    dist[v] = distance;
                    queue.add(new Vertex(v, distance));
                    parent[v] = u; // Update the parent of vertex v
                }
            }
        }

        return pathExists;
    }

}



