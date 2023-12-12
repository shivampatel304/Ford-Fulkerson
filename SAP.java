import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

public class SAP {
    public static boolean dijkstra_SAP(int[][] graph, int sourceNode, int sinkNode, int[] parent) {
        int[] dist = new int[graph.length];  // Use graph.length instead of V
        Arrays.fill(dist, Integer.MAX_VALUE);

        PriorityQueue<Vertex> queue = new PriorityQueue<>();
        queue.add(new Vertex(sourceNode, 0));
        dist[sourceNode] = 0;
        parent[sourceNode] = -1; // Set parent of source node to -1

        while (!queue.isEmpty()) {
            Vertex currentNode = queue.poll();
            int u = currentNode.node;

            if (u == sinkNode) {
                // Path from source to sink exists
                return true;
            }

            for (int v = 0; v < graph.length; v++) {  // Use graph.length instead of V
                if (graph[u][v] > 0) {
                    int newDistance = dist[u] + 1;

                    if (newDistance < dist[v]) {
                        dist[v] = newDistance;
                        queue.add(new Vertex(v, newDistance));
                        parent[v] = u; // Update the parent of vertex v
                    }
                }
            }
        }

        // No path from source to sink
        return false;
    }



}

class Vertex implements Comparable<Vertex> {
    int node;
    int distance;

    public Vertex(int node, int distance) {
        this.node = node;
        this.distance = distance;
    }

    @Override
    public int compareTo(Vertex other){
        return Integer.compare(this.distance,other.distance);
    }
}