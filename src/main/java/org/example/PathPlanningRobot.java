import java.awt.*;
import java.util.*;
import java.util.List;

public class PathPlanningRobot {

    public static void main(String[] args) {
        int[][] maze = {
                {0, 0, 0, 0, 0},
                {0, 1, 1, 0, 0},
                {0, 0, 0, 1, 0},
                {0, 1, 0, 1, 0},
                {0, 1, 0, 0, 0}
        };
        Point start = new Point(0, 0);
        Point end = new Point(4, 4);
        List<Point> path = findPath(maze, start, end);
        System.out.println(path);
        executePath(path);
    }

    public static List<Point> findPath(int[][] maze, Point start, Point end) {
        Set<Point> visited = new HashSet<>();
        Map<Point, Point> cameFrom = new HashMap<>();
        Map<Point, Double> gScore = new HashMap<>();
        Map<Point, Double> fScore = new HashMap<>();
        PriorityQueue<Point> frontier = new PriorityQueue<>((a, b) -> (int) (fScore.get(a) - fScore.get(b)));

        gScore.put(start, 0.0);
        fScore.put(start, heuristic(start, end));
        frontier.offer(start);

        while (!frontier.isEmpty()) {
            Point current = frontier.poll();
            if (current.equals(end)) {
                return reconstructPath(cameFrom, current);
            }
            visited.add(current);
            for (Point neighbor : getNeighbors(maze, current)) {
                if (visited.contains(neighbor)) {
                    continue;
                }
                double tentativeGScore = gScore.get(current) + 1;
                if (!frontier.contains(neighbor) || tentativeGScore < gScore.get(neighbor)) {
                    cameFrom.put(neighbor, current);
                    gScore.put(neighbor, tentativeGScore);
                    fScore.put(neighbor, tentativeGScore + heuristic(neighbor, end));
                    frontier.offer(neighbor);
                }
            }
        }
        return null;
    }

    public static double heuristic(Point a, Point b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    public static List<Point> getNeighbors(int[][] maze, Point p) {
        List<Point> neighbors = new ArrayList<>();
        int[][] directions = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
        for (int[] dir : directions) {
            int x = p.x + dir[0];
            int y = p.y + dir[1];
            if (x < 0 || x >= maze.length || y < 0 || y >= maze[0].length || maze[x][y] == 1) {
                continue;
            }
            neighbors.add(new Point(x, y));
        }
        return neighbors;
    }

    public static List<Point> reconstructPath(Map<Point, Point> cameFrom, Point current) {
        List<Point> path = new ArrayList<>();
        path.add(current);
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            path.add(0, current);
        }
        return path;
    }

    public static void executePath(List<Point> path) {
        for (int i = 0; i < path.size() - 1; i++) {
            Point current = path.get(i);
            Point next = path.get(i + 1);
            moveRobot(current, next);
        }
    }

    public static void moveRobot(Point current, Point next) {
        // Code to move robot
    }
}
