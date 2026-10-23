# Intelligent Campus Navigation Agent (CSN304, P-1)



## How to run

```bash
python intelligent_campus_navigation_agent.py
```

The application uses only Python's standard library, so no `pip install` is required.

## What the project demonstrates

The campus is represented as an undirected weighted graph. Each location is a
state and travelling along a path is an action. Every path has distance, time,
accessibility, and crowd-level information.

- **BFS:** finds a route by minimum number of edges.
- **DFS:** explores one branch deeply before backtracking.
- **A\*:** uses `f(n) = g(n) + h(n)` to find a low-cost route using a heuristic.
- **Obstacles:** any checked path is removed from the graph before searching.
- **Advanced feature:** the user can prefer distance, time, accessibility, or crowd level.
- **Output:** the interface displays the final route, path cost, closed/expanded list,
  and open-list trace.

## Environment classification

| Property | Why it applies |
|---|---|
| Observable | The agent can access the location and path data in the campus map. |
| Deterministic | A selected action leads to the same connected location in this simulation. |
| Sequential | The current location and earlier route decisions affect the next decision. |
| Dynamic | Crowd levels and blocked paths can change while selecting a route. |

## Demonstration plan

1. Select `Main Gate` as start and `Science Lab` as goal.
2. Run BFS and note the closed list and route.
3. Run DFS and compare its route with BFS.
4. Run A* with `Distance`, then with `Time` and `Crowd level`.
5. Tick a path under **Obstacles / blocked paths** and run the search again.
6. Explain that the graph, states, actions, open list, closed list, cost, and heuristic
   satisfy the project requirements.
