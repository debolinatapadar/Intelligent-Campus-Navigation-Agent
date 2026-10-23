"""
CSN304 Artificial Intelligence Project - P-1
Intelligent Campus Navigation Agent

Run:
    python intelligent_campus_navigation_agent.py

The project is intentionally written with Python's standard library only.
It demonstrates graph representation, BFS, DFS, A*, environment analysis,
obstacles, route criteria, and a Tkinter GUI.
"""

import heapq
import math
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk


@dataclass(frozen=True)
class Edge:
    a: str
    b: str
    distance: float       # metres (project/demo values)
    time: float           # minutes (project/demo values)
    accessible: bool
    crowd: str            # Low, Medium, High

    @property
    def key(self):
        return tuple(sorted((self.a, self.b)))


LOCATIONS = {
    "Main Gate": (65, 310),
    "Admin Block": (145, 170),
    "Library": (280, 100),
    "CSE Block": (420, 165),
    "Science Lab": (575, 90),
    "Cafeteria": (420, 315),
    "Auditorium": (280, 400),
    "Hostel": (590, 405),
    "Sports Complex": (735, 300),
    "Medical Center": (735, 105),
}


EDGES = [
    Edge("Main Gate", "Admin Block", 180, 3, True, "Low"),
    Edge("Main Gate", "Library", 300, 5, True, "Medium"),
    Edge("Admin Block", "Library", 150, 3, True, "Low"),
    Edge("Admin Block", "CSE Block", 260, 5, True, "Medium"),
    Edge("Admin Block", "Medical Center", 430, 8, True, "Low"),
    Edge("Library", "CSE Block", 210, 4, True, "High"),
    Edge("Library", "Auditorium", 280, 5, True, "Low"),
    Edge("CSE Block", "Science Lab", 170, 3, False, "Medium"),
    Edge("CSE Block", "Cafeteria", 230, 4, True, "High"),
    Edge("Science Lab", "Medical Center", 210, 4, True, "Low"),
    Edge("Science Lab", "Sports Complex", 330, 7, True, "Medium"),
    Edge("Cafeteria", "Auditorium", 160, 3, True, "High"),
    Edge("Cafeteria", "Hostel", 300, 6, True, "Medium"),
    Edge("Auditorium", "Hostel", 250, 5, True, "Low"),
    Edge("Hostel", "Sports Complex", 200, 4, True, "High"),
    Edge("Hostel", "Medical Center", 240, 4, True, "Low"),
    Edge("Medical Center", "Sports Complex", 260, 5, True, "Medium"),
]


CROWD_PENALTY = {"Low": 0, "Medium": 35, "High": 80}


def make_graph(blocked):
    """Return an adjacency list after removing blocked paths."""
    graph = {location: [] for location in LOCATIONS}
    for edge in EDGES:
        if edge.key in blocked:
            continue
        graph[edge.a].append((edge.b, edge))
        graph[edge.b].append((edge.a, edge))
    for neighbours in graph.values():
        neighbours.sort(key=lambda item: item[0])
    return graph


def edge_cost(edge, criterion):
    if criterion == "Distance":
        return edge.distance
    if criterion == "Time":
        return edge.time
    if criterion == "Accessibility":
        return edge.distance + (0 if edge.accessible else 1000)
    if criterion == "Crowd level":
        return edge.distance + CROWD_PENALTY[edge.crowd]
    return edge.distance


def heuristic(node, goal, criterion):
    """Admissible layout-based estimate for the chosen edge cost."""
    straight = math.dist(LOCATIONS[node], LOCATIONS[goal])
    ratios = []
    for edge in EDGES:
        layout_distance = math.dist(LOCATIONS[edge.a], LOCATIONS[edge.b])
        if layout_distance:
            ratios.append(edge_cost(edge, criterion) / layout_distance)
    return straight * min(ratios)


def search(start, goal, algorithm, criterion, blocked):
    """Run BFS, DFS, or A* and return a result dictionary for the GUI."""
    graph = make_graph(blocked)
    if start == goal:
        return {"path": [start], "cost": 0, "closed": [start],
                "open_trace": ["OPEN: []"], "algorithm": algorithm}

    parent = {start: None}
    closed = []
    open_trace = []

    if algorithm in ("BFS", "DFS"):
        frontier = [start]
        visited = {start}
        found = False
        while frontier:
            current = frontier.pop(0) if algorithm == "BFS" else frontier.pop()
            closed.append(current)
            if current == goal:
                found = True
                open_trace.append("OPEN at goal: " + ", ".join(frontier))
                break
            for neighbour, _edge in graph[current]:
                if neighbour not in visited:
                    visited.add(neighbour)
                    parent[neighbour] = current
                    frontier.append(neighbour)
            label = "queue" if algorithm == "BFS" else "stack"
            open_trace.append(
                f"After expanding {current} | {label}: " +
                (", ".join(frontier) if frontier else "[]")
            )
        if not found:
            return {"path": [], "cost": None, "closed": closed,
                    "open_trace": open_trace + ["OPEN: []"],
                    "algorithm": algorithm}
    else:
        counter = 0
        frontier = [(heuristic(start, goal, criterion), counter, start)]
        g_score = {start: 0}
        best_open = {start: 0}
        found = False
        while frontier:
            _f, _order, current = heapq.heappop(frontier)
            if current in closed:
                continue
            closed.append(current)
            best_open.pop(current, None)
            if current == goal:
                found = True
                open_trace.append("OPEN at goal: " +
                                  (", ".join(best_open) if best_open else "[]"))
                break
            for neighbour, edge in graph[current]:
                new_g = g_score[current] + edge_cost(edge, criterion)
                if new_g < g_score.get(neighbour, float("inf")):
                    parent[neighbour] = current
                    g_score[neighbour] = new_g
                    counter += 1
                    f_score = new_g + heuristic(neighbour, goal, criterion)
                    heapq.heappush(frontier, (f_score, counter, neighbour))
                    best_open[neighbour] = f_score
            open_items = sorted(best_open, key=lambda item: best_open[item])
            open_trace.append(
                f"After expanding {current} | priority queue: " +
                (", ".join(f"{n} (f={best_open[n]:.1f})" for n in open_items)
                 if open_items else "[]")
            )
        if not found:
            return {"path": [], "cost": None, "closed": closed,
                    "open_trace": open_trace + ["OPEN: []"],
                    "algorithm": algorithm}

    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()

    # The final cost is calculated from the actual path, not from GUI labels.
    path_cost = 0
    for a, b in zip(path, path[1:]):
        path_cost += edge_cost(next(edge for edge in EDGES if edge.key ==
                                    tuple(sorted((a, b)))), criterion)
    return {"path": path, "cost": path_cost, "closed": closed,
            "open_trace": open_trace, "algorithm": algorithm}


class CampusApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Intelligent Campus Navigation Agent - CSN304")
        self.root.geometry("1200x760")
        self.root.minsize(1050, 680)
        self.blocked_vars = {}
        self.last_result = None
        self.build_ui()
        self.draw_map()
        self.run_search()

    def build_ui(self):
        title = tk.Label(self.root, text="INTELLIGENT CAMPUS NAVIGATION AGENT",
                         font=("Arial", 18, "bold"), fg="#145da0")
        title.pack(pady=(10, 2))
        tk.Label(self.root, text="CSN304 Artificial Intelligence | Group G-100 | Project P-1",
                 font=("Arial", 10)).pack(pady=(0, 8))

        controls = ttk.LabelFrame(self.root, text="Agent controls")
        controls.pack(fill="x", padx=12, pady=5)

        self.start_var = tk.StringVar(value="Main Gate")
        self.goal_var = tk.StringVar(value="Science Lab")
        self.algorithm_var = tk.StringVar(value="A*")
        self.criterion_var = tk.StringVar(value="Distance")

        fields = [
            ("Start location", self.start_var, list(LOCATIONS)),
            ("Goal location", self.goal_var, list(LOCATIONS)),
            ("Search algorithm", self.algorithm_var, ["BFS", "DFS", "A*"]),
            ("Route preference", self.criterion_var,
             ["Distance", "Time", "Accessibility", "Crowd level"]),
        ]
        for col, (label, variable, values) in enumerate(fields):
            ttk.Label(controls, text=label).grid(row=0, column=col, padx=8, pady=4)
            ttk.Combobox(controls, textvariable=variable, values=values,
                         state="readonly", width=19).grid(row=1, column=col,
                                                            padx=8, pady=4)
        ttk.Button(controls, text="Find route", command=self.run_search).grid(
            row=1, column=4, padx=10, pady=4)
        ttk.Button(controls, text="Reset obstacles", command=self.reset_obstacles).grid(
            row=1, column=5, padx=5, pady=4)

        body = tk.Frame(self.root)
        body.pack(fill="both", expand=True, padx=12, pady=6)
        left = tk.Frame(body)
        left.pack(side="left", fill="both", expand=True)
        right = tk.Frame(body, width=390)
        right.pack(side="right", fill="y", padx=(10, 0))
        right.pack_propagate(False)

        self.canvas = tk.Canvas(left, bg="#f8fbff", highlightthickness=1,
                                highlightbackground="#b8cbe0")
        self.canvas.pack(fill="both", expand=True)
        tk.Label(left, text="Blue edges = selected route | Red dashed edges = blocked obstacles | Grey edges = available paths",
                 fg="#40566e", font=("Arial", 9)).pack(anchor="w", pady=3)

        obstacle_box = ttk.LabelFrame(right, text="Obstacles / blocked paths")
        obstacle_box.pack(fill="x", pady=(0, 7))
        tk.Label(obstacle_box, text="Tick a path to close it, then Find route.",
                 fg="#5d6875").pack(anchor="w", padx=8, pady=(5, 2))
        for edge in EDGES:
            key = edge.key
            variable = tk.BooleanVar(value=False)
            self.blocked_vars[key] = variable
            ttk.Checkbutton(obstacle_box,
                            text=f"{edge.a}  ↔  {edge.b}", variable=variable,
                            command=self.draw_map).pack(anchor="w", padx=8, pady=1)

        result_box = ttk.LabelFrame(right, text="Agent output")
        result_box.pack(fill="both", expand=True)
        self.output = tk.Text(result_box, wrap="word", font=("Consolas", 9),
                              bg="#fbfdff", state="disabled")
        self.output.pack(fill="both", expand=True, padx=5, pady=5)

    def blocked_edges(self):
        return {key for key, variable in self.blocked_vars.items() if variable.get()}

    def reset_obstacles(self):
        for variable in self.blocked_vars.values():
            variable.set(False)
        self.draw_map()
        self.run_search()

    def draw_map(self):
        if not hasattr(self, "canvas"):
            return
        self.canvas.delete("all")
        blocked = self.blocked_edges()
        route_edges = set()
        if self.last_result and self.last_result.get("path"):
            route_edges = {tuple(sorted(pair)) for pair in
                           zip(self.last_result["path"], self.last_result["path"][1:])}
        for edge in EDGES:
            x1, y1 = LOCATIONS[edge.a]
            x2, y2 = LOCATIONS[edge.b]
            if edge.key in blocked:
                color, width, dash = "#d9534f", 2, (8, 4)
            elif edge.key in route_edges:
                color, width, dash = "#1376d2", 5, None
            else:
                color, width, dash = "#aab8c7", 2, None
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=width,
                                    dash=dash)
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            self.canvas.create_rectangle(mx - 20, my - 9, mx + 20, my + 9,
                                         fill="white", outline="")
            self.canvas.create_text(mx, my, text=f"{edge.distance:g}m",
                                    fill="#4c5d70", font=("Arial", 8))

        for name, (x, y) in LOCATIONS.items():
            fill = "#e9f3ff"
            if name == self.start_var.get():
                fill = "#b9f6ca"
            if name == self.goal_var.get():
                fill = "#ffe0b2"
            self.canvas.create_oval(x - 42, y - 22, x + 42, y + 22,
                                    fill=fill, outline="#1c527e", width=2)
            self.canvas.create_text(x, y, text=name, width=75,
                                    font=("Arial", 9, "bold"), fill="#12344d")

    def run_search(self):
        start, goal = self.start_var.get(), self.goal_var.get()
        algorithm, criterion = self.algorithm_var.get(), self.criterion_var.get()
        self.last_result = search(start, goal, algorithm, criterion,
                                  self.blocked_edges())
        self.draw_map()
        self.show_result(start, goal, algorithm, criterion, self.last_result)

    def show_result(self, start, goal, algorithm, criterion, result):
        self.output.configure(state="normal")
        self.output.delete("1.0", "end")
        lines = [
            "RESULT",
            "=" * 44,
            f"Algorithm: {algorithm}",
            f"Preference: {criterion}",
            f"Start: {start}",
            f"Goal: {goal}",
            "",
        ]
        if result["path"]:
            lines += ["FINAL ROUTE", " -> ".join(result["path"]),
                      f"Path cost ({criterion.lower()}): {result['cost']:.1f}",
                      f"Steps/edges: {len(result['path']) - 1}", ""]
        else:
            lines += ["NO ROUTE FOUND.",
                      "The goal is unreachable with these blocked paths.", ""]
        lines += ["CLOSED LIST (expanded states)",
                  " -> ".join(result["closed"]) or "[]", "",
                  "OPEN-LIST TRACE"]
        lines.extend(result["open_trace"])
        lines += ["", "Environment: observable, deterministic, sequential, dynamic"]
        self.output.insert("1.0", "\n".join(lines))
        self.output.configure(state="disabled")


def main():
    root = tk.Tk()
    CampusApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
