use std::{
    collections::{BinaryHeap, HashMap, HashSet, VecDeque},
    fmt::Display,
    ops::{Add, RangeInclusive},
};

use anyhow::anyhow;
use clap::Parser;
use itertools::Itertools;
use ordered_float::OrderedFloat;
use petgraph::{Graph, graph::NodeIndex};
use termion::{
    color::{Blue, Color, Fg, Green, Magenta, Red},
    style::Reset,
};

#[derive(Debug, Parser)]
struct Args {
    #[arg(long, default_value_t = 20)]
    width: usize,

    #[arg(long, default_value_t = 10)]
    height: usize,
}

#[derive(Debug)]
struct Layout {
    width: usize,
    height: usize,
    robots: Vec<Robot>,
    graph: Graph<Vertex, ()>,
}

/// Position of each cell in the layout/graph
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
struct Vertex {
    x: i32,
    y: i32,
}

/// Position of a robot at a specific point in time
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
struct Location {
    position: Vertex,
    time: Time,
}

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
enum Time {
    Instant(usize),
    Forever,
}

#[derive(Debug, Clone)]
struct Robot {
    color: String,
    position: Vertex,
    route: Route,
}

#[derive(Debug, Clone, Default)]
struct Route(Vec<Location>);

impl Robot {
    fn new<C: Color>(x: i32, y: i32, color: C) -> Self {
        Self {
            color: format!("{}", Fg(color)),
            position: Vertex::new(x, y),
            route: Default::default(),
        }
    }

    fn path(&self) -> String {
        format!("{}·{Reset}", self.color)
    }
}

impl Display for Robot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}●{Reset}", self.color)
    }
}

impl Vertex {
    const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    fn distance_squared(&self, other: Self) -> f32 {
        ((self.x - other.x) as f32).powi(2) + ((self.y - other.y) as f32).powi(2)
    }
}

impl Add for Vertex {
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Route {
    fn continuously(vertices: impl Iterator<Item = Vertex>) -> Self {
        let mut path = vertices
            .enumerate()
            .map(|(time, position)| Location {
                position,
                time: Time::Instant(time),
            })
            .collect::<Vec<_>>();
        if let Some(goal) = path.last_mut() {
            goal.time = Time::Forever;
        }
        Self(path)
    }
    fn intersection(&self, other: &Self) -> Vec<Location> {
        let a = self.0.iter().copied().collect::<HashSet<_>>();
        let b = other.0.iter().copied().collect::<HashSet<_>>();
        a.intersection(&b).copied().collect()
    }
}

impl Layout {
    fn new(width: usize, height: usize) -> Self {
        let mut graph = Graph::new();
        let mut cache = HashMap::new();
        for (x, y) in (0..width).cartesian_product(0..height) {
            let v = Vertex::new(x as i32, y as i32);
            cache.insert((v.x, v.y), graph.add_node(v));
        }

        for ((x, y), node) in cache.iter() {
            for (i, j) in &[(-1, 0), (1, 0), (0, -1), (0, 1)] {
                let neighbor = (x + i, y + j);
                if let Some(n) = cache.get(&neighbor).copied() {
                    graph.add_edge(*node, n, ());
                }
            }
        }
        Self {
            graph,
            width,
            height,
            robots: Default::default(),
        }
    }

    fn node(&self, v: Vertex) -> Option<NodeIndex> {
        self.graph.node_indices().find(|n| self.graph[*n] == v)
    }

    fn route(&self, start: Vertex, goal: Vertex) -> anyhow::Result<Route> {
        let target = self
            .node(goal)
            .ok_or(anyhow!("Goal not present: {goal:?}"))?;
        #[derive(Debug, Clone, PartialEq, Eq)]
        struct Item {
            n: NodeIndex,
            cost: OrderedFloat<f32>,
            time: usize,
        }

        impl Ord for Item {
            fn cmp(&self, other: &Self) -> std::cmp::Ordering {
                other.cost.cmp(&self.cost) // reverse for min heap
            }
        }
        impl PartialOrd for Item {
            fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
                Some(self.cmp(other))
            }
        }

        const N: Vertex = Vertex::new(0, -1);
        const S: Vertex = Vertex::new(0, 1);
        const W: Vertex = Vertex::new(-1, 0);
        const E: Vertex = Vertex::new(1, 0);
        const WAIT: Vertex = Vertex::new(0, 0);

        let mut open = BinaryHeap::new();
        let mut scores = HashMap::new();
        let mut routing_table = HashMap::<NodeIndex, NodeIndex>::new();
        let start = self
            .node(start)
            .ok_or(anyhow!("Start not present or free: {start:?}"))?;
        scores.insert(start, 0.0);
        open.push(Item {
            cost: 0.0.into(),
            time: 0,
            n: start,
        });

        while let Some(item) = open.pop() {
            if item.n == target {
                // Reached goal
                let mut current = item.n;
                let mut route = VecDeque::new();
                route.push_back(target);
                while let Some(previous) = routing_table.get(&current).copied() {
                    route.push_front(previous);
                    current = previous;
                }
                return Ok(Route::continuously(
                    route.into_iter().map(|n| self.graph[n]),
                ));
            }

            // Node expansion
            for action in &[N, W, S, E, WAIT] {
                let Some(candidate) = self.node(self.graph[item.n] + *action) else {
                    // candidate not reachable
                    continue;
                };

                let tentative_g = scores[&item.n] + 1.0;
                if scores.get(&candidate).is_none_or(|g| tentative_g < *g) {
                    scores.insert(candidate, tentative_g);
                    // valid candidate
                    let h = self.graph[candidate].distance_squared(goal);
                    routing_table.insert(candidate, item.n);
                    let item = Item {
                        cost: OrderedFloat(tentative_g + h),
                        n: candidate,
                        time: item.time + 1,
                    };
                    open.push(item);
                }
            }
        }

        Err(anyhow!("Failed to find path from {start:?} -> {goal:?}"))
    }

    fn obstacle(&mut self, width: RangeInclusive<i32>, height: RangeInclusive<i32>) {
        let removals = width
            .cartesian_product(height)
            .filter_map(|(x, y)| self.node(Vertex::new(x, y)))
            .collect::<HashSet<_>>();
        self.graph.retain_nodes(|_, n| !removals.contains(&n));
    }
}

impl Display for Layout {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "╭")?;
        for _ in 0..self.width {
            write!(f, "─")?;
        }

        let intersections = self
            .robots
            .iter()
            .tuple_combinations()
            .flat_map(|(a, b)| a.route.intersection(&b.route))
            .map(|loc: Location| loc.position)
            .collect::<HashSet<_>>();
        writeln!(f, "╮")?;
        for y in 0..self.height {
            write!(f, "│")?;
            for x in 0..self.width {
                let v = Vertex::new(x as i32, y as i32);
                match self.robots.iter().find(|r| r.position == v) {
                    Some(robot) => write!(f, "{robot}")?,
                    None => {
                        if intersections.contains(&v) {
                            write!(f, "{}✕{Reset}", Fg(Magenta))?;
                        } else if let Some(robot) = self
                            .robots
                            .iter()
                            .find(|r| r.route.0.iter().any(|n| n.position == v))
                        {
                            write!(f, "{}", robot.path())?;
                        } else if !self.node(v).is_some() {
                            // Obstacle
                            write!(f, "█")?;
                        } else {
                            // Free space
                            write!(f, " ")?;
                        }
                    }
                }
            }
            writeln!(f, "│")?;
        }
        write!(f, "╰")?;
        for _ in 0..self.width {
            write!(f, "─")?;
        }
        writeln!(f, "╯")?;
        Ok(())
    }
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let mut layout = Layout::new(args.width, args.height);

    // robots
    layout.robots.push(Robot::new(4, 0, Blue));
    layout.robots.push(Robot::new(7, 3, Red));
    layout.robots.push(Robot::new(0, 9, Green));

    // walls
    layout.obstacle(0..=12, 4..=5);
    layout.obstacle(10..=11, 1..=5);

    layout.robots[0].route = layout.route(layout.robots[0].position, Vertex::new(6, 9))?;
    layout.robots[1].route = layout.route(layout.robots[1].position, Vertex::new(19, 4))?;
    layout.robots[2].route = layout.route(layout.robots[2].position, Vertex::new(6, 9))?;

    print!("{layout}");

    Ok(())
}
