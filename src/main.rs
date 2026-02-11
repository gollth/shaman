use std::{
    collections::{HashMap, HashSet},
    fmt::Display,
    ops::RangeInclusive,
};

use anyhow::{Context, anyhow};
use clap::Parser;
use itertools::Itertools;
use petgraph::{Graph, algo::astar, graph::NodeIndex};
use termion::{
    color::{Blue, Color, Fg, Magenta, Red},
    style::Reset,
};

#[derive(Debug, Parser)]
struct Args {
    #[arg(long, default_value_t = 20)]
    width: usize,

    #[arg(long, default_value_t = 10)]
    height: usize,
}

#[derive(Debug, Clone)]
struct Robot {
    color: String,
    position: Vertex,
    route: Vec<Vertex>,
}

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

#[derive(Debug)]
struct Layout {
    width: usize,
    height: usize,
    robots: Vec<Robot>,
    graph: Graph<Vertex, ()>,
}

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
struct Vertex {
    x: i32,
    y: i32,
}
impl Vertex {
    fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    fn distance_squared(&self, other: Self) -> f32 {
        ((self.x - other.x) as f32).powi(2) + ((self.y - other.y) as f32).powi(2)
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

    fn node(&self, x: i32, y: i32) -> Option<NodeIndex> {
        let v = Vertex::new(x, y);
        self.graph.node_indices().find(|n| self.graph[*n] == v)
    }

    fn route(&self, start: (i32, i32), goal: (i32, i32)) -> anyhow::Result<Vec<Vertex>> {
        let target = self
            .node(goal.0, goal.1)
            .ok_or(anyhow!("Goal not present: {goal:?}"))?;
        let (_, route) = astar(
            &self.graph,
            self.node(start.0, start.1)
                .ok_or(anyhow!("Start not present: {start:?}"))?,
            |n| target == n,
            |_| 1.,
            |n| self.graph[n].distance_squared(Vertex::new(goal.0, goal.1)),
        )
        .context("Failed to find path from {start} -> {goal}")?;

        Ok(route.into_iter().map(|n| self.graph[n]).collect())
    }

    fn obstacle(&mut self, width: RangeInclusive<i32>, height: RangeInclusive<i32>) {
        let removals = width
            .cartesian_product(height)
            .filter_map(|(x, y)| self.node(x, y))
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
        writeln!(f, "╮")?;
        for y in 0..self.height {
            write!(f, "│")?;
            for x in 0..self.width {
                let v = Vertex::new(x as i32, y as i32);
                match self.robots.iter().find(|r| r.position == v) {
                    Some(robot) => write!(f, "{robot}")?,
                    None => {
                        let paths = self
                            .robots
                            .iter()
                            .filter(|r| r.route.iter().any(|n| *n == v))
                            .collect::<Vec<_>>();
                        if paths.len() > 1 {
                            write!(f, "{}✕{Reset}", Fg(Magenta))?;
                        } else if let Some(robot) = paths.first() {
                            write!(f, "{}", robot.path())?;
                        } else if !self.node(v.x, v.y).is_some() {
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

    // walls
    layout.obstacle(0..=12, 4..=5);
    layout.obstacle(10..=12, 1..=5);

    layout.robots[0].route = layout.route((4, 0), (6, 9))?;
    layout.robots[1].route = layout.route((7, 3), (19, 4))?;

    print!("{layout}");

    Ok(())
}
