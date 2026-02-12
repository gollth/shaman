use std::{
    collections::{BinaryHeap, HashMap, HashSet, VecDeque},
    fmt::Display,
    ops::{Add, RangeFrom, RangeInclusive},
    time::Duration,
};

use anyhow::anyhow;
use clap::Parser;
use itertools::Itertools;
use ordered_float::OrderedFloat;
use petgraph::{Graph, graph::NodeIndex};
use termion::{color::*, cursor, style::Reset};

#[derive(Debug, Parser)]
struct Args {
    /// Amount of colums of the grid
    #[arg(long, default_value_t = 20)]
    width: usize,

    /// Amount of rows of the grid
    #[arg(long, default_value_t = 10)]
    height: usize,

    /// How fast to simulate
    #[arg(short, long, default_value_t = 0.)]
    fps: f32,
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
#[derive(Debug, Clone, Hash, PartialEq, Eq)]
struct Location {
    position: Vertex,
    time: Time,
}

type Time = usize;

#[derive(Debug, Clone)]
struct Robot {
    color: String,
    position: Vertex,
    route: Route,
}

#[derive(Debug, Clone, Default)]
struct Route(VecDeque<Location>);

#[derive(Debug, Clone, Default)]
struct PriorityConstraints {
    temporary: HashMap<usize, Vertex>,
    permanent: Option<(RangeFrom<usize>, Vertex)>,
}

impl From<&Route> for PriorityConstraints {
    fn from(route: &Route) -> Self {
        Self {
            temporary: route.0.iter().map(|l| (l.time, l.position)).collect(),
            permanent: route.0.back().map(|l| (l.time.., l.position)),
        }
    }
}

impl PriorityConstraints {
    fn at(&self, time: usize) -> Option<Vertex> {
        self.temporary
            .get(&time)
            .or_else(|| {
                self.permanent
                    .iter()
                    .find_map(|(range, v)| range.contains(&time).then_some(v))
            })
            .copied()
    }
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
    fn standing_at_goal(locations: impl Iterator<Item = Location>) -> Self {
        Self(locations.collect())
    }
    fn intersection(&self, other: &Self) -> Vec<Location> {
        // TODO: This does not cover goal conflicts anymore, because `Time::Range` is not properly
        // hashed: Maybe ignore last element and manually add it to the intersection?
        let a = self.0.iter().cloned().collect::<HashSet<_>>();
        let b = other.0.iter().cloned().collect::<HashSet<_>>();
        a.intersection(&b).cloned().collect()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Action {
    WAIT,
    N,
    W,
    E,
    S,
}

impl Action {
    const ALL: [Self; 5] = [Self::N, Self::W, Self::S, Self::E, Self::WAIT];

    fn direction(&self) -> Vertex {
        match self {
            Self::WAIT => Vertex::new(0, 0),
            Self::N => Vertex::new(0, -1),
            Self::S => Vertex::new(0, 1),
            Self::W => Vertex::new(-1, 0),
            Self::E => Vertex::new(1, 0),
        }
    }

    fn cost(&self) -> f32 {
        match self {
            Self::WAIT => 1.,
            _ => 2.,
        }
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
        self.route_with(start, goal, PriorityConstraints::default())
    }

    fn route_with(
        &self,
        start: Vertex,
        goal: Vertex,
        constraints: PriorityConstraints,
    ) -> anyhow::Result<Route> {
        let target = self
            .node(goal)
            .ok_or(anyhow!("Goal not present: {goal:?}"))?;
        #[derive(Debug, Clone, PartialEq, Eq)]
        struct Item {
            n: NodeIndex,
            cost: OrderedFloat<f32>,
            time: usize,
            came_from: Option<Box<Item>>,
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

        let mut open = BinaryHeap::new();
        let mut scores = HashMap::new();
        scores.insert((0, start), 0.0);
        let start = self
            .node(start)
            .ok_or(anyhow!("Start not present or free: {start:?}"))?;
        open.push(Item {
            cost: 0.0.into(),
            time: 0,
            n: start,
            came_from: None,
        });

        const MAX_ITER: usize = 10000;
        let mut i = 0;
        while let Some(item) = open.pop() {
            anyhow::ensure!(
                i < MAX_ITER,
                "Failed to find a solution within {MAX_ITER} iterations",
            );
            i += 1;
            if item.n == target {
                // Reached goal
                let mut current = Box::new(item);
                let mut route = VecDeque::new();
                route.push_back(Location {
                    time: current.time,
                    position: self.graph[current.n],
                });
                while let Some(previous) = current.came_from {
                    route.push_front(Location {
                        time: previous.time,
                        position: self.graph[previous.n],
                    });
                    current = previous;
                }
                return Ok(Route::standing_at_goal(route.into_iter()));
            }

            // Node expansion
            for action in &Action::ALL {
                let t = item.time + 1;
                let v = self.graph[item.n];
                let location = Location {
                    time: t,
                    position: v + action.direction(),
                };
                let Some(candidate) = self.node(location.position) else {
                    // candidate not reachable
                    continue;
                };

                // Same location constraint check
                if constraints
                    .at(t)
                    .is_some_and(|obstacle| obstacle == location.position)
                {
                    // candidate would collide with a priority constraint in the future
                    continue;
                }

                // Swapping location constraint check
                if constraints
                    .at(item.time)
                    .zip(constraints.at(t))
                    .is_some_and(|(now, next)| now == location.position && next == v)
                {
                    // candidate would switch location with the priority constraint
                    continue;
                }

                let tentative_g = scores[&(item.time, v)] + action.cost();
                if scores
                    .get(&(t, location.position))
                    .is_none_or(|g| tentative_g < *g)
                {
                    scores.insert((t, location.position), tentative_g);
                    // valid candidate
                    let h = self.graph[candidate].distance_squared(goal);
                    // println!("  + {action:?} => {tentative_g} + {h}");
                    let item = Item {
                        cost: OrderedFloat(tentative_g + h),
                        n: candidate,
                        time: t,
                        came_from: Some(Box::new(item.clone())),
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

    fn simulate(&mut self) {
        for robot in &mut self.robots {
            let Some(next) = robot.route.0.pop_front() else {
                continue;
            };

            robot.position = next.position;
        }
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
    layout.robots.push(Robot::new(5, 0, Blue));
    layout.robots.push(Robot::new(8, 3, Red));
    layout.robots.push(Robot::new(19, 8, Green));

    // walls
    layout.obstacle(0..=12, 4..=5);
    layout.obstacle(10..=11, 1..=5);
    layout.obstacle(14..=19, 4..=5);

    layout.robots[0].route = layout.route(layout.robots[0].position, Vertex::new(6, 9))?;
    layout.robots[1].route = layout.route_with(
        layout.robots[1].position,
        Vertex::new(19, 3),
        (&layout.robots[0].route).into(),
    )?;
    layout.robots[2].route = layout.route_with(
        layout.robots[2].position,
        Vertex::new(5, 0),
        (&layout.robots[0].route).into(),
    )?;

    if args.fps == 0. {
        println!("{layout}");
        return Ok(());
    }

    let dt = Duration::from_secs_f32(1. / args.fps);
    let steps = layout
        .robots
        .iter()
        .map(|r| r.route.0.len())
        .max()
        .unwrap_or_default();
    print!("{}", cursor::Hide);
    for _ in 0..steps {
        layout.simulate();
        print!(
            "{layout}{}{}",
            cursor::Left(layout.width as u16 + 2),
            cursor::Up(layout.height as u16 + 2)
        );
        std::thread::sleep(dt);
    }
    print!("{layout}{}", cursor::Show);

    Ok(())
}
