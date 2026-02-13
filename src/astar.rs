use std::{
    collections::{BinaryHeap, HashMap, VecDeque},
    ops::RangeFrom,
};

use anyhow::anyhow;
use ordered_float::OrderedFloat;

use crate::{
    layout::{Layout, Vertex},
    robot::Location,
    route::Route,
};

/// A priority constraint, which this `astar::solve()` needs to respect
#[derive(Debug, Clone, Default)]
pub struct RightOfWay {
    temporary: HashMap<usize, Vertex>,
    permanent: Option<(RangeFrom<usize>, Vertex)>,
}

impl RightOfWay {
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

impl From<&Route> for RightOfWay {
    fn from(route: &Route) -> Self {
        Self {
            temporary: route
                .iter()
                .rev()
                .skip(1)
                .map(|l| (l.time, l.position))
                .collect(),
            permanent: route.iter().last().map(|l| (l.time.., l.position)),
        }
    }
}

/// Possible action the robot can take on a single location
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

/// Priority-aware A*
///
/// Plan the shortest path from `start` -> `goal` avoiding static obstacles on `layout`.
/// Also avoid the dynamic obstacle (other robot's path) defined by `constraint`, i.e. by waiting
/// or rerouting
pub fn solve(
    layout: &Layout,
    start: Vertex,
    goal: Vertex,
    constraint: &RightOfWay,
) -> anyhow::Result<Route> {
    anyhow::ensure!(!layout.is_blocked(start), "Start not free: {start}");
    anyhow::ensure!(!layout.is_blocked(goal), "Goal not free: {goal}");

    let mut open = BinaryHeap::new();
    let mut scores = HashMap::new();
    let s = Location {
        time: 0,
        position: start,
    };
    scores.insert(s, 0.0);
    open.push(Item {
        cost: 0.0.into(),
        location: s,
        came_from: None,
    });

    // TODO: Detect standstill better, e.g. by having an upper bound for consequtive WAITs
    const MAX_ITER: usize = 10000;
    let mut i = 0;
    while let Some(item) = open.pop() {
        anyhow::ensure!(
            i < MAX_ITER,
            "Failed to find a solution within {MAX_ITER} iterations",
        );
        i += 1;
        if item.location.position == goal {
            // Reached goal
            let mut current = Box::new(item);
            let mut route = VecDeque::new();
            route.push_back(current.location);
            while let Some(previous) = current.came_from {
                route.push_front(previous.location);
                current = previous;
            }
            return Ok(route.into_iter().collect());
        }

        // Node expansion
        for action in &Action::ALL {
            let now = item.location.time;
            let then = now + 1;
            let here = item.location.position;
            let there = here + action.direction();
            let candidate = Location {
                position: there,
                time: then,
            };
            if layout.is_blocked(there) {
                // candidate not reachable
                continue;
            }

            // Same location constraint check
            if constraint
                .at(then)
                .is_some_and(|obstacle| obstacle == candidate.position)
            {
                // candidate would collide with a priority constraint in the future
                continue;
            }

            // Swapping location constraint check
            if constraint
                .at(now)
                .zip(constraint.at(then))
                .is_some_and(|(now, then)| now == there && then == here)
            {
                // candidate would switch location with the priority constraint
                continue;
            }

            let tentative_g = scores[&item.location] + action.cost();
            if scores.get(&candidate).is_none_or(|g| tentative_g < *g) {
                scores.insert(candidate, tentative_g);
                // valid candidate
                let h = candidate.position.distance_squared(goal);
                // println!("  + {action:?} => {tentative_g} + {h}");
                let item = Item {
                    cost: OrderedFloat(tentative_g + h),
                    location: candidate,
                    came_from: Some(Box::new(item.clone())),
                };
                open.push(item);
            }
        }
    }

    Err(anyhow!("Failed to find path from {start:?} -> {goal:?}"))
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Item {
    location: Location,
    cost: OrderedFloat<f32>,
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
