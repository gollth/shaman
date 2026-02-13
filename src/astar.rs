//! Low level path planning for a single robot
use std::{
    collections::{BinaryHeap, VecDeque},
    iter::Sum,
    ops::{AddAssign, RangeFrom},
};

use miette::SourceSpan;
use ordered_float::OrderedFloat;
use rustc_hash::FxHashMap;

use crate::{
    layout::{Layout, Vertex},
    parser::ParseError,
    robot::Location,
    route::Route,
};

/// A priority constraint, which this [crate::astar::solve()] needs to respect
#[derive(Debug, Clone, Default)]
pub struct RightOfWay {
    temporary: FxHashMap<usize, Vertex>,
    permanent: Vec<(RangeFrom<usize>, Vertex)>,
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

impl AddAssign for RightOfWay {
    fn add_assign(&mut self, other: Self) {
        self.temporary.extend(other.temporary);
        self.permanent.extend(other.permanent);
    }
}

impl Sum for RightOfWay {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(RightOfWay::default(), |mut acc, other| {
            acc += other;
            acc
        })
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
            permanent: route
                .iter()
                .rev()
                .take(1)
                .map(|l| (l.time.., l.position))
                .collect(),
        }
    }
}

/// Possible action the robot can take on a single location
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    #[default]
    WAIT,
    N,
    W,
    E,
    S,
}

impl Action {
    pub const ALL: [Self; 5] = [Self::N, Self::W, Self::S, Self::E, Self::WAIT];

    pub fn direction(&self) -> Vertex {
        match self {
            Self::WAIT => Vertex::new(0, 0),
            Self::N => Vertex::new(0, -1),
            Self::S => Vertex::new(0, 1),
            Self::W => Vertex::new(-1, 0),
            Self::E => Vertex::new(1, 0),
        }
    }

    fn cost(&self, previous: Self) -> f32 {
        match (self, previous) {
            (Self::WAIT, _) => 1.,
            (a, b) if *a == b => 2.,
            _ => 5.,
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
    start: (Vertex, SourceSpan),
    goal: (Vertex, SourceSpan),
    constraint: &RightOfWay,
) -> miette::Result<Route> {
    miette::ensure!(!layout.is_blocked(start.0), "Start not free: {}", start.0);
    miette::ensure!(!layout.is_blocked(goal.0), "Goal not free: {}", goal.0);

    let mut open = BinaryHeap::new();
    let mut scores = FxHashMap::default();
    let s = Location {
        time: 0,
        position: start.0,
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
        if i >= MAX_ITER {
            break;
        }
        i += 1;
        if item.location.position == goal.0 {
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
            let previous_action = item
                .came_from
                .as_ref()
                .map(|prev| here - prev.location.position)
                .unwrap_or_default();

            let tentative_g = scores[&item.location] + action.cost(previous_action);
            if scores.get(&candidate).is_none_or(|g| tentative_g < *g) {
                scores.insert(candidate, tentative_g);
                // valid candidate
                let h = candidate.position.distance_squared(goal.0);
                let item = Item {
                    cost: OrderedFloat(tentative_g + h),
                    location: candidate,
                    came_from: Some(Box::new(item.clone())),
                };
                open.push(item);
            }
        }
    }

    Err(ParseError::RouteNotFound {
        src: layout.code(),
        start: start.1,
        goal: goal.1,
    }
    .into())
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
