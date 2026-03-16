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
    error::ShamanError,
    layout::{Layout, Vertex},
    robot::Location,
    route::{Route, Segment},
};

/// A priority constraint, which this [crate::astar::solve()] needs to respect
#[derive(Debug, Clone, Default)]
pub struct RightOfWay {
    // TODO: At this point this really should become a HashSet...
    temporary: FxHashMap<usize, Vertex>,
    permanent: Vec<(RangeFrom<usize>, Vertex)>,
}

impl RightOfWay {
    pub(crate) fn at(&self, time: usize) -> Option<Vertex> {
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
    Wait,
    N,
    W,
    E,
    S,
}

impl Action {
    pub const ALL: [Self; 5] = [Self::N, Self::W, Self::S, Self::E, Self::Wait];

    pub fn direction(&self) -> Vertex {
        match self {
            Self::Wait => Vertex::new(0, 0),
            Self::N => Vertex::new(0, -1),
            Self::S => Vertex::new(0, 1),
            Self::W => Vertex::new(-1, 0),
            Self::E => Vertex::new(1, 0),
        }
    }

    fn cost(&self, previous: Self) -> f32 {
        match (previous, *self) {
            (_, Self::Wait) => 1.2, // slightly penalizing to encourage movement
            (a, b) if a == b => 1., // moving in the same direction
            (Self::Wait, _) => 1.5, // Was stopped, now starting to move
            _ => 3.,                // Changing direction, this is costly!
        }
    }
}

/// Normal A* without any priority constraints
///
/// Quicker than [solve()] and is able to return an `Err` if no path exists
pub(crate) fn vacuum(
    layout: &Layout,
    source_span: (Location, SourceSpan),
    goal_span: (Vertex, SourceSpan),
) -> Result<Route, ShamanError> {
    let segment = solve(
        layout,
        source_span,
        goal_span,
        &RightOfWay::default(),
        usize::MAX,
    )?;
    Ok(segment.into_route())
}

/// Priority-aware A*
///
/// Plan the shortest path from `start` -> `goal` avoiding static obstacles on `layout`.
/// Also avoid the dynamic obstacle (other robot's path) defined by `constraint`, i.e. by waiting
/// or rerouting. The `constraints` are only considered up a a time window of `horizon`.
pub fn solve(
    layout: &Layout,
    start: (Location, SourceSpan),
    goal: (Vertex, SourceSpan),
    constraint: &RightOfWay,
    window: usize,
) -> Result<Segment, ShamanError> {
    let mut open = BinaryHeap::new();
    let mut scores = FxHashMap::default();
    let mut came_from = FxHashMap::default();
    let s = start.0;
    scores.insert(s, 0.0);
    open.push(Item {
        cost: 0.0.into(),
        location: s,
    });

    let horizon = start.0.time.saturating_add(window);
    let mut best_node = s;
    let mut min_h = s.position.distance_squared(goal.0);

    while let Some(item) = open.pop() {
        if item.location.position == goal.0 {
            // Reached goal
            return Ok(Segment::Complete(reconstruct_route(
                item.location,
                &came_from,
            )));
        }

        if item.location.time > layout.free_cell_count() {
            // Safety timeout
            continue;
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

            // Conflict check (only within window)
            if candidate.time <= horizon {
                if constraint
                    .at(then)
                    .is_some_and(|obstacle| obstacle == candidate.position)
                {
                    // candidate would collide with a priority constraint in the future
                    continue;
                }

                if constraint
                    .at(now)
                    .zip(constraint.at(then))
                    .is_some_and(|(now, then)| now == there && then == here)
                {
                    // candidate would switch location with the priority constraint
                    continue;
                }
            }

            let previous_action = came_from
                .get(&item.location)
                .map(|prev| here - prev.position)
                .unwrap_or_default();

            let tentative_g = scores[&item.location] + action.cost(previous_action);
            if scores.get(&candidate).is_none_or(|g| tentative_g < *g) {
                scores.insert(candidate, tentative_g);
                // valid candidate
                let h = candidate.position.distance_squared(goal.0);
                if h < min_h {
                    min_h = h;
                    best_node = candidate;
                }
                came_from.insert(candidate, item.location);
                let item = Item {
                    cost: OrderedFloat(tentative_g + h),
                    location: candidate,
                };
                open.push(item);
            }
        }
    }

    if best_node.time >= horizon {
        // Goal not reached within horizon, return the best path found within the window
        return Ok(Segment::Partial(reconstruct_route(best_node, &came_from)));
    }

    Err(ShamanError::RouteNotFound {
        src: layout.code(),
        start: start.1,
        goal: goal.1,
    })
}

fn reconstruct_route(mut current: Location, came_from: &FxHashMap<Location, Location>) -> Route {
    let mut route = VecDeque::new();
    route.push_back(current);
    while let Some(previous) = came_from.get(&current).copied() {
        route.push_front(previous);
        current = previous;
    }
    route.into_iter().collect()
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Item {
    location: Location,
    cost: OrderedFloat<f32>,
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
