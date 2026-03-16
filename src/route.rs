use std::collections::VecDeque;

use itertools::Itertools;

use crate::{Time, astar::RightOfWay, layout::Vertex, robot::Location};

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Route(VecDeque<Location>);

impl FromIterator<Location> for Route {
    fn from_iter<T: IntoIterator<Item = Location>>(iter: T) -> Self {
        Self(iter.into_iter().collect())
    }
}

impl Route {
    pub fn duration(&self) -> Time {
        self.0.back().map(|l| l.time).unwrap_or_default()
    }

    pub fn iter(&self) -> impl DoubleEndedIterator<Item = Location> {
        self.0.iter().copied()
    }

    pub fn conflicts(&self, other: &Self) -> bool {
        !self.intersection(other).is_empty()
    }

    pub fn intersection(&self, other: &Self) -> Vec<Vertex> {
        let a = RightOfWay::from(self);
        let b = RightOfWay::from(other);
        (0..=self.duration().max(other.duration()))
            .tuple_windows()
            .filter_map(|(now, then)| {
                let a_now = a.at(now)?;
                let b_now = b.at(now)?;
                let a_then = a.at(then)?;
                let b_then = b.at(then)?;

                if a_now == b_now {
                    // Node conflict
                    return Some(vec![a_now]);
                }

                if a_now == b_then && a_then == b_now {
                    // Edge Conflict
                    return Some(vec![a_now, b_now]);
                }

                // No conflict
                None
            })
            .flatten()
            .collect()
    }

    /// Copy the last location in the route for `n` times, but increase its time
    pub fn stay(&mut self, n: usize) {
        if let Some(x) = self.0.back().cloned() {
            self.0.extend((0..=n).map(|t| Location {
                time: x.time + t,
                ..x
            }));
        }
    }

    pub fn pop(&mut self) -> Option<Location> {
        self.0.pop_front()
    }

    pub fn extend(&mut self, other: Route) {
        self.0.extend(other.0);
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }
}
