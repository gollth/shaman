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

    pub fn pop(&mut self) -> Option<Location> {
        self.0.pop_front()
    }
}
