use rustc_hash::FxHashSet;
use std::collections::VecDeque;

use itertools::Itertools;

use crate::{Time, layout::Vertex, robot::Location};

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
        let a = self.0.iter().cloned().collect::<FxHashSet<_>>();
        let b = other.0.iter().cloned().collect::<FxHashSet<_>>();
        let mut intersection = a.intersection(&b).map(|l| l.position).collect::<Vec<_>>();
        intersection.extend(
            self.0
                .back()
                .zip(other.0.back())
                .filter(|(a, b)| a.position == b.position)
                .map(|(a, _)| a.position),
        );

        intersection.extend(
            self.0
                .iter()
                .tuple_windows()
                .filter(|(now, then)| {
                    other
                        .0
                        .iter()
                        .tuple_windows()
                        .find(|(a, _)| a.time == now.time)
                        .is_some_and(|(a, b)| {
                            b.position == now.position && a.position == then.position
                        })
                })
                .flat_map(|(a, b)| vec![a.position, b.position]),
        );
        intersection
    }

    pub fn pop(&mut self) -> Option<Location> {
        self.0.pop_front()
    }
}
