use std::collections::{HashSet, VecDeque};

use crate::{layout::Vertex, robot::Location};

#[derive(Debug, Clone, Default)]
pub struct Route(VecDeque<Location>);

impl FromIterator<Location> for Route {
    fn from_iter<T: IntoIterator<Item = Location>>(iter: T) -> Self {
        Self(iter.into_iter().collect())
    }
}

impl Route {
    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn iter(&self) -> impl DoubleEndedIterator<Item = Location> {
        self.0.iter().copied()
    }

    pub fn intersection(&self, other: &Self) -> Vec<Vertex> {
        let a = self.0.iter().cloned().collect::<HashSet<_>>();
        let b = other.0.iter().cloned().collect::<HashSet<_>>();
        a.intersection(&b)
            .map(|l| l.position)
            .chain(
                self.0
                    .back()
                    .zip(other.0.back())
                    .filter(|(a, b)| a.position == b.position)
                    .map(|(a, _)| a.position),
            )
            .collect()
    }

    pub fn pop(&mut self) -> Option<Location> {
        self.0.pop_front()
    }
}
