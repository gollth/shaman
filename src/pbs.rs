//! Priority based solving of MAPF problem
use std::collections::BinaryHeap;

use itertools::Itertools;
use miette::{Result, miette};
use petgraph::{acyclic::Acyclic, algo::toposort, data::Build, prelude::*};
use rustc_hash::FxHashMap;

use crate::{Params, Shaman, Time, astar::RightOfWay, layout::Layout, robot::Robot};

/// Main entry point for finding the best [Idea] for a MAPF problem
#[derive(Debug)]
pub struct Pbs {
    time: Time,
    params: Params,
    layout: Layout,
    queue: BinaryHeap<Idea>,
    horizon: char,
}

impl From<Shaman> for Pbs {
    fn from(value: Shaman) -> Self {
        let mut queue = BinaryHeap::new();
        queue.push(Idea {
            params: value.params,
            robots: value.robots,
            priorities: Acyclic::new(),
        });

        Self {
            time: value.time,
            params: value.params,
            layout: value.layout,
            horizon: value.horizon,
            queue,
        }
    }
}

impl Pbs {
    /// Solve the MAPD problem by:
    ///
    /// 1. Finding a collision between any pair of robots
    /// 2. Fixing one of the two and make the other use the first as [RightOfWay] constraint
    /// 3. Repeating 2. with both robots flipped
    pub fn solve(mut self) -> Result<Shaman> {
        let w = self.params.window;
        while let Some(idea) = self.queue.pop() {
            let Some((a, b)) = idea
                .robots
                .values()
                .tuple_combinations()
                .find(|(a, b)| a.route().take(w).conflicts(&b.route().take(w)))
                .map(|(a, b)| (a.name(), b.name()))
            else {
                // No more conflicts (=
                return Ok(Shaman {
                    time: self.time,
                    horizon: self.horizon,
                    params: self.params,
                    layout: self.layout,
                    robots: idea
                        .robots
                        .values()
                        .map(|r| (r.name(), r.clone()))
                        .collect(),
                });
            };

            for (boss, subordinate) in [(a, b), (b, a)] {
                if let Some(child) = idea.branch(&self.layout, boss, subordinate) {
                    self.queue.push(child);
                }
            }
        }

        Err(miette!("Ran out of ideas =("))
    }
}

/// A single possible solution to the overall MAPF problem
#[derive(Debug, Default, Clone)]
pub struct Idea {
    params: Params,
    priorities: Acyclic<StableDiGraph<char, ()>>,
    robots: FxHashMap<char, Robot>,
}

impl Ord for Idea {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.cost().cmp(&other.cost()).reverse() // for min heap
    }
}

impl PartialEq for Idea {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other).is_eq()
    }
}
impl PartialOrd for Idea {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Eq for Idea {}

impl Idea {
    fn cost(&self) -> usize {
        self.robots.values().map(|r| r.route().duration()).sum()
    }

    fn plan(&mut self, layout: &Layout) -> Result<()> {
        let order = toposort(&self.priorities, None)
            .expect("Cycle detected")
            .into_iter()
            .map(|n| self.priorities[n])
            .collect::<Vec<_>>();

        let mut constraints = RightOfWay::default();
        for n in &order {
            let robot = self.robots.get_mut(n).unwrap();

            robot.plan(layout, &constraints, self.params.window)?;
            constraints += robot.route().into();
        }

        Ok(())
    }

    fn find_or_create_node(&mut self, name: char) -> NodeIndex {
        self.priorities
            .node_indices()
            .find(|n| self.priorities[*n] == name)
            .unwrap_or_else(|| self.priorities.add_node(name))
    }

    fn branch(&self, layout: &Layout, boss: char, subordinate: char) -> Option<Self> {
        let mut child = self.clone();

        let b = child.find_or_create_node(boss);
        let s = child.find_or_create_node(subordinate);
        if child.priorities.contains_edge(b, s) {
            return None;
        }
        if child.priorities.try_add_edge(b, s, ()).is_err() {
            return None;
        }

        child.plan(layout).ok()?; // Plan would lead to deadlock

        Some(child)
    }
}
