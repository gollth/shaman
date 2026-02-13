use std::fmt::Display;

use termion::{
    color::{Color, Fg},
    style::Reset,
};

use crate::{
    Time,
    astar::RightOfWay,
    layout::{Layout, Vertex},
    route::Route,
};

/// Position of a robot at a specific point in time
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub struct Location {
    pub position: Vertex,
    pub time: Time,
}

#[derive(Debug, Clone)]
pub struct Robot {
    color: String,
    position: Vertex,
    route: Route,
}

impl Robot {
    pub fn new<C: Color>(x: i32, y: i32, color: C) -> Self {
        Self {
            color: format!("{}", Fg(color)),
            position: Vertex::new(x, y),
            route: Default::default(),
        }
    }

    pub fn position(&self) -> Vertex {
        self.position
    }

    pub fn route(&self) -> &Route {
        &self.route
    }

    pub fn pathicon(&self) -> String {
        format!("{}·{Reset}", self.color)
    }

    pub(crate) fn simulate(&mut self) {
        let Some(next) = self.route.pop() else {
            return;
        };

        self.position = next.position;
    }

    pub(crate) fn plan(
        &mut self,
        layout: &Layout,
        goal: Vertex,
        constraints: &RightOfWay,
    ) -> anyhow::Result<()> {
        self.route = crate::astar::solve(layout, self.position(), goal, constraints)?;
        Ok(())
    }
}

impl Display for Robot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}●{Reset}", self.color)
    }
}
