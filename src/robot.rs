use miette::SourceSpan;
use std::fmt::Display;

use termion::{
    color::{Fg, Rgb},
    style::Reset,
};

use crate::{
    Time,
    astar::{self, RightOfWay},
    error::ShamanError,
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
    name: char,
    color: String,
    position: (Vertex, SourceSpan),
    route: Route,
    goals: Vec<(Vertex, SourceSpan)>,
}

impl Robot {
    pub fn new(name: char, x: i32, y: i32, span: SourceSpan) -> Self {
        let color = match name {
            'A' => Rgb(0, 0, 255),
            'B' => Rgb(255, 0, 0),
            'C' => Rgb(0, 255, 0),
            'D' => Rgb(255, 255, 0),
            'E' => Rgb(0, 255, 255),
            _ => panic!(),
        };
        Self {
            name,
            color: format!("{}", Fg(color)),
            position: (Vertex::new(x, y), span),
            route: Default::default(),
            goals: Default::default(),
        }
    }

    pub fn name(&self) -> char {
        self.name
    }

    pub fn position(&self) -> (Vertex, SourceSpan) {
        self.position
    }

    pub fn add_goal(&mut self, v: Vertex, span: SourceSpan) {
        self.goals.push((v, span));
    }

    pub fn goals(&self) -> impl Iterator<Item = Vertex> {
        self.goals.iter().map(|g| g.0)
    }

    pub fn route(&self) -> &Route {
        &self.route
    }

    pub fn color(&self) -> &str {
        &self.color
    }

    pub fn pathicon(&self) -> String {
        format!("{}·{Reset}", self.color())
    }

    pub(crate) fn simulate(&mut self) {
        let Some(next) = self.route.pop() else {
            return;
        };

        self.position.0 = next.position;
    }

    pub(crate) fn plan(
        &mut self,
        layout: &Layout,
        constraint: &RightOfWay,
    ) -> Result<(), ShamanError> {
        let (position, mut source_span) = self.position();
        let mut location = Location { position, time: 0 };

        self.route.clear();
        for (goal, goal_span) in self.goals.iter().cloned() {
            self.route.extend(astar::solve(
                layout,
                (location, source_span),
                (goal, goal_span),
                constraint,
            )?);
            self.route.stay(3); // simulate load handling
            location.time = self.route.duration() + 1;
            location.position = goal;
            source_span = goal_span;
        }
        Ok(())
    }
}

impl Display for Robot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}●{Reset}", self.color)
    }
}
