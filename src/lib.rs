mod astar;
mod layout;
mod robot;
mod route;

use std::{collections::HashSet, fmt::Display, time::Duration};
use termion::{
    color::{Fg, Magenta},
    cursor,
    style::Reset,
};

use crate::{
    layout::{Layout, Vertex},
    robot::Robot,
};
use itertools::Itertools;
use termion::color::*;

pub type Time = usize;

/// Top level entry point for defining a layout & a list of robots
struct Shaman {
    robots: Vec<Robot>,
    layout: Layout,
}

impl Shaman {
    pub fn simulate(&mut self) {
        for robot in &mut self.robots {
            robot.simulate();
        }
    }
}

impl Display for Shaman {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "╭")?;
        for _ in 0..self.layout.width() {
            write!(f, "─")?;
        }

        let intersections = self
            .robots
            .iter()
            .tuple_combinations()
            .flat_map(|(a, b)| a.route().intersection(b.route()))
            .collect::<HashSet<_>>();
        writeln!(f, "╮")?;
        for y in 0..self.layout.height() {
            write!(f, "│")?;
            for x in 0..self.layout.width() {
                let v = Vertex::new(x as i32, y as i32);
                match self.robots.iter().find(|r| r.position() == v) {
                    Some(robot) => write!(f, "{robot}")?,
                    None => {
                        if intersections.contains(&v) {
                            write!(f, "{}✕{Reset}", Fg(Magenta))?;
                        } else if let Some(robot) = self
                            .robots
                            .iter()
                            .find(|r| r.route().iter().any(|n| n.position == v))
                        {
                            write!(f, "{}", robot.pathicon())?;
                        } else if self.layout.is_blocked(v) {
                            // Obstacle
                            write!(f, "█")?;
                        } else {
                            // Free space
                            write!(f, " ")?;
                        }
                    }
                }
            }
            writeln!(f, "│")?;
        }
        write!(f, "╰")?;
        for _ in 0..self.layout.width() {
            write!(f, "─")?;
        }
        writeln!(f, "╯")?;
        Ok(())
    }
}

pub fn level(fps: f32) -> anyhow::Result<()> {
    let mut sim = Shaman {
        layout: Layout::empty(20, 10),
        robots: vec![
            Robot::new(5, 0, Blue),
            Robot::new(8, 3, Red),
            Robot::new(19, 8, Green),
            Robot::new(0, 9, Yellow),
        ],
    };

    // walls
    sim.layout.obstacle(0..=12, 4..=5);
    sim.layout.obstacle(10..=11, 1..=5);
    sim.layout.obstacle(14..=19, 4..=5);

    sim.robots[0].plan(&sim.layout, Vertex::new(6, 9), &Default::default())?;
    let constraints = sim.robots[0].route().into();
    sim.robots[1].plan(&sim.layout, Vertex::new(19, 3), &constraints)?;
    sim.robots[2].plan(&sim.layout, Vertex::new(5, 0), &constraints)?;
    sim.robots[3].plan(&sim.layout, Vertex::new(6, 9), &constraints)?;

    if fps == 0. {
        println!("{sim}");
        return Ok(());
    }

    let dt = Duration::from_secs_f32(1. / fps);
    let steps = sim
        .robots
        .iter()
        .map(|r| r.route().len())
        .max()
        .unwrap_or_default();
    print!("{}", cursor::Hide);
    for _ in 0..steps {
        sim.simulate();
        print!(
            "{sim}{}{}",
            cursor::Left(sim.layout.width() as u16 + 2),
            cursor::Up(sim.layout.height() as u16 + 2)
        );
        std::thread::sleep(dt);
    }
    print!("{sim}{}", cursor::Show);
    Ok(())
}
