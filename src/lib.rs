mod astar;
mod layout;
mod parser;
mod pbs;
mod robot;
mod route;

use std::{fmt::Display, path::Path, time::Duration};
use termion::{
    color::{Fg, Magenta},
    cursor,
    style::Reset,
};

use crate::{
    layout::{Layout, Vertex},
    pbs::Pbs,
    robot::Robot,
};
use itertools::Itertools;
use miette::{NamedSource, Result, miette};
use rustc_hash::{FxHashMap, FxHashSet};

pub type Time = usize;

/// Top level entry point for defining a layout & a list of robots
#[derive(Debug)]
pub struct Shaman {
    robots: FxHashMap<char, Robot>,
    layout: Layout,
}

impl Shaman {
    pub fn parse<P: AsRef<Path>>(file: P) -> Result<Self> {
        let file = file.as_ref().display().to_string();
        let content = std::fs::read_to_string(&file).map_err(|e| miette!("{file}: {e}"))?;

        let mut sim: Shaman = parser::parse(&file, &content)?;
        for robot in sim.robots.values_mut() {
            robot.plan(&sim.layout, &Default::default())?;
        }
        Ok(sim)
    }

    fn new(code: NamedSource<String>, width: i32, height: i32) -> Self {
        Self {
            robots: Default::default(),
            layout: Layout::empty(code, width as usize, height as usize),
        }
    }

    pub fn simulate(&mut self) {
        for robot in self.robots.values_mut() {
            robot.simulate();
        }
    }

    pub fn solve(self) -> Result<Self> {
        Pbs::from(self).solve()
    }

    fn simulation_duration(&self) -> Time {
        self.robots
            .values()
            .map(|r| r.route().duration())
            .max()
            .unwrap_or_default()
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
            .values()
            .tuple_combinations()
            .flat_map(|(a, b)| a.route().intersection(b.route()))
            .collect::<FxHashSet<_>>();
        writeln!(f, "╮")?;
        for y in 0..self.layout.height() {
            write!(f, "│")?;
            for x in 0..self.layout.width() {
                let v = Vertex::new(x as i32, y as i32);
                match self.robots.values().find(|r| r.position().0 == v) {
                    Some(robot) => write!(f, "{robot}")?,
                    None => {
                        if intersections.contains(&v) {
                            write!(f, "{}✕{Reset}", Fg(Magenta))?;
                        } else if let Some(robot) = self
                            .robots
                            .values()
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

pub fn level(map: &Path, fps: f32, stop: bool) -> Result<()> {
    miette::set_hook(Box::new(|_| {
        Box::new(miette::MietteHandlerOpts::new().context_lines(10).build())
    }))?;

    let mut sim = Shaman::parse(map)?;
    if !stop {
        sim = sim.solve()?;
    }

    if fps == 0. {
        println!("{sim}");
        return Ok(());
    }

    let dt = Duration::from_secs_f32(1. / fps);
    print!("{}", cursor::Hide);
    for _ in 0..=sim.simulation_duration() {
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
