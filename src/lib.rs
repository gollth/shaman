mod astar;
mod error;
mod layout;
mod parser;
mod pbs;
mod robot;
mod route;

use derivative::Derivative;
use std::{fmt::Display, iter::repeat, path::Path, time::Duration};
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
    time: Time,
    params: Params,
    robots: FxHashMap<char, Robot>,
    layout: Layout,
    horizon: char,
}

impl Shaman {
    pub fn parse<P: AsRef<Path>>(file: P) -> Result<Self> {
        let file = file.as_ref().display().to_string();
        let content = std::fs::read_to_string(&file).map_err(|e| miette!("{file}: {e}"))?;

        let mut sim: Shaman = parser::parse(&file, &content)?;
        for robot in sim.robots.values_mut() {
            robot.plan(&sim.layout, &Default::default(), Params::default().window)?;
        }
        Ok(sim)
    }

    fn new(code: NamedSource<String>, width: i32, height: i32) -> Self {
        Self {
            time: Time::default(),
            horizon: DEFAULT_HORIZON,
            params: Params::default(),
            robots: Default::default(),
            layout: Layout::empty(code, width as usize, height as usize),
        }
    }

    pub fn with_params(mut self, params: Params) -> Self {
        self.params = params;
        self
    }

    pub fn with_horizon_char(mut self, c: char) -> Self {
        self.horizon = c;
        self
    }

    fn time_to_replan(&self) -> bool {
        self.time.is_multiple_of(self.params.replan)
    }

    pub fn simulate(&mut self) {
        for robot in self.robots.values_mut() {
            robot.simulate();
        }
    }

    pub fn solve(self) -> Result<Self> {
        Pbs::from(self).solve()
    }

    pub fn finished(&self) -> bool {
        self.robots.values().all(|r| r.is_idle() && !r.has_goals())
    }
}

impl Display for Shaman {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "╭")?;
        let title = if self.time_to_replan() {
            format!("⏱t={} ", self.time)
        } else {
            format!(" t={} ", self.time)
        };
        write!(f, "{:─^1$}", title, self.layout.width())?;

        let intersections = self
            .robots
            .values()
            .tuple_combinations()
            .flat_map(|(a, b)| a.route().intersection(b.route()))
            .collect::<FxHashSet<_>>();
        let horizons = self
            .robots
            .values()
            .flat_map(|r| {
                r.route()
                    .iter()
                    .take(self.params.window)
                    .map(|l| l.position)
                    .zip(repeat(r.color().to_owned()))
            })
            .collect::<FxHashMap<_, _>>();
        let routes = self
            .robots
            .values()
            .flat_map(|r| {
                r.route()
                    .iter()
                    .map(|l| l.position)
                    .zip(repeat(r.color().to_owned()))
            })
            .collect::<FxHashMap<_, _>>();

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
                        } else if let Some((color, goal)) = self
                            .robots
                            .values()
                            .find_map(|r| Some((r.color(), r.goals().position(|g| g == v)? + 1)))
                        {
                            write!(f, "{color}{goal}{Reset}")?;
                        } else if let Some(color) = horizons.get(&v) {
                            write!(f, "{color}{}{Reset}", self.horizon)?;
                        } else if let Some(color) = routes.get(&v) {
                            write!(f, "{color}·{Reset}")?;
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

pub const DEFAULT_REPLAN: usize = 5;
pub const DEFAULT_WINDOW: usize = 5;
pub const DEFAULT_HORIZON: char = '⚬';

#[derive(Debug, Clone, Copy, PartialEq, Eq, Derivative)]
#[derivative(Default)]
pub struct Params {
    #[derivative(Default(value = "DEFAULT_WINDOW"))]
    pub window: usize,
    #[derivative(Default(value = "DEFAULT_REPLAN"))]
    pub replan: usize,
}

pub fn level(
    map: &Path,
    params: Params,
    fps: f32,
    stop: bool,
    keep: bool,
    horizon: char,
) -> Result<()> {
    miette::set_hook(Box::new(|_| {
        Box::new(miette::MietteHandlerOpts::new().context_lines(10).build())
    }))?;

    let mut sim = Shaman::parse(map)?
        .with_params(params)
        .with_horizon_char(horizon);
    if stop {
        println!("{sim}");
        return Ok(());
    }

    if fps == 0. {
        sim = sim.solve()?;
        println!("{sim}");
        return Ok(());
    }

    let dt = Duration::from_secs_f32(1. / fps);
    print!("{}", cursor::Hide);
    while !sim.finished() {
        if sim.time_to_replan() {
            sim = sim.solve()?;
        }
        sim.simulate();
        sim.time += 1;

        print!("{sim}");
        if !keep {
            print!(
                "{}{}",
                cursor::Left(sim.layout.width() as u16 + 2),
                cursor::Up(sim.layout.height() as u16 + 2)
            );
        } else {
            println!();
        }
        std::thread::sleep(dt);
    }
    print!("{sim}{}", cursor::Show);
    Ok(())
}
