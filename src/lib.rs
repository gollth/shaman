mod astar;
mod layout;
mod parser;
mod robot;
mod route;

use std::{collections::HashSet, fmt::Display, path::Path, time::Duration};
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
use miette::{Result, miette};

pub type Time = usize;

/// Top level entry point for defining a layout & a list of robots
#[derive(Debug)]
struct Shaman {
    robots: Vec<Robot>,
    layout: Layout,
}

impl Shaman {
    fn parse<P: AsRef<Path>>(file: P) -> Result<Self> {
        let file = file.as_ref().display().to_string();
        let content = std::fs::read_to_string(&file).map_err(|e| miette!("{file}: {e}"))?;
        parser::parse(&file, &content)
    }

    fn new(width: i32, height: i32) -> Self {
        Self {
            robots: Default::default(),
            layout: Layout::empty(width as usize, height as usize),
        }
    }

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

pub fn level(map: &Path, fps: f32) -> Result<()> {
    miette::set_hook(Box::new(|_| {
        Box::new(miette::MietteHandlerOpts::new().context_lines(10).build())
    }))?;

    let mut sim = Shaman::parse(map)?;

    sim.robots[0].plan(&sim.layout, &Default::default())?;
    let constraints = sim.robots[0].route().into();
    sim.robots[1].plan(&sim.layout, &constraints)?;
    sim.robots[2].plan(&sim.layout, &constraints)?;
    sim.robots[3].plan(&sim.layout, &constraints)?;

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
