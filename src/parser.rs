use std::str::FromStr;

use anyhow::{Context, anyhow};
use enum_as_inner::EnumAsInner;
use itertools::Itertools;
use nom::{
    Finish, IResult, Parser,
    branch::alt,
    bytes::complete::tag,
    character::{complete::newline, one_of},
    combinator::eof,
    multi::many_till,
};

use crate::{Shaman, layout::Vertex, robot::Robot};

impl FromStr for Shaman {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // TODO: Better error messages with expected tokens
        let (_, grid) = grid(s).finish().map_err(|e| anyhow!("{e}"))?;

        let grid = grid
            .into_iter()
            .enumerate()
            .flat_map(|(y, row)| {
                row.into_iter()
                    .enumerate()
                    .map(move |(x, c)| ((x as i32, y as i32), c))
            })
            .collect::<Vec<_>>();

        let mut shaman = Shaman::new(
            grid.iter().map(|((x, _), _)| *x).max().unwrap_or_default() + 1,
            grid.iter().map(|((_, y), _)| *y).max().unwrap_or_default() + 1,
        );

        let robots = grid
            .iter()
            .filter_map(|(c, cell)| Some((*c, cell.into_robot().ok()?)))
            .sorted_by_key(|(_, n)| *n)
            .collect::<Vec<_>>();

        let dups = robots
            .iter()
            .map(|(_, n)| *n)
            .duplicates()
            .collect::<Vec<_>>();
        anyhow::ensure!(
            dups.is_empty(),
            "Each robot can only have one start position, but {dups:?} are defined multiple times"
        );
        anyhow::ensure!(
            robots.len() <= 4,
            "Maximum of 4 robots allowed, but you defined {}",
            robots.len()
        );

        shaman
            .robots
            .extend(robots.into_iter().map(|((x, y), n)| Robot::new(n, x, y)));

        for v in grid
            .iter()
            .filter(|(_, cell)| cell.is_obstacle())
            .map(|((x, y), _)| Vertex::new(*x, *y))
        {
            shaman.layout.block(v);
        }

        for ((x, y), cell) in grid {
            match cell {
                Cell::Goal(c) => {
                    let index = (c as u8) - ('A' as u8);
                    shaman.robots.get_mut(index as usize).ok_or(anyhow!(
                        "Cannot create route '{}' because no robot '{c}' was defined in the layout",
                        c.to_ascii_lowercase()
                    ))?.set_goal(Vertex::new(x,y)).context(format!("Robot {c}"))?;
                }
                _ => {}
            }
        }

        Ok(shaman)
    }
}

#[derive(Debug, Clone, Copy, EnumAsInner)]
enum Cell {
    Free,
    Robot(char),
    Goal(char),
    Obstacle,
}

fn grid(s: &str) -> IResult<&str, Vec<Vec<Cell>>> {
    many_till(row, eof).map(ignore_delim()).parse(s)
}

fn row(s: &str) -> IResult<&str, Vec<Cell>> {
    many_till(cell, newline).map(ignore_delim()).parse(s)
}

fn cell(s: &str) -> IResult<&str, Cell> {
    alt((free, obstacle, robot, goal)).parse(s)
}

fn obstacle(s: &str) -> IResult<&str, Cell> {
    alt((tag("#"), tag("█")))
        .map(always(Cell::Obstacle))
        .parse(s)
}

fn free(s: &str) -> IResult<&str, Cell> {
    tag(" ").map(always(Cell::Free)).parse(s)
}

fn robot(s: &str) -> IResult<&str, Cell> {
    one_of("ABCD").map(|c| Cell::Robot(c)).parse(s)
}

fn goal(s: &str) -> IResult<&str, Cell> {
    one_of("abcd")
        .map(|c| Cell::Goal(c.to_ascii_uppercase()))
        .parse(s)
}

pub fn always<A: Copy, B>(x: A) -> impl Fn(B) -> A {
    move |_| x
}

pub fn ignore_delim<A, B>() -> impl Fn((A, B)) -> A {
    |(a, _)| a
}
