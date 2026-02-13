use std::str::FromStr;

use enum_as_inner::EnumAsInner;
use eyre::{WrapErr, eyre};
use itertools::Itertools;
use nom::{
    Parser,
    branch::alt,
    bytes::complete::tag,
    character::complete::{newline, one_of},
    combinator::eof,
    multi::many_till,
};

use crate::{Shaman, layout::Vertex, robot::Robot};

pub type Span<'a> = nom_locate::LocatedSpan<&'a str>;
pub type IResult<'a, O> = nom::IResult<Span<'a>, O>;

impl FromStr for Shaman {
    type Err = eyre::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // TODO: Better error messages with expected tokens

        let (_, grid) = grid.parse(Span::new(s)).map_err(|e| match e {
            nom::Err::Incomplete(needed) => eyre!("{needed:?}"),
            nom::Err::Error(e) => eyre!("{e}"),
            nom::Err::Failure(_) => todo!(),
        })?;

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
        eyre::ensure!(
            dups.is_empty(),
            "Each robot can only have one start position, but {dups:?} are defined multiple times"
        );
        eyre::ensure!(
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
                    shaman.robots.get_mut(index as usize).ok_or(eyre!(
                        "Cannot create route '{}' because no robot '{c}' was defined in the layout",
                        c.to_ascii_lowercase()
                    ))?.set_goal(Vertex::new(x,y)).wrap_err(format!("Robot {c}"))?;
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

fn grid(s: Span) -> IResult<Vec<Vec<Cell>>> {
    many_till(row, eof).map(ignore_delim()).parse(s)
}

fn row(s: Span) -> IResult<Vec<Cell>> {
    many_till(cell, newline).map(ignore_delim()).parse(s)
}

fn cell(s: Span) -> IResult<Cell> {
    alt((free, obstacle, robot, goal)).parse(s)
}

fn obstacle(s: Span) -> IResult<Cell> {
    alt((tag("#"), tag("█")))
        .map(always(Cell::Obstacle))
        .parse(s)
}

fn free(s: Span) -> IResult<Cell> {
    tag(" ").map(always(Cell::Free)).parse(s)
}

fn robot(s: Span) -> IResult<Cell> {
    one_of("ABCD").map(|c| Cell::Robot(c)).parse(s)
}

fn goal(s: Span) -> IResult<Cell> {
    one_of("abcd")
        .map(|c: char| Cell::Goal(c.to_ascii_uppercase()))
        .parse(s)
}

pub fn always<A: Copy, B>(x: A) -> impl Fn(B) -> A {
    move |_| x
}

pub fn ignore_delim<A, B>() -> impl Fn((A, B)) -> A {
    |(a, _)| a
}
