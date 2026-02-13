#![allow(unused)] // false positive for ParseError struct

use enum_as_inner::EnumAsInner;
use itertools::Itertools;
use miette::{Diagnostic, NamedSource, Result, SourceSpan, WrapErr, ensure, miette};
use nom::{
    Parser,
    branch::alt,
    character::complete::{char, newline},
    combinator::eof,
    multi::many_till,
};
use nom_locate::{LocatedSpan, position};
use thiserror::Error;

use crate::{Shaman, layout::Vertex, robot::Robot};

type Span<'a> = LocatedSpan<&'a str>;
type IResult<'a, T> = nom::IResult<Span<'a>, T>;

#[derive(Error, Debug, Diagnostic)]
pub enum ParseError {
    #[error(
        "Expected either an obstacle (# or █), a free cell (space), a robot (A..D) or a goal (a..d)"
    )]
    InvalidCell {
        #[source_code]
        src: NamedSource<String>,
        #[label("here")]
        highlight: SourceSpan,
    },

    #[error("Robot names must be unique")]
    DuplicateRobots {
        #[source_code]
        src: NamedSource<String>,
        #[label("first")]
        a: SourceSpan,
        #[label("second")]
        b: SourceSpan,
    },

    #[error("No robot named '{robot}' defined")]
    NoRobotForGoal {
        #[source_code]
        src: NamedSource<String>,
        robot: char,
        #[label("for this goal")]
        goal: SourceSpan,
    },
}

pub(crate) fn parse(filename: &str, s: &str) -> Result<Shaman> {
    let src = NamedSource::new(filename, s.to_string());

    let (_, grid) = grid.parse(Span::new(s)).map_err(|e| match e {
        nom::Err::Incomplete(more) => miette!("Failed to parse map, expected more input: {more:?}"),
        nom::Err::Error(e) => ParseError::InvalidCell {
            src: src.clone(),
            highlight: (e.input.location_offset(), 1).into(),
        }
        .into(),
        nom::Err::Failure(e) => miette!("Failed to parse map: {e}"),
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
        .filter_map(|(c, cell)| Some((*c, cell.span, cell.inner.into_robot().ok()?)))
        .sorted_by_key(|(_, _, n)| *n)
        .collect::<Vec<_>>();

    let dups = robots
        .iter()
        .cloned()
        .map(|(_, span, n)| (n, span))
        .into_group_map();

    if let Some((b, a)) = dups
        .into_iter()
        .sorted_by_key(|(n, _)| *n)
        .filter_map(|(_, mut ds)| Some((ds.pop()?, ds.pop()?)))
        .next()
    {
        return Err(ParseError::DuplicateRobots {
            src: src.clone(),
            a: (a.location_offset(), 1).into(),
            b: (b.location_offset(), 1).into(),
        }
        .into());
    }

    ensure!(
        robots.len() <= 4,
        "Maximum of 4 robots allowed, but you defined {}",
        robots.len()
    );

    shaman.robots.extend(
        robots
            .into_iter()
            .map(|((x, y), _, n)| (n, Robot::new(n, x, y))),
    );

    for v in grid
        .iter()
        .filter(|(_, cell)| cell.inner.is_obstacle())
        .map(|((x, y), _)| Vertex::new(*x, *y))
    {
        shaman.layout.block(v);
    }

    for ((x, y), cell) in grid {
        match cell {
            Spanned {
                span,
                inner: Cell::Goal(n),
            } => {
                shaman
                    .robots
                    .get_mut(&n)
                    .ok_or(ParseError::NoRobotForGoal {
                        src: src.clone(),
                        robot: n,
                        goal: (span.location_offset(), 1).into(),
                    })?
                    .set_goal(Vertex::new(x, y))
                    .wrap_err(format!("Robot {n}"))?;
            }
            _ => {}
        }
    }

    Ok(shaman)
}

struct Spanned<'a, T> {
    span: Span<'a>,
    inner: T,
}

#[derive(Debug, Clone, Copy, EnumAsInner)]
enum Cell {
    Free,
    Robot(char),
    Goal(char),
    Obstacle,
}

fn grid(s: Span) -> IResult<Vec<Vec<Spanned<Cell>>>> {
    many_till(many_till(cell, newline).map(ignore_delim()), eof)
        .map(ignore_delim())
        .parse(s)
}

fn cell(s: Span) -> IResult<Spanned<Cell>> {
    let (s, span) = position(s)?;
    let (s, cell) = alt((
        char(' ').map(always(Cell::Free)),
        char('#').or(char('█')).map(always(Cell::Obstacle)),
        char('A')
            .or(char('B'))
            .or(char('C'))
            .or(char('D'))
            .map(|c| Cell::Robot(c)),
        char('a')
            .or(char('b'))
            .or(char('c'))
            .or(char('d'))
            .map(|c: char| Cell::Goal(c.to_ascii_uppercase())),
    ))
    .parse(s)?;
    Ok((s, Spanned { span, inner: cell }))
}

pub fn always<A: Copy, B>(x: A) -> impl Fn(B) -> A {
    move |_| x
}

pub fn ignore_delim<A, B>() -> impl Fn((A, B)) -> A {
    |(a, _)| a
}
