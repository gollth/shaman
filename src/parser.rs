use enum_as_inner::EnumAsInner;
use itertools::Itertools;
use miette::{NamedSource, Result};
use nom::{
    Parser,
    branch::alt,
    character::complete::{char, newline},
    combinator::eof,
    multi::many_till,
};
use nom_locate::{LocatedSpan, position};

use crate::{Shaman, error::ShamanError, layout::Vertex, robot::Robot};

type Span<'a> = LocatedSpan<&'a str>;
type IResult<'a, T> = nom::IResult<Span<'a>, T>;

pub(crate) fn parse(filename: &str, s: &str) -> Result<Shaman, ShamanError> {
    let src = NamedSource::new(filename, s.to_string());

    let (_, grid) = grid.parse(Span::new(s)).map_err(|e| match e {
        nom::Err::Incomplete(more) => panic!("Failed to parse map, expected more input: {more:?}"),
        nom::Err::Error(e) => ShamanError::InvalidCell {
            src: src.clone(),
            highlight: (e.input.location_offset(), 1).into(),
        },
        nom::Err::Failure(e) => panic!("Failed to parse map: {e}"),
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
        src.clone(),
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
        return Err(ShamanError::DuplicateRobots {
            src: src.clone(),
            a: (a.location_offset(), 1).into(),
            b: (b.location_offset(), 1).into(),
        });
    }

    shaman.robots.extend(
        robots
            .into_iter()
            .map(|((x, y), s, n)| (n, Robot::new(n, x, y, (s.location_offset(), 1).into()))),
    );

    for v in grid
        .iter()
        .filter(|(_, cell)| cell.inner.is_obstacle())
        .map(|((x, y), _)| Vertex::new(*x, *y))
    {
        shaman.layout.block(v);
    }

    for ((x, y), Spanned { span, inner }) in grid {
        let (n, goal) = match inner {
            Cell::Goal(n) => (n, Vertex::new(x, y)),
            Cell::GoalSouth(n) => (n, Vertex::new(x, y + 1)),
            _ => continue,
        };
        shaman
            .robots
            .get_mut(&n)
            .ok_or(ShamanError::NoRobotForGoal {
                src: src.clone(),
                robot: n,
                goal: (span.location_offset(), 1).into(),
            })?
            .set_goal(&shaman.layout, goal, (span.location_offset(), 1).into())?
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
    GoalSouth(char),
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
            .map(Cell::Robot),
        char('a')
            .or(char('b'))
            .or(char('c'))
            .or(char('d'))
            .map(|c| c.to_ascii_uppercase())
            .map(Cell::Goal),
        char('ⓐ')
            .or(char('ⓑ'))
            .or(char('ⓒ'))
            .or(char('ⓓ'))
            .map(|c| ((c as u32 - 0x24D0 + 0x41) as u8) as char)
            .map(Cell::GoalSouth),
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
