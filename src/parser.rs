use std::collections::HashMap;

use enum_as_inner::EnumAsInner;
use itertools::Itertools;
use miette::{NamedSource, Result};
use nom::{
    AsChar, Input, Parser,
    branch::alt,
    character::{
        anychar,
        complete::{char, newline, space0},
        one_of,
    },
    combinator::eof,
    error::ParseError,
    multi::{many_till, separated_list0, separated_list1},
    sequence::terminated,
};
use nom_locate::{LocatedSpan, position};

use crate::{Shaman, Time, error::ShamanError, layout::Vertex, robot::Robot};

type Span<'a> = LocatedSpan<&'a str>;
type IResult<'a, T> = nom::IResult<Span<'a>, T>;

pub(crate) fn parse(filename: &str, s: &str) -> Result<Shaman, ShamanError> {
    let src = NamedSource::new(filename, s.to_string());

    let (_, mut scenario) = scenario(Span::new(s)).map_err(|e| match e {
        nom::Err::Incomplete(more) => panic!("Failed to parse map, expected more input: {more:?}"),
        nom::Err::Error(e) => ShamanError::InvalidCell {
            src: src.clone(),
            highlight: (e.input.location_offset(), 1).into(),
        },
        nom::Err::Failure(e) => panic!("Failed to parse map: {e}"),
    })?;

    let grid = scenario
        .grid
        .into_iter()
        .filter(|row| !row.is_empty())
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

    shaman
        .robots
        .extend(robots.into_iter().map(|((x, y), s, n)| {
            (
                n,
                Robot::new(n, x, y, (s.location_offset(), 1).into(), Time::default()),
            )
        }));

    for robot in shaman.robots.keys() {
        let default_goal = robot.to_ascii_lowercase();
        scenario
            .definitions
            .entry(*robot)
            .and_modify(|defs| {
                if defs.is_empty() {
                    defs.push(default_goal)
                }
            })
            .or_insert(vec![default_goal]);
    }

    for v in grid
        .iter()
        .filter(|(_, cell)| cell.inner.is_obstacle())
        .map(|((x, y), _)| Vertex::new(*x, *y))
    {
        shaman.layout.block(v);
    }

    let labels = grid
        .into_iter()
        .filter_map(|((x, y), Spanned { span, inner })| {
            Some((*inner.as_label()?, (span, Vertex::new(x, y))))
        })
        .collect::<HashMap<_, _>>();

    if let Some(highlight) = labels
        .iter()
        .find(|(l, _)| !scenario.definitions.values().flatten().contains(*l))
        .map(|(_, (span, _))| (span.location_offset(), 1).into())
    {
        return Err(ShamanError::InvalidCell {
            src: src.clone(),
            highlight,
        });
    }

    for robot in shaman.robots.values_mut() {
        let Some(def) = scenario.definitions.get(&robot.name()) else {
            continue;
        };

        for (span, goal) in def
            .iter()
            .map(|l| labels.get(l).expect("defined a label which is never used"))
            .copied()
        {
            robot.add_goal(goal, (span.location_offset(), 1).into());
        }
    }

    Ok(shaman)
}

#[derive(Debug)]
struct Spanned<'a, T> {
    span: Span<'a>,
    inner: T,
}

#[derive(Debug)]
struct Scenario<'a> {
    definitions: HashMap<char, Vec<char>>,
    grid: Vec<Vec<Spanned<'a, Cell>>>,
}

#[derive(Debug, Clone, Copy, EnumAsInner)]
enum Cell {
    Free,
    Robot(char),
    Label(char),
    Obstacle,
}

fn scenario(s: Span) -> IResult<Scenario> {
    (definitions, grid)
        .map(|(definitions, grid)| Scenario { grid, definitions })
        .parse(s)
}

fn grid(s: Span) -> IResult<Vec<Vec<Spanned<Cell>>>> {
    many_till(many_till(cell, newline).map(ignore_delim()), eof)
        .map(ignore_delim())
        .parse(s)
}

fn definitions(s: Span) -> IResult<HashMap<char, Vec<char>>> {
    let (s, defs) = separated_list0(
        newline,
        (
            spaced(one_of("ABCDEFGHIJKLMNOPQRSTUVWXYZ")),
            spaced(char('=')),
            spaced(char('[')),
            separated_list1(spaced(char(',')), anychar),
            spaced(char(']')),
        )
            .map(|(robot, _, _, goals, _)| (robot, goals)),
    )
    .parse(s)?;
    Ok((s, defs.into_iter().collect()))
}

fn cell(s: Span) -> IResult<Spanned<Cell>> {
    let (s, span) = position(s)?;
    let (s, cell) = alt((
        char(' ').map(always(Cell::Free)),
        char('#').or(char('█')).map(always(Cell::Obstacle)),
        one_of("ABCDEFGHIJKLMNOPQRSTUVWXYZ").map(Cell::Robot),
        anychar.map(Cell::Label),
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

fn spaced<I, O, E, P>(inner: P) -> impl Parser<I, Output = O, Error = E>
where
    I: Input,
    I::Item: AsChar,
    E: ParseError<I>,
    P: Parser<I, Output = O, Error = E>,
{
    terminated(inner, space0)
}
