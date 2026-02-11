use std::fmt::Display;

use clap::Parser;
use grid::Grid;
use termion::{
    color::{Blue, Fg, Green, Red, Yellow},
    style::Reset,
};

#[derive(Debug, Parser)]
struct Args {
    #[arg(long, default_value_t = 20)]
    width: usize,

    #[arg(long, default_value_t = 10)]
    height: usize,
}

#[derive(Debug, Clone, Copy)]
enum Robot {
    A,
    B,
    C,
    D,
}

impl Display for Robot {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::A => write!(f, "{}●{Reset}", Fg(Blue)),
            Self::B => write!(f, "{}●{Reset}", Fg(Red)),
            Self::C => write!(f, "{}●{Reset}", Fg(Yellow)),
            Self::D => write!(f, "{}●{Reset}", Fg(Green)),
        }
    }
}

#[derive(Debug)]
struct Layout {
    grid: Grid<Cell>,
}

impl Layout {
    fn new(width: usize, height: usize) -> Self {
        Self {
            grid: Grid::new(height, width),
        }
    }
}

impl Display for Layout {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "╭")?;
        for _ in 0..self.grid.cols() {
            write!(f, "─")?;
        }
        writeln!(f, "╮")?;
        for row in self.grid.iter_rows() {
            write!(f, "│")?;
            for cell in row {
                match cell {
                    Cell::Empty => write!(f, " ")?,
                    Cell::Occupied(robot) => write!(f, "{robot}")?,
                }
            }
            writeln!(f, "│")?;
        }
        write!(f, "╰")?;
        for _ in 0..self.grid.cols() {
            write!(f, "─")?;
        }
        writeln!(f, "╯")?;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Default)]
enum Cell {
    #[default]
    Empty,
    Occupied(Robot),
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let mut layout = Layout::new(args.width, args.height);

    layout.grid[(4, 0)] = Cell::Occupied(Robot::A);
    layout.grid[(4, args.width - 1)] = Cell::Occupied(Robot::B);
    layout.grid[(0, 10)] = Cell::Occupied(Robot::C);
    layout.grid[(args.height - 1, 10)] = Cell::Occupied(Robot::D);

    print!("{layout}");
    Ok(())
}
