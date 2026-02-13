use itertools::Itertools;
use std::{collections::HashSet, ops::Add};

/// The definition of the 2D grid space, with free & blocked cells
#[derive(Debug)]
pub struct Layout {
    space: HashSet<Vertex>,
    width: usize,
    height: usize,
}

/// Position of each cell in the layout
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
pub struct Vertex {
    x: i32,
    y: i32,
}

impl Vertex {
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    pub fn distance_squared(&self, other: Self) -> f32 {
        ((self.x - other.x) as f32).powi(2) + ((self.y - other.y) as f32).powi(2)
    }
}

impl Add for Vertex {
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Layout {
    pub fn empty(width: usize, height: usize) -> Self {
        Self {
            space: (0..width)
                .cartesian_product(0..height)
                .map(|(x, y)| Vertex::new(x as i32, y as i32))
                .collect(),
            width,
            height,
        }
    }

    /// Amount of rows of this layout
    pub(crate) fn height(&self) -> usize {
        self.height
    }

    /// Amount of columns of this layout
    pub(crate) fn width(&self) -> usize {
        self.width
    }

    /// Add a rectangular obstacle into this layout
    pub fn obstacle<W, H>(&mut self, width: W, height: H)
    where
        W: Iterator<Item = i32> + Clone,
        H: Iterator<Item = i32> + Clone,
    {
        for vertex in width
            .cartesian_product(height)
            .map(|(x, y)| Vertex::new(x, y))
        {
            self.block(vertex);
        }
    }

    /// Mark a single [Vertex] of this layout as obstacle
    pub(crate) fn block(&mut self, v: Vertex) -> bool {
        self.space.remove(&v)
    }

    pub(crate) fn is_blocked(&self, v: Vertex) -> bool {
        !self.space.contains(&v)
    }
}
