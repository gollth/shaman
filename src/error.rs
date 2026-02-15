#![allow(unused)] // false positive for ParseError struct

use miette::{Diagnostic, NamedSource, SourceSpan};
use thiserror::Error;

#[derive(Error, Debug, Diagnostic)]
pub enum ShamanError {
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

    #[error("Only one goal per robot")]
    DuplicateGoals {
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

    #[error("No route found")]
    RouteNotFound {
        #[source_code]
        src: NamedSource<String>,
        #[label("from here")]
        start: SourceSpan,
        #[label("to here")]
        goal: SourceSpan,
    },
}
