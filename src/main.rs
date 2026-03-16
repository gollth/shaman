use std::path::PathBuf;

use clap::Parser;
use miette::{Result, miette};

#[derive(Debug, Parser)]
struct Args {
    /// How fast to simulate
    #[arg(short, long, default_value_t = 0.)]
    fps: f32,

    /// Don't solve right away, but print the conflicted solution
    #[arg(short('x'))]
    stop: bool,

    /// Don't clear the screen after each frame. Useful for debugging individual steps
    #[arg(short('k'))]
    keep: bool,

    /// Path to a map file to use
    map: PathBuf,

    /// Replanning peroid in timesteps
    #[arg(short('H'), default_value_t = shaman::DEFAULT_REPLAN)]
    replan: usize,

    /// Window of paths, where no conflict should happen in timesteps
    #[arg(short('W'), default_value_t = shaman::DEFAULT_WINDOW)]
    window: usize,
}

fn main() -> Result<()> {
    let args = Args::parse();
    if args.replan > args.window {
        return Err(miette!(
            "Provide a bigger window: `shaman -H {} -W {} ...`",
            args.replan,
            args.replan,
        )
        .context("h ≤ w")
        .context("Replanning period must be smaller or equal to conflict window"));
    }
    shaman::level(
        &args.map,
        shaman::Params {
            window: args.window,
            replan: args.replan,
        },
        args.fps,
        args.stop,
        args.keep,
    )?;
    Ok(())
}
