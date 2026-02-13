use std::path::PathBuf;

use clap::Parser;
use miette::Result;

#[derive(Debug, Parser)]
struct Args {
    /// How fast to simulate
    #[arg(short, long, default_value_t = 0.)]
    fps: f32,

    /// Don't solve right away, but print the conflicted solution
    #[arg(short('x'))]
    stop: bool,

    /// Path to a map file to use
    map: PathBuf,
}

fn main() -> Result<()> {
    let args = Args::parse();
    shaman::level(&args.map, args.fps, args.stop)?;
    Ok(())
}
