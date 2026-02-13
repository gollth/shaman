use clap::Parser;

#[derive(Debug, Parser)]
struct Args {
    /// How fast to simulate
    #[arg(short, long, default_value_t = 0.)]
    fps: f32,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    shaman::level(args.fps)?;
    Ok(())
}
