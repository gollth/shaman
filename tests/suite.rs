use rstest::rstest;
use shaman::Shaman;
use std::path::PathBuf;

#[rstest]
fn regression(#[files("maps/*.txt")] file: PathBuf) {
    Shaman::parse(file).unwrap().solve().unwrap();
}

#[rstest]
fn impossible(#[files("maps/impossible/*.txt")] file: PathBuf) {
    let e = Shaman::parse(file).unwrap_err();
    let msg = format!("{e:#}");
    assert!(msg.contains("Failed to find a solution"), "{msg}");
}
