use rstest::rstest;
use shaman::Shaman;
use std::path::PathBuf;

#[rstest]
fn regression(#[files("maps/*.txt")] file: PathBuf) {
    Shaman::parse(file).unwrap().solve().unwrap();
}

#[rstest]
#[case::no_path("maps/impossible/no-path.txt", "No route found")]
#[case::invalid_symbol("maps/impossible/invalid-symbol.txt", "Expected either an obstacle")]
fn impossible(#[case] file: &str, #[case] expectation: &str) {
    let e = Shaman::parse(file).unwrap_err();
    let msg = format!("{e:#}");
    assert!(
        msg.contains(expectation),
        "Expected that '{expectation}' would be part of the error but it wasn't: {msg}"
    );
}
