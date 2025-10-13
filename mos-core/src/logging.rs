use log::LevelFilter;
use env_logger;
use std::str::FromStr;

pub fn init_logging(level: &str) -> Result<(), log::ParseLevelError> {
    let level = LevelFilter::from_str(level)?;
    env_logger::builder().filter_level(level).init();
    Ok(())
}
