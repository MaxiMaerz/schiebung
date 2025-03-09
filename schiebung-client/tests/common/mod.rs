
pub fn setup_logger() {
  let _ = env_logger::Builder::new()
  .filter(None, log::LevelFilter::Debug)
  .try_init();
}