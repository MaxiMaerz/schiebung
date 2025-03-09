
pub fn setup_logger() {
  let _ = env_logger::Builder::new()
  .filter(None, log::LevelFilter::Debug)
  .is_test(true)
  .try_init();
}