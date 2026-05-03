use dirs::home_dir;
use serde::{Deserialize, Serialize};

/// Runtime configuration for [`BufferTree`](crate::BufferTree).
///
/// Loaded from disk via [`get_config`] using [`confy`], with sensible
/// fallbacks supplied by the [`Default`] impl when no config file exists.
#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(default)]
pub struct BufferConfig {
    /// How long (in seconds) to keep historical samples per edge before
    /// older entries are evicted. A larger window enables lookups further
    /// in the past at the cost of memory.
    pub buffer_window: f64,
    /// Filesystem directory where buffer visualizations and other artifacts
    /// are written. Defaults to the user's home directory.
    pub save_path: String,
}

impl Default for BufferConfig {
    fn default() -> Self {
        BufferConfig {
            buffer_window: 120.0,
            save_path: home_dir().unwrap().display().to_string(),
        }
    }
}

/// Load [`BufferConfig`] from the platform-standard config location.
///
/// Uses [`confy`] under the application name `"schiebung"` and config name
/// `"schiebung-core.yaml"`. If the file does not exist it is created with
/// the [`Default`] values; this only fails if I/O or deserialization errors
/// occur.
pub fn get_config() -> Result<BufferConfig, confy::ConfyError> {
    let config = confy::load("schiebung", "schiebung-core.yaml");
    match config {
        Ok(config) => Ok(config),
        Err(e) => {
            println!("Error loading config: {:?}", e);
            Err(e)
        }
    }
}
