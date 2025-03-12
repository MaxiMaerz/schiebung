use dirs::home_dir;
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(default)]
pub struct BufferConfig {
    pub max_transform_history: usize,
    pub save_path: String,
}

impl Default for BufferConfig {
    fn default() -> Self {
        BufferConfig {
            max_transform_history: 1000,
            save_path: home_dir().unwrap().display().to_string(),
        }
    }
}

pub fn get_config() -> Result<BufferConfig, confy::ConfyError> {
    let config_path = confy::get_configuration_file_path("schiebung", "schiebung-core")?;

    let mut cfg = BufferConfig::default();
    if config_path.exists() {
        println!("Loading config from: {:?}", config_path);
        cfg = confy::load_path(config_path)?;
    } else {
        // no config found, generate default
        println!("No config found, using default");
    };
    Ok(cfg)
}
