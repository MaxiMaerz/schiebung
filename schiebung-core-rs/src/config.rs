use dirs::home_dir;
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(default)]
pub struct BufferConfig {
    pub buffer_window: f64,
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
