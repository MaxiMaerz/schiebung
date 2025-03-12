use dirs::home_dir;
use serde::{Deserialize, Serialize};

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
    let config = confy::load("schiebung", "schiebung-core.yaml");
    match config {
        Ok(config) => Ok(config),
        Err(e) => {
            println!("Error loading config: {:?}", e);
            Err(e)
        }
    }
}
