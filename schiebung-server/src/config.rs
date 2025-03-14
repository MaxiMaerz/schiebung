use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(default)]
pub struct ServerConfig {
    pub max_subscribers: usize,
}

impl Default for ServerConfig {
    fn default() -> Self {
        ServerConfig {
            max_subscribers: 10,
        }
    }
}

pub fn get_config() -> Result<ServerConfig, confy::ConfyError> {
    let config = confy::load("schiebung", "schiebung-server.yaml");
    match config {
        Ok(config) => Ok(config),
        Err(e) => {
            println!("Error loading config: {:?}", e);
            Err(e)
        }
    }
}
