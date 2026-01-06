pub const TRANSFORM_PUB_TOPIC: &str = "schiebung/transforms/new";
pub const TRANSFORM_QUERY_TOPIC: &str = "schiebung/transforms/get";

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct ZenohConfig {
    #[serde(default = "default_mode")]
    pub mode: String,
}

fn default_mode() -> String {
    "peer".to_string()
}

impl Default for ZenohConfig {
    fn default() -> Self {
        Self {
            mode: default_mode(),
        }
    }
}

impl ZenohConfig {
    pub fn to_zenoh_config(&self) -> Result<zenoh::Config, crate::error::CommsError> {
        let mut config = zenoh::Config::default();
        config
            .insert_json5("mode", &format!("\"{}\"", self.mode))
            .map_err(|e| {
                crate::error::CommsError::Config(format!("Failed to configure zenoh: {}", e))
            })?;
        Ok(config)
    }
}
