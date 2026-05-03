pub const TRANSFORM_PUB_TOPIC: &str = "schiebung/transforms/new";
pub const TRANSFORM_QUERY_TOPIC: &str = "schiebung/transforms/get";

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct ZenohConfig {
    #[serde(default = "default_mode")]
    pub mode: String,
    /// Endpoints to listen on (e.g. `["tcp/127.0.0.1:7447"]`). Empty = use zenoh defaults.
    #[serde(default)]
    pub listen: Vec<String>,
    /// Endpoints to actively connect to (e.g. `["tcp/127.0.0.1:7447"]`). Empty = use zenoh defaults.
    #[serde(default)]
    pub connect: Vec<String>,
    /// Enable UDP multicast peer discovery. Defaults to `true` (zenoh default).
    /// Set to `false` for deterministic deployments using explicit `listen`/`connect` endpoints.
    #[serde(default = "default_multicast_scouting")]
    pub multicast_scouting: bool,
}

fn default_mode() -> String {
    "peer".to_string()
}

fn default_multicast_scouting() -> bool {
    true
}

impl Default for ZenohConfig {
    fn default() -> Self {
        Self {
            mode: default_mode(),
            listen: Vec::new(),
            connect: Vec::new(),
            multicast_scouting: default_multicast_scouting(),
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
        if !self.listen.is_empty() {
            config
                .insert_json5("listen/endpoints", &json_string_array(&self.listen))
                .map_err(|e| {
                    crate::error::CommsError::Config(format!(
                        "Failed to configure zenoh listen endpoints: {}",
                        e
                    ))
                })?;
        }
        if !self.connect.is_empty() {
            config
                .insert_json5("connect/endpoints", &json_string_array(&self.connect))
                .map_err(|e| {
                    crate::error::CommsError::Config(format!(
                        "Failed to configure zenoh connect endpoints: {}",
                        e
                    ))
                })?;
        }
        if !self.multicast_scouting {
            config
                .insert_json5("scouting/multicast/enabled", "false")
                .map_err(|e| {
                    crate::error::CommsError::Config(format!(
                        "Failed to disable zenoh multicast scouting: {}",
                        e
                    ))
                })?;
        }
        Ok(config)
    }
}

fn json_string_array(items: &[String]) -> String {
    let escaped: Vec<String> = items
        .iter()
        .map(|s| format!("\"{}\"", s.replace('\\', "\\\\").replace('"', "\\\"")))
        .collect();
    format!("[{}]", escaped.join(","))
}
