/// Standard ROS 2 topic for dynamic transforms (`tf2_msgs/TFMessage`).
pub const TF_TOPIC: &str = "/tf";
/// Standard ROS 2 topic for static transforms (`tf2_msgs/TFMessage`, transient local).
pub const TF_STATIC_TOPIC: &str = "/tf_static";
/// Service for transform lookups (`schiebung_msgs/LookupTransform`).
pub const LOOKUP_TRANSFORM_SERVICE: &str = "/schiebung/lookup_transform";
/// Service returning the server's buffer content, latest sample per edge
/// (`schiebung_msgs/GetTransforms`).
pub const GET_TRANSFORMS_SERVICE: &str = "/schiebung/get_transforms";

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
    /// ROS domain id (default 0). Part of the rmw_zenoh key expressions, so it must
    /// match the `ROS_DOMAIN_ID` of any ROS 2 system this instance should talk to.
    #[serde(default)]
    pub domain_id: usize,
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
            domain_id: 0,
        }
    }
}

impl ZenohConfig {
    /// Build a config from environment variables, falling back to defaults:
    ///
    /// - `SCHIEBUNG_LISTEN`: comma-separated endpoints to listen on
    /// - `SCHIEBUNG_CONNECT`: comma-separated endpoints to connect to
    /// - `SCHIEBUNG_MULTICAST`: set to `false` to disable multicast scouting
    /// - `ROS_DOMAIN_ID`: ROS domain id (same variable ROS 2 uses)
    ///
    /// Useful on networks where UDP multicast discovery is blocked and
    /// explicit endpoints are required.
    pub fn from_env() -> Self {
        let mut config = Self::default();
        if let Ok(listen) = std::env::var("SCHIEBUNG_LISTEN") {
            config.listen = split_endpoints(&listen);
        }
        if let Ok(connect) = std::env::var("SCHIEBUNG_CONNECT") {
            config.connect = split_endpoints(&connect);
        }
        if let Ok(multicast) = std::env::var("SCHIEBUNG_MULTICAST") {
            config.multicast_scouting = multicast.trim() != "false";
        }
        if let Ok(domain_id) = std::env::var("ROS_DOMAIN_ID") {
            if let Ok(domain_id) = domain_id.trim().parse() {
                config.domain_id = domain_id;
            }
        }
        config
    }

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
        // Required by the zenoh AdvancedPublisher cache backing transient_local
        // durability (used for /tf_static). Mirrors the ros-z/rmw_zenoh default.
        config
            .insert_json5(
                "timestamping/enabled",
                r#"{"router": true, "peer": true, "client": true}"#,
            )
            .map_err(|e| {
                crate::error::CommsError::Config(format!(
                    "Failed to enable zenoh timestamping: {}",
                    e
                ))
            })?;
        Ok(config)
    }

    /// Build a hiroz context from this config.
    ///
    /// Passing our own `zenoh::Config` overrides the hiroz defaults, which
    /// assume an rmw_zenoh router at `tcp/localhost:7447`. This keeps the
    /// router-less peer-to-peer mode working for the standalone (no ROS) case.
    pub(crate) fn to_zcontext(&self) -> Result<hiroz::context::ZContext, crate::error::CommsError> {
        use hiroz::Builder;
        hiroz::context::ZContextBuilder::default()
            .with_domain_id(self.domain_id)
            .with_zenoh_config(self.to_zenoh_config()?)
            .build()
            .map_err(|e| {
                crate::error::CommsError::Config(format!("Failed to build hiroz context: {}", e))
            })
    }
}

fn split_endpoints(value: &str) -> Vec<String> {
    value
        .split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(String::from)
        .collect()
}

fn json_string_array(items: &[String]) -> String {
    let escaped: Vec<String> = items
        .iter()
        .map(|s| format!("\"{}\"", s.replace('\\', "\\\\").replace('"', "\\\"")))
        .collect();
    format!("[{}]", escaped.join(","))
}
