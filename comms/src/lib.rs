pub mod messages_capnp {
    include!(concat!(env!("OUT_DIR"), "/messages_capnp.rs"));
}

use std::error::Error;

// Re-export the generated Cap'n Proto types
pub use messages_capnp::*;

// Re-export modules
pub mod client;
pub mod config;
pub mod server;

pub use client::TransformPublisher;
pub use config::ZenohConfig;

const TRANSLATION_SIZE: u32 = 3;
const ROTATION_SIZE: u32 = 4;

// Type conversion helpers
impl From<schiebung::types::TransformType> for messages_capnp::TransformKind {
    fn from(tt: schiebung::types::TransformType) -> Self {
        use schiebung::types::TransformType;
        match tt {
            TransformType::Static => Self::Static,
            TransformType::Dynamic => Self::Dynamic,
        }
    }
}

impl From<messages_capnp::TransformKind> for schiebung::types::TransformType {
    fn from(tk: messages_capnp::TransformKind) -> Self {
        use schiebung::types::TransformType;
        match tk {
            messages_capnp::TransformKind::Static => TransformType::Static,
            messages_capnp::TransformKind::Dynamic => TransformType::Dynamic,
        }
    }
}

/// Helper to serialize a NewTransform to bytes
pub fn serialize_new_transform(
    from: &str,
    to: &str,
    time: f64,
    translation: &[f64; 3],
    rotation: &[f64; 4],
    kind: messages_capnp::TransformKind,
) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut message = capnp::message::Builder::new_default();
    let mut transform = message.init_root::<new_transform::Builder>();

    transform.set_from(from);
    transform.set_to(to);
    transform.set_time(time);

    {
        let mut trans = transform.reborrow().init_translation(TRANSLATION_SIZE);
        trans.set(0, translation[0]);
        trans.set(1, translation[1]);
        trans.set(2, translation[2]);
    }

    {
        let mut rot = transform.reborrow().init_rotation(ROTATION_SIZE);
        rot.set(0, rotation[0]);
        rot.set(1, rotation[1]);
        rot.set(2, rotation[2]);
        rot.set(3, rotation[3]);
    }

    transform.set_kind(kind);

    let mut buffer = Vec::new();
    capnp::serialize::write_message(&mut buffer, &message)?;
    Ok(buffer)
}

/// Helper to deserialize a NewTransform from bytes
pub fn deserialize_new_transform(
    data: &[u8],
) -> Result<
    (
        String,
        String,
        f64,
        [f64; 3],
        [f64; 4],
        messages_capnp::TransformKind,
    ),
    Box<dyn Error>,
> {
    let reader =
        capnp::serialize::read_message(&mut &data[..], capnp::message::ReaderOptions::new())?;
    let transform = reader.get_root::<new_transform::Reader>()?;

    let translation = {
        let trans = transform.get_translation()?;
        [trans.get(0), trans.get(1), trans.get(2)]
    };

    let rotation = {
        let rot = transform.get_rotation()?;
        [rot.get(0), rot.get(1), rot.get(2), rot.get(3)]
    };

    let kind = transform.get_kind()?;

    Ok((
        transform.get_from()?.to_str()?.to_string(),
        transform.get_to()?.to_str()?.to_string(),
        transform.get_time(),
        translation,
        rotation,
        kind,
    ))
}

pub fn serialize_transform_request(
    id: u64,
    from: &str,
    to: &str,
    time: f64,
) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut message = capnp::message::Builder::new_default();
    let mut request = message.init_root::<transform_request::Builder>();

    request.set_id(id);
    request.set_from(from);
    request.set_to(to);
    request.set_time(time);

    let mut buffer = Vec::new();
    capnp::serialize::write_message(&mut buffer, &message)?;
    Ok(buffer)
}

pub fn deserialize_transform_request(
    data: &[u8],
) -> Result<(u64, String, String, f64), Box<dyn Error>> {
    let reader =
        capnp::serialize::read_message(&mut &data[..], capnp::message::ReaderOptions::new())?;
    let request = reader.get_root::<transform_request::Reader>()?;

    Ok((
        request.get_id(),
        request.get_from()?.to_str()?.to_string(),
        request.get_to()?.to_str()?.to_string(),
        request.get_time(),
    ))
}

pub fn serialize_transform_response(
    id: u64,
    time: f64,
    translation: &[f64; 3],
    rotation: &[f64; 4],
    success: bool,
    error_message: &str,
) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut message = capnp::message::Builder::new_default();
    let mut response = message.init_root::<transform_response::Builder>();

    response.set_id(id);
    response.set_time(time);
    response.set_success(success);
    response.set_error_message(error_message);

    {
        let mut trans = response.reborrow().init_translation(TRANSLATION_SIZE);
        trans.set(0, translation[0]);
        trans.set(1, translation[1]);
        trans.set(2, translation[2]);
    }

    {
        let mut rot = response.reborrow().init_rotation(ROTATION_SIZE);
        rot.set(0, rotation[0]);
        rot.set(1, rotation[1]);
        rot.set(2, rotation[2]);
        rot.set(3, rotation[3]);
    }

    let mut buffer = Vec::new();
    capnp::serialize::write_message(&mut buffer, &message)?;
    Ok(buffer)
}

pub fn deserialize_transform_response(
    data: &[u8],
) -> Result<(u64, f64, [f64; 3], [f64; 4], bool, String), Box<dyn Error>> {
    let reader =
        capnp::serialize::read_message(&mut &data[..], capnp::message::ReaderOptions::new())?;
    let response = reader.get_root::<transform_response::Reader>()?;

    let translation = {
        let trans = response.get_translation()?;
        [trans.get(0), trans.get(1), trans.get(2)]
    };

    let rotation = {
        let rot = response.get_rotation()?;
        [rot.get(0), rot.get(1), rot.get(2), rot.get(3)]
    };

    Ok((
        response.get_id(),
        response.get_time(),
        translation,
        rotation,
        response.get_success(),
        response.get_error_message()?.to_str()?.to_string(),
    ))
}
