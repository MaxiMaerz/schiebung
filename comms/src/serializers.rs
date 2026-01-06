use crate::error::CommsError;
use crate::messages_capnp::{self, new_transform, transform_request, transform_response};
use schiebung::types::StampedIsometry;

const TRANSLATION_SIZE: u32 = 3;
const ROTATION_SIZE: u32 = 4;

/// Serialize a new transform with StampedIsometry
pub fn serialize_new_transform(
    from: &str,
    to: &str,
    stamped_isometry: &StampedIsometry,
    kind: messages_capnp::TransformKind,
) -> Result<Vec<u8>, CommsError> {
    let mut message = capnp::message::Builder::new_default();
    let mut transform = message.init_root::<new_transform::Builder>();

    transform.set_from(from);
    transform.set_to(to);
    transform.set_time(stamped_isometry.stamp());

    let translation = stamped_isometry.translation();
    {
        let mut trans = transform.reborrow().init_translation(TRANSLATION_SIZE);
        for (i, &val) in translation.iter().enumerate() {
            trans.set(i as u32, val);
        }
    }

    let rotation = stamped_isometry.rotation();
    {
        let mut rot = transform.reborrow().init_rotation(ROTATION_SIZE);
        for (i, &val) in rotation.iter().enumerate() {
            rot.set(i as u32, val);
        }
    }

    transform.set_kind(kind);

    let mut buffer = Vec::new();
    capnp::serialize::write_message(&mut buffer, &message)?;
    Ok(buffer)
}

/// Deserialize a new transform into StampedIsometry
pub fn deserialize_new_transform(
    data: &[u8],
) -> Result<
    (
        String,
        String,
        StampedIsometry,
        messages_capnp::TransformKind,
    ),
    CommsError,
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

    let stamped_isometry = StampedIsometry::new(translation, rotation, transform.get_time());
    let kind = transform.get_kind()?;

    Ok((
        transform.get_from()?.to_str()?.to_string(),
        transform.get_to()?.to_str()?.to_string(),
        stamped_isometry,
        kind,
    ))
}

/// Serialize a transform request
pub fn serialize_transform_request(from: &str, to: &str, time: f64) -> Result<Vec<u8>, CommsError> {
    let mut message = capnp::message::Builder::new_default();
    let mut request = message.init_root::<transform_request::Builder>();

    request.set_from(from);
    request.set_to(to);
    request.set_time(time);

    let mut buffer = Vec::new();
    capnp::serialize::write_message(&mut buffer, &message)?;
    Ok(buffer)
}

/// Deserialize a transform request
pub fn deserialize_transform_request(data: &[u8]) -> Result<(String, String, f64), CommsError> {
    let reader =
        capnp::serialize::read_message(&mut &data[..], capnp::message::ReaderOptions::new())?;
    let request = reader.get_root::<transform_request::Reader>()?;

    Ok((
        request.get_from()?.to_str()?.to_string(),
        request.get_to()?.to_str()?.to_string(),
        request.get_time(),
    ))
}

/// Serialize a transform response with StampedIsometry
pub fn serialize_transform_response(
    stamped_isometry: &StampedIsometry,
    success: bool,
    error_message: &str,
) -> Result<Vec<u8>, CommsError> {
    let mut message = capnp::message::Builder::new_default();
    let mut response = message.init_root::<transform_response::Builder>();

    response.set_time(stamped_isometry.stamp());
    response.set_success(success);
    response.set_error_message(error_message);

    let translation = stamped_isometry.translation();
    {
        let mut trans = response.reborrow().init_translation(TRANSLATION_SIZE);
        for (i, &val) in translation.iter().enumerate() {
            trans.set(i as u32, val);
        }
    }

    let rotation = stamped_isometry.rotation();
    {
        let mut rot = response.reborrow().init_rotation(ROTATION_SIZE);
        for (i, &val) in rotation.iter().enumerate() {
            rot.set(i as u32, val);
        }
    }

    let mut buffer = Vec::new();
    capnp::serialize::write_message(&mut buffer, &message)?;
    Ok(buffer)
}

/// Deserialize a transform response into Result<StampedIsometry, String>
/// Returns Ok(StampedIsometry) on success, or Err(error_message) on failure
pub fn deserialize_transform_response(
    data: &[u8],
) -> Result<Result<StampedIsometry, String>, CommsError> {
    let reader =
        capnp::serialize::read_message(&mut &data[..], capnp::message::ReaderOptions::new())?;
    let response = reader.get_root::<transform_response::Reader>()?;

    let success = response.get_success();

    if success {
        let translation = {
            let trans = response.get_translation()?;
            [trans.get(0), trans.get(1), trans.get(2)]
        };

        let rotation = {
            let rot = response.get_rotation()?;
            [rot.get(0), rot.get(1), rot.get(2), rot.get(3)]
        };

        let stamped_isometry = StampedIsometry::new(translation, rotation, response.get_time());
        Ok(Ok(stamped_isometry))
    } else {
        let error_message = response.get_error_message()?.to_str()?.to_string();
        Ok(Err(error_message))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use schiebung::types::StampedIsometry;

    #[test]
    fn test_transform_response_roundtrip() {
        // Test successful response
        let stamped_iso = StampedIsometry::new([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 42.0);

        let serialized = serialize_transform_response(&stamped_iso, true, "").unwrap();
        let deserialized = deserialize_transform_response(&serialized).unwrap();

        match deserialized {
            Ok(result) => {
                assert_eq!(result.stamp(), 42.0);
                let trans = result.translation();
                assert_eq!(trans, [1.0, 2.0, 3.0]);
                let rot = result.rotation();
                assert_eq!(rot, [0.0, 0.0, 0.0, 1.0]);
            }
            Err(e) => panic!("Expected success, got error: {}", e),
        }
    }

    #[test]
    fn test_transform_response_error() {
        // Test error response
        let dummy = StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0);

        let serialized = serialize_transform_response(&dummy, false, "test error").unwrap();
        let deserialized = deserialize_transform_response(&serialized).unwrap();

        match deserialized {
            Ok(_) => panic!("Expected error, got success"),
            Err(e) => assert_eq!(e, "test error"),
        }
    }
}
