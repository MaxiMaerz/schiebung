use iceoryx2::port::event_id::EventId;
use nalgebra::{Isometry, Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use std::cmp::Ordering;

#[derive(Clone, Debug)]
pub enum TransformType {
    /// Changes over time
    Dynamic = 0,
    /// Does not change over time
    Static = 1,
}

impl TryFrom<u8> for TransformType {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(TransformType::Dynamic),
            1 => Ok(TransformType::Static),
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct TransformRequest {
    pub id: u128,
    pub from: [char; 100],
    pub to: [char; 100],
    pub time: f64,
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct TransformResponse {
    pub id: u128,
    pub time: f64,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}

#[derive(Debug)]
#[repr(C)]
pub struct NewTransform {
    pub from: [char; 100],
    pub to: [char; 100],
    pub time: f64,
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
    pub kind: u8,
}

#[derive(Debug)]
pub enum PubSubEvent {
    PublisherConnected = 0,
    PublisherDisconnected = 1,
    SubscriberConnected = 2,
    SubscriberDisconnected = 3,
    SentSample = 4,
    Error = 5,
    ReceivedSample = 6,
    SentHistory = 7,
    ProcessDied = 8,
    Unknown,
}

impl From<PubSubEvent> for EventId {
    fn from(value: PubSubEvent) -> Self {
        EventId::new(value as usize)
    }
}

impl From<EventId> for PubSubEvent {
    fn from(value: EventId) -> Self {
        match value.as_value() {
            0 => PubSubEvent::PublisherConnected,
            1 => PubSubEvent::PublisherDisconnected,
            2 => PubSubEvent::SubscriberConnected,
            3 => PubSubEvent::SubscriberDisconnected,
            4 => PubSubEvent::SentSample,
            5 => PubSubEvent::Error,
            6 => PubSubEvent::ReceivedSample,
            7 => PubSubEvent::SentHistory,
            8 => PubSubEvent::ProcessDied,
            _ => PubSubEvent::Unknown,
        }
    }
}

#[derive(Clone, Debug)]
pub struct StampedTransform {
    stamp: f64,
    translation: Vector3<f64>,
    rotation: UnitQuaternion<f64>,
}
impl Into<StampedTransform> for TransformResponse {
    fn into(self) -> StampedTransform {
        let translation_vector = Vector3::new(
            self.translation[0],
            self.translation[1],
            self.translation[2],
        );

        let rotation_quaternion = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            self.rotation[3],
            self.rotation[0],
            self.rotation[1],
            self.rotation[2],
        ));

        StampedTransform {
            stamp: self.time,
            translation: translation_vector,
            rotation: rotation_quaternion,
        }
    }
}
#[derive(Clone, Debug)]
pub struct StampedIsometry {
    pub isometry: Isometry3<f64>,
    /// The time at which this isometry was recorded in seconds
    pub stamp: f64,
}

impl PartialEq for StampedIsometry {
    fn eq(&self, other: &Self) -> bool {
        self.stamp == other.stamp
    }
}

impl Eq for StampedIsometry {}

impl Ord for StampedIsometry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.stamp.partial_cmp(&other.stamp).unwrap()
    }
}

impl PartialOrd for StampedIsometry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Into<StampedIsometry> for TransformResponse {
    fn into(self) -> StampedIsometry {
        let isometry = Isometry::from_parts(
            Translation3::new(
                self.translation[0],
                self.translation[1],
                self.translation[2],
            ),
            UnitQuaternion::new_normalize(Quaternion::new(
                self.rotation[3],
                self.rotation[0],
                self.rotation[1],
                self.rotation[2],
            )),
        );
        StampedIsometry {
            isometry,
            stamp: self.time,
        }
    }
}
