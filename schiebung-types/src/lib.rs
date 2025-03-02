
#[derive(Debug, Clone)]
#[repr(C)]
pub struct TransformRequest {
    pub id: i32,
    pub from: [char; 100],
    pub to: [char; 100],
    pub time: f64,
}

#[derive(Debug, Clone)]
#[repr(C)]
pub struct TransformResponse {
    pub id: i32,
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
}

use iceoryx2::port::event_id::EventId;

pub enum PubSubEvent {
    PublisherConnected = 0,
    PublisherDisconnected = 1,
    SubscriberConnected = 2,
    SubscriberDisconnected = 3,
    SentSample = 4,
    ReceivedSample = 5,
    SentHistory = 6,
    ProcessDied = 7,
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
            5 => PubSubEvent::ReceivedSample,
            6 => PubSubEvent::SentHistory,
            7 => PubSubEvent::ProcessDied,
            _ => PubSubEvent::Unknown,
        }
    }
}