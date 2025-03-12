use iceoryx2::port::event_id::EventId;

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
