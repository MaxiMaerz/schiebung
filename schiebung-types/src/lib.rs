
#[derive(Debug)]
#[repr(C)]
pub struct TransformRequest {
    pub id: i32,
    pub from: [char; 100],
    pub to: [char; 100],
    pub time: f64,
}

#[derive(Debug)]
#[repr(C)]
pub struct TransformTest {
    pub id: i32,
}