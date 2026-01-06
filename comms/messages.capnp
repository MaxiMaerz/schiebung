@0xf4a8b3c2d1e0f9a7;

# Transform type enumeration
enum TransformKind {
  dynamic @0;
  static @1;
}

# New transform being published
struct NewTransform {
  from @0 :Text;
  to @1 :Text;
  time @2 :Float64;
  translation @3 :List(Float64);  # [x, y, z]
  rotation @4 :List(Float64);     # [x, y, z, w] quaternion
  kind @5 :TransformKind;
}

# Request for a transform lookup
struct TransformRequest {
  from @0 :Text;
  to @1 :Text;
  time @2 :Float64;
}

# Response to a transform request
struct TransformResponse {
  time @0 :Float64;
  translation @1 :List(Float64);  # [x, y, z]
  rotation @2 :List(Float64);     # [x, y, z, w] quaternion
  success @3 :Bool;
  errorMessage @4 :Text;
}
