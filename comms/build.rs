fn main() {
    capnpc::CompilerCommand::new()
        .file("messages.capnp")
        .run()
        .expect("Failed to compile Cap'n Proto schema");
}
