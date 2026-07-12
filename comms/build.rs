use std::collections::HashSet;
use std::path::{Path, PathBuf};

/// Generate Rust types for the vendored ROS 2 message definitions in `ros_msgs/`.
///
/// Every package we use is generated locally (`external_crate: None`), including
/// the standard ones. hiroz-codegen does bundle std_msgs / geometry_msgs /
/// builtin_interfaces, but it does **not** bundle `tf2_msgs`, and its
/// `generate_user_messages()` convenience entrypoint emits references to the
/// `hiroz_msgs` crate -- which is not published on crates.io. Driving
/// `MessageGenerator` directly with our own vendored definitions keeps the
/// dependency tree crates.io-only and the RIHS01 hashes under our control.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let msgs_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("ros_msgs");
    let out_dir = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());

    let packages: Vec<PathBuf> = std::fs::read_dir(&msgs_dir)?
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| path.is_dir())
        .collect();

    let local_packages: HashSet<String> = packages
        .iter()
        .filter_map(|p| p.file_name()?.to_str().map(String::from))
        .collect();

    let package_refs: Vec<&Path> = packages.iter().map(PathBuf::as_path).collect();

    // Jazzy folds `service_msgs/ServiceEventInfo` into every service hash, and we
    // do not vendor that package. Pull it (and `builtin_interfaces`, which it
    // references and which must therefore already be resolved when it is) from
    // hiroz's bundled Jazzy assets as *dependency-only* packages: they are parsed
    // so the RIHS01 hashes resolve, but no Rust is emitted for them.
    //
    // Only these two are passed. The emit filter keys on package *name*, so any
    // bundled package sharing a name with one of ours would get emitted whole --
    // handing over the entire bundled set would drag all of geometry_msgs and
    // std_msgs into our public API instead of just the types we vendor.
    const HASH_DEPS: [&str; 2] = ["service_msgs", "builtin_interfaces"];
    let dep_packages: Vec<PathBuf> = hiroz_codegen::discover_bundled_packages(false)?
        .into_iter()
        .filter(|p| {
            p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|name| HASH_DEPS.contains(&name))
        })
        .collect();
    let dep_refs: Vec<&Path> = dep_packages.iter().map(PathBuf::as_path).collect();

    hiroz_codegen::MessageGenerator::new(hiroz_codegen::GeneratorConfig {
        generate_cdr: true,
        generate_protobuf: false,
        generate_type_info: true,
        is_humble: false,
        output_dir: out_dir,
        // Everything we reference is generated locally, so there is no external
        // message crate to defer to.
        external_crate: None,
        local_packages,
        json_out: None,
    })
    .generate_from_msg_files_with_deps(&package_refs, &dep_refs)?;

    // The generated code gates its PyO3 conversions behind a `python_registry`
    // feature that only upstream's `hiroz_msgs` crate declares. We never enable
    // it, but rustc must be told the cfg is expected or it warns on every type.
    println!("cargo::rustc-check-cfg=cfg(feature, values(\"python_registry\"))");

    // Watch the message tree recursively: `rerun-if-changed` on only the files
    // that exist today would not fire when a *new* .msg/.srv is added.
    watch_dirs(&msgs_dir);
    Ok(())
}

fn watch_dirs(dir: &Path) {
    println!("cargo:rerun-if-changed={}", dir.display());
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                watch_dirs(&path);
            } else {
                println!("cargo:rerun-if-changed={}", path.display());
            }
        }
    }
}
