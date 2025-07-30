{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = with pkgs; [
    rustc
    cargo
    rustfmt
    clippy
    pkg-config
    openssl
  ];

  # Needed by reqwest with rustls-tls and/or native-tls
  RUSTFLAGS = "-C target-cpu=native";

  shellHook = ''
    echo "Welcome to the Trip Planner Rust Dev Shell!"
    echo "Dependencies loaded: Rust, Cargo, rustfmt, clippy, OpenSSL"
  '';
}
