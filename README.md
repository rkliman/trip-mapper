# Trip Mapper

Trip Mapper is a Rust application that reads trip definitions from a YAML file, geocodes waypoints, calculates routes and distances, and generates interactive HTML maps using OpenStreetMap and Leaflet. It is designed to help you visualize and analyze trips with multiple waypoints.

## Features

- **YAML-based trip definitions:** Define trips and waypoints in a simple YAML file ([trips.yaml](trips.yaml)).
- **Geocoding with caching:** Converts place names to coordinates, caching results in [geocode_cache.json](geocode_cache.json) for efficiency.
- **Route calculation:** Supports car routing (using OSRM) and straight-line (haversine) distances.
- **Interactive maps:** Generates a combined map ([combined_map.html](combined_map.html)) and per-trip HTML maps with Leaflet and OpenStreetMap tiles.
- **Summary statistics:** Prints total distances per travel method.

## Project Structure

- [`main.rs`](main.rs): Main application logic.
- [`trips.yaml`](trips.yaml): Input file with trip and waypoint definitions.
- [`geocode_cache.json`](geocode_cache.json): Stores geocoded coordinates for reuse.
- [`combined_map.html`](combined_map.html): Output map showing all trips.
- [`shell.nix`](shell.nix): Nix shell for reproducible development environment.
- [`Cargo.toml`](Cargo.toml): Rust project manifest.

## Getting Started

### Prerequisites

- [Rust](https://www.rust-lang.org/tools/install)
- [Nix](https://nixos.org/download.html) (optional, for reproducible environment)

### Setup

**With Nix:**
```sh
nix-shell
```

**Without Nix:**
Install Rust and Cargo using rustup.

### Build and Run

```sh
cargo build
cargo run
```

### Usage

1. Edit [`trips.yaml`](trips.yaml) to define your trips. Example:
    ```yaml
    trips:
      - name: "Sample Trip"
        waypoints:
          - "New York, NY"
          - "Boston, MA"
        method: "car"
    ```
2. Run the application:
    ```sh
    cargo run
    ```
3. Open [`combined_map.html`](combined_map.html) in your browser to view all trips.

## Development

- Format code: `cargo fmt`
- Lint code: `cargo clippy`

## License

MIT License

---