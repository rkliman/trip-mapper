use crate::Coordinates;
use serde_json::Value;
use reqwest::Client;

// Constants moved from main.rs
pub const METERS_TO_MILES: f64 = 1609.34;
pub const EARTH_RADIUS_MILES: f64 = 3958.8;
pub const GEODESIC_SEGMENTS: usize = 100; // Number of segments for geodesic curves

/// Calculate the haversine distance between two coordinates in miles
pub fn haversine_distance(coord1: Coordinates, coord2: Coordinates) -> f64 {
    let (lon1, lat1) = (coord1.0.to_radians(), coord1.1.to_radians());
    let (lon2, lat2) = (coord2.0.to_radians(), coord2.1.to_radians());

    let dlon = lon2 - lon1;
    let dlat = lat2 - lat1;

    let a = dlat.sin().powi(2) + lat1.cos() * lat2.cos() * dlon.sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    EARTH_RADIUS_MILES * c
}

/// Calculate the total haversine distance for a series of coordinates
pub fn calculate_total_haversine_distance(coords: &[Coordinates]) -> f64 {
    coords
        .windows(2)
        .map(|window| haversine_distance(window[0], window[1]))
        .sum()
}

/// Create a simple LineString GeoJSON from coordinates
pub fn create_linestring_geojson(coords: &[Coordinates]) -> String {
    let coord_pairs: Vec<String> = coords
        .iter()
        .map(|(lon, lat)| format!("[{},{}]", lon, lat))
        .collect();

    format!(
        r#"{{"type": "LineString", "coordinates": [{}]}}"#,
        coord_pairs.join(",")
    )
}

/// Create a geodesic line between coordinates using great circle interpolation
pub fn create_geodesic_geojson(coords: &[Coordinates]) -> String {
    if coords.len() < 2 {
        return create_linestring_geojson(coords);
    }

    let mut all_points = Vec::new();
    
    // For each segment between waypoints, create a geodesic curve
    for window in coords.windows(2) {
        let start = window[0];
        let end = window[1];
        
        let geodesic_points = interpolate_geodesic(start, end, GEODESIC_SEGMENTS);
        
        // Add all points except the last one (to avoid duplicates at waypoints)
        all_points.extend(&geodesic_points[..geodesic_points.len() - 1]);
    }
    
    // Add the final point
    if let Some(&last_coord) = coords.last() {
        all_points.push(last_coord);
    }

    let coord_pairs: Vec<String> = all_points
        .iter()
        .map(|(lon, lat)| format!("[{},{}]", lon, lat))
        .collect();

    let geojson = format!(
        r#"{{"type": "LineString", "coordinates": [{}]}}"#,
        coord_pairs.join(",")
    );

    geojson
}

/// Interpolate points along a great circle between two coordinates
pub fn interpolate_geodesic(start: Coordinates, end: Coordinates, segments: usize) -> Vec<Coordinates> {
    let (lon1, lat1) = (start.0.to_radians(), start.1.to_radians());
    let (lon2, lat2) = (end.0.to_radians(), end.1.to_radians());
    
    let mut points = Vec::with_capacity(segments + 1);
    
    // Calculate the angular distance between the points
    let d = haversine_distance(start, end) / EARTH_RADIUS_MILES;
    
    // If the distance is very small, just return the endpoints
    if d < 0.001 {
        return vec![start, end];
    }
    
    for i in 0..=segments {
        let f = i as f64 / segments as f64;
        
        // Use spherical linear interpolation (slerp) for great circle
        let a = (1.0 - f) * d.sin() / d.sin();
        let b = f * d.sin() / d.sin();
        
        let x = a * lat1.cos() * lon1.cos() + b * lat2.cos() * lon2.cos();
        let y = a * lat1.cos() * lon1.sin() + b * lat2.cos() * lon2.sin();
        let z = a * lat1.sin() + b * lat2.sin();
        
        let lat = z.atan2((x * x + y * y).sqrt());
        let lon = y.atan2(x);
        
        points.push((lon.to_degrees(), lat.to_degrees()));
    }
    
    points
}

/// Generate a cache key for a route based on coordinates
/// Rounds coordinates to 4 decimal places (~11m precision) to allow for small variations
pub fn generate_route_cache_key(coords: &[Coordinates]) -> String {
    let rounded_coords: Vec<String> = coords
        .iter()
        .map(|(lon, lat)| format!("{:.4},{:.4}", lon, lat))
        .collect();
    format!("route:{}", rounded_coords.join(";"))
}

/// Get current Unix timestamp in seconds
pub fn get_current_timestamp() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs()
}

pub async fn fetch_osm_way_coordinates(way_id: u64) -> Result<Vec<(f64, f64)>, Box<dyn std::error::Error>> {
    let query = format!(
        "[out:json];way({});(._;>;);out;",
        way_id
    );
    let url = "https://overpass-api.de/api/interpreter";
    let client = Client::new();
    let resp = client
        .post(url)
        .header("Content-Type", "application/x-www-form-urlencoded")
        .body(format!("data={}", urlencoding::encode(&query)))
        .send()
        .await?
        .text()
        .await?;

    let json: Value = serde_json::from_str(&resp)?;

    // Map node ID to coordinates
    let mut node_map = std::collections::HashMap::new();
    let mut way_nodes = Vec::new();

    if let Some(elements) = json["elements"].as_array() {
        for el in elements {
            if el["type"] == "node" {
                let id = el["id"].as_u64().unwrap();
                let lat = el["lat"].as_f64().unwrap();
                let lon = el["lon"].as_f64().unwrap();
                node_map.insert(id, (lon, lat));
            } else if el["type"] == "way" {
                if let Some(nodes) = el["nodes"].as_array() {
                    way_nodes = nodes.iter().filter_map(|n| n.as_u64()).collect();
                }
            }
        }
    }

    // Build ordered coordinate list
    let coords = way_nodes
        .iter()
        .filter_map(|id| node_map.get(id).copied())
        .collect();

    Ok(coords)
}

pub async fn fetch_osm_relation_coordinates(relation_id: u64) -> Result<Vec<(f64, f64)>, Box<dyn std::error::Error>> {
    let query = format!(
        "[out:json];relation({});(._;>;);out;",
        relation_id
    );
    let url = "https://overpass-api.de/api/interpreter";
    let client = reqwest::Client::new();
    let resp = client
        .post(url)
        .header("Content-Type", "application/x-www-form-urlencoded")
        .body(format!("data={}", urlencoding::encode(&query)))
        .send()
        .await?
        .text()
        .await?;

    let json: Value = serde_json::from_str(&resp)?;

    // Build maps for ways and nodes
    let mut node_map = std::collections::HashMap::new();
    let mut way_map = std::collections::HashMap::new();
    let mut relation_ways = Vec::new();

    if let Some(elements) = json["elements"].as_array() {
        for el in elements {
            match el["type"].as_str() {
                Some("node") => {
                    let id = el["id"].as_u64().unwrap();
                    let lat = el["lat"].as_f64().unwrap();
                    let lon = el["lon"].as_f64().unwrap();
                    node_map.insert(id, (lon, lat));
                }
                Some("way") => {
                    let id = el["id"].as_u64().unwrap();
                    let nodes = el["nodes"].as_array()
                        .map(|arr| arr.iter().filter_map(|n| n.as_u64()).collect::<Vec<_>>())
                        .unwrap_or_default();
                    way_map.insert(id, nodes);
                }
                Some("relation") => {
                    if let Some(members) = el["members"].as_array() {
                        for m in members {
                            if m["type"] == "way" {
                                if let Some(way_id) = m["ref"].as_u64() {
                                    relation_ways.push(way_id);
                                }
                            }
                        }
                    }
                }
                _ => {}
            }
        }
    }

    // Concatenate all way node coordinates in order
    let mut coords = Vec::new();
    for way_id in relation_ways {
        if let Some(node_ids) = way_map.get(&way_id) {
            for node_id in node_ids {
                if let Some(coord) = node_map.get(node_id) {
                    coords.push(*coord);
                }
            }
        }
    }

    Ok(coords)
}

pub async fn fetch_osm_ways_chained(way_ids: &[u64]) -> Result<Vec<Coordinates>, Box<dyn std::error::Error>> {
    let ids_str = way_ids.iter().map(|id| id.to_string()).collect::<Vec<_>>().join(",");
    let query = format!(
        "[out:json];way({});(._;>;);out;",
        ids_str
    );
    let url = "https://overpass-api.de/api/interpreter";
    let client = Client::new();
    let resp = client
        .post(url)
        .header("Content-Type", "application/x-www-form-urlencoded")
        .body(format!("data={}", urlencoding::encode(&query)))
        .send()
        .await?
        .text()
        .await?;

    let json: Value = serde_json::from_str(&resp)?;

    // Build maps for ways and nodes
    let mut node_map = std::collections::HashMap::new();
    let mut way_map = std::collections::HashMap::new();

    if let Some(elements) = json["elements"].as_array() {
        for el in elements {
            match el["type"].as_str() {
                Some("node") => {
                    let id = el["id"].as_u64().unwrap();
                    let lat = el["lat"].as_f64().unwrap();
                    let lon = el["lon"].as_f64().unwrap();
                    node_map.insert(id, (lon, lat));
                }
                Some("way") => {
                    let id = el["id"].as_u64().unwrap();
                    let nodes = el["nodes"].as_array()
                        .map(|arr| arr.iter().filter_map(|n| n.as_u64()).collect::<Vec<_>>())
                        .unwrap_or_default();
                    way_map.insert(id, nodes);
                }
                _ => {}
            }
        }
    }

    // Concatenate all way node coordinates in order
    let mut coords = Vec::new();
    for way_id in way_ids {
        if let Some(node_ids) = way_map.get(way_id) {
            for node_id in node_ids {
                if let Some(coord) = node_map.get(node_id) {
                    coords.push(*coord);
                }
            }
        }
    }

    Ok(coords)
}

pub fn create_styled_linestring_geojson(coords: &[Coordinates], method: &str) -> String {
    let coord_pairs: Vec<String> = coords
        .iter()
        .map(|(lon, lat)| format!("[{},{}]", lon, lat))
        .collect();

    // Style for hiking/walking
    let (stroke, dasharray) = match method {
        "hiking" | "walking" => ("#FFA500", "8,4"), // orange dashed
        _ => ("#3388ff", "1"), // default blue solid
    };

    format!(
        r#"{{
            "type": "LineString",
            "coordinates": [{}],
            "properties": {{
                "stroke": "{}",
                "stroke-width": 3,
                "stroke-dasharray": "{}",
                "method": "{}"
            }}
        }}"#,
        coord_pairs.join(","),
        stroke,
        dasharray,
        method
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haversine_distance() {
        // Test distance between NYC and LA (approximately 2445 miles)
        let nyc = (-74.0059, 40.7128);
        let la = (-118.2437, 34.0522);
        let distance = haversine_distance(nyc, la);
        
        // Should be within 50 miles of expected distance
        assert!((distance - 2445.0).abs() < 50.0);
    }

    #[test]
    fn test_total_distance_calculation() {
        let coords = vec![
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
        ];
        
        let total = calculate_total_haversine_distance(&coords);
        let expected = haversine_distance(coords[0], coords[1]) + 
                      haversine_distance(coords[1], coords[2]);
        
        assert!((total - expected).abs() < 0.001);
    }

    #[test]
    fn test_geodesic_interpolation() {
        let start = (0.0, 0.0);
        let end = (1.0, 1.0);
        let points = interpolate_geodesic(start, end, 10);
        
        assert_eq!(points.len(), 11); // 10 segments = 11 points
        assert_eq!(points[0], start);
        assert_eq!(points[10], end);
    }

    #[test]
    fn test_cache_key_generation() {
        let coords = vec![(1.123456, 2.987654), (3.111111, 4.222222)];
        let key = generate_route_cache_key(&coords);
        assert_eq!(key, "route:1.1235,2.9877;3.1111,4.2222");
    }
}