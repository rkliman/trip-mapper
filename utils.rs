use crate::Coordinates;

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

    format!(
        r#"{{"type": "LineString", "coordinates": [{}]}}"#,
        coord_pairs.join(",")
    )
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