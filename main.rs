// main.rs
use reqwest::Client;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::error::Error;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::{fmt, fs};
use tokio::time::{sleep, Duration};

// Custom error types for better error handling
#[derive(Debug)]
enum TripPlannerError {
    Io(std::io::Error),
    Yaml(serde_yaml::Error),
    Json(serde_json::Error),
    Network(reqwest::Error),
    Geocoding(String),
}

impl fmt::Display for TripPlannerError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            TripPlannerError::Io(e) => write!(f, "IO error: {}", e),
            TripPlannerError::Yaml(e) => write!(f, "YAML parsing error: {}", e),
            TripPlannerError::Json(e) => write!(f, "JSON error: {}", e),
            TripPlannerError::Network(e) => write!(f, "Network error: {}", e),
            TripPlannerError::Geocoding(msg) => write!(f, "Geocoding error: {}", msg),
        }
    }
}

impl Error for TripPlannerError {}

impl From<std::io::Error> for TripPlannerError {
    fn from(error: std::io::Error) -> Self {
        TripPlannerError::Io(error)
    }
}

impl From<serde_yaml::Error> for TripPlannerError {
    fn from(error: serde_yaml::Error) -> Self {
        TripPlannerError::Yaml(error)
    }
}

impl From<serde_json::Error> for TripPlannerError {
    fn from(error: serde_json::Error) -> Self {
        TripPlannerError::Json(error)
    }
}

impl From<reqwest::Error> for TripPlannerError {
    fn from(error: reqwest::Error) -> Self {
        TripPlannerError::Network(error)
    }
}

// Configuration constants
const NOMINATIM_BASE_URL: &str = "https://nominatim.openstreetmap.org/search";
const OSRM_BASE_URL: &str = "https://router.project-osrm.org/route/v1/driving";
const USER_AGENT: &str = "trip-planner-rust";
const GEOCODE_CACHE_FILE: &str = "geocode_cache.json";
const ROUTE_CACHE_FILE: &str = "route_cache.json";
const OUTPUT_FILE: &str = "combined_map.html";
const METERS_TO_MILES: f64 = 1609.34;
const EARTH_RADIUS_MILES: f64 = 3958.8;
const API_DELAY_MS: u64 = 100; // Rate limiting delay
const CACHE_EXPIRY_DAYS: u64 = 30; // Routes expire after 30 days
const GEODESIC_SEGMENTS: usize = 100; // Number of segments for geodesic curves

#[derive(Debug, Deserialize, Clone)]
struct TripFile {
    trips: Vec<Trip>,
}

#[derive(Debug, Deserialize, Clone)]
struct Trip {
    name: String,
    date: Option<String>,
    segments: Vec<TripSegment>,
}

#[derive(Debug, Deserialize, Clone)]
struct TripSegment {
    method: Option<String>,
    waypoints: Vec<String>,
}

impl TripSegment {
    fn get_method(&self) -> &str {
        self.method.as_deref().unwrap_or("car")
    }
}

#[derive(Debug, Deserialize)]
struct NominatimResult {
    lat: String,
    lon: String,
}

#[derive(Debug, Deserialize, Clone)]
struct RouteResponse {
    routes: Vec<Route>,
}

#[derive(Debug, Deserialize, Clone)]
struct Route {
    geometry: serde_json::Value,
    distance: f64,
}

#[derive(Debug, Deserialize)]
struct OSRMErrorResponse {
    code: Option<String>,
    message: Option<String>,
}

type Coordinates = (f64, f64);
type GeocodeCache = HashMap<String, Coordinates>;

#[derive(Debug, Serialize, Deserialize, Clone)]
struct CachedRoute {
    geojson: String,
    distance_miles: f64,
    cached_at: u64, // Unix timestamp
}

type RouteCache = HashMap<String, CachedRoute>;

struct TripProcessor {
    client: Client,
    geocode_cache: GeocodeCache,
    route_cache: RouteCache,
}

impl TripProcessor {
    fn new() -> Self {
        let geocode_cache = Self::load_geocode_cache();
        let mut route_cache = Self::load_route_cache();
        
        // Clean up expired routes
        let initial_route_count = route_cache.len();
        Self::cleanup_expired_routes(&mut route_cache);
        let cleaned_route_count = route_cache.len();
        
        if initial_route_count != cleaned_route_count {
            println!("Cleaned up {} expired route entries", initial_route_count - cleaned_route_count);
        }
        
        println!("Geocode cache loaded with {} entries", geocode_cache.len());
        println!("Route cache loaded with {} entries", route_cache.len());
        
        Self {
            client: Client::new(),
            geocode_cache,
            route_cache,
        }
    }

    fn load_geocode_cache() -> GeocodeCache {
        match fs::read_to_string(GEOCODE_CACHE_FILE) {
            Ok(data) => serde_json::from_str(&data).unwrap_or_default(),
            Err(_) => HashMap::new(),
        }
    }

    fn load_route_cache() -> RouteCache {
        match fs::read_to_string(ROUTE_CACHE_FILE) {
            Ok(data) => serde_json::from_str(&data).unwrap_or_default(),
            Err(_) => HashMap::new(),
        }
    }

    fn save_geocode_cache(&self) -> Result<(), TripPlannerError> {
        let data = serde_json::to_string_pretty(&self.geocode_cache)?;
        fs::write(GEOCODE_CACHE_FILE, data)?;
        println!("Saved geocode cache with {} entries", self.geocode_cache.len());
        Ok(())
    }

    fn save_route_cache(&self) -> Result<(), TripPlannerError> {
        let data = serde_json::to_string_pretty(&self.route_cache)?;
        fs::write(ROUTE_CACHE_FILE, data)?;
        println!("Saved route cache with {} entries", self.route_cache.len());
        Ok(())
    }

    fn save_all_caches(&self) -> Result<(), TripPlannerError> {
        self.save_geocode_cache()?;
        self.save_route_cache()?;
        Ok(())
    }

    /// Generate a cache key for a route based on coordinates
    /// Rounds coordinates to 4 decimal places (~11m precision) to allow for small variations
    fn generate_route_cache_key(&self, coords: &[Coordinates]) -> String {
        let rounded_coords: Vec<String> = coords
            .iter()
            .map(|(lon, lat)| format!("{:.4},{:.4}", lon, lat))
            .collect();
        format!("route:{}", rounded_coords.join(";"))
    }

    fn get_current_timestamp() -> u64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs()
    }

    fn cleanup_expired_routes(route_cache: &mut RouteCache) {
        let current_time = Self::get_current_timestamp();
        let expiry_threshold = current_time - (CACHE_EXPIRY_DAYS * 24 * 60 * 60);
        
        route_cache.retain(|_, cached_route| {
            cached_route.cached_at > expiry_threshold
        });
    }

    async fn geocode(&mut self, query: &str) -> Result<Coordinates, TripPlannerError> {
        // Check cache first
        if let Some(&coords) = self.geocode_cache.get(query) {
            // println!("Using cached coordinates for '{}': {:?}", query, coords);
            return Ok(coords);
        }

        // Rate limiting
        sleep(Duration::from_millis(API_DELAY_MS)).await;

        let url = format!(
            "{}?q={}&format=json&limit=1",
            NOMINATIM_BASE_URL,
            urlencoding::encode(query)
        );

        let resp = self
            .client
            .get(&url)
            .header("User-Agent", USER_AGENT)
            .send()
            .await?
            .json::<Vec<NominatimResult>>()
            .await?;

        if let Some(result) = resp.first() {
            let coords = (
                result.lon.parse().map_err(|_| {
                    TripPlannerError::Geocoding(format!("Invalid longitude for query: '{}'", query))
                })?,
                result.lat.parse().map_err(|_| {
                    TripPlannerError::Geocoding(format!("Invalid latitude for query: '{}'", query))
                })?,
            );

            self.geocode_cache.insert(query.to_string(), coords);
            // println!("Geocoded '{}': {:?}", query, coords);
            Ok(coords)
        } else {
            Err(TripPlannerError::Geocoding(format!(
                "Location not found for query: '{}'",
                query
            )))
        }
    }

    async fn geocode_waypoints(&mut self, waypoints: &[String]) -> Result<Vec<Coordinates>, TripPlannerError> {
        let mut coords = Vec::with_capacity(waypoints.len());
        for waypoint in waypoints {
            coords.push(self.geocode(waypoint).await?);
        }
        Ok(coords)
    }

    async fn get_route_data(
        &mut self,
        coords: &[Coordinates],
    ) -> Result<(String, f64), TripPlannerError> {
        // Check cache first
        let cache_key = self.generate_route_cache_key(coords);
        if let Some(cached_route) = self.route_cache.get(&cache_key) {
            // println!("Using cached route for {} coordinates", coords.len());
            return Ok((cached_route.geojson.clone(), cached_route.distance_miles));
        }

        let coord_strings: Vec<String> = coords
            .iter()
            .map(|(lon, lat)| format!("{},{}", lon, lat))
            .collect();

        let url = format!(
            "{}/{}?overview=full&geometries=geojson",
            OSRM_BASE_URL,
            coord_strings.join(";")
        );

        // println!("Requesting route from OSRM: {}", url);

        // Rate limiting for API calls
        sleep(Duration::from_millis(API_DELAY_MS)).await;

        let response = self
            .client
            .get(&url)
            .header("User-Agent", USER_AGENT)
            .send()
            .await?;

        let status = response.status();
        let response_text = response.text().await?;
        
        if !status.is_success() {
            return Err(TripPlannerError::Geocoding(format!(
                "OSRM API returned status {}: {}",
                status, response_text
            )));
        }

        // println!("OSRM Response: {}", response_text);

        // Try to parse as RouteResponse first
        match serde_json::from_str::<RouteResponse>(&response_text) {
            Ok(resp) => {
                let route = resp
                    .routes
                    .first()
                    .ok_or_else(|| TripPlannerError::Geocoding("No routes in response".to_string()))?;

                let geojson = serde_json::to_string(&route.geometry)?;
                let distance_miles = route.distance / METERS_TO_MILES;

                // Cache the successful route
                let cached_route = CachedRoute {
                    geojson: geojson.clone(),
                    distance_miles,
                    cached_at: Self::get_current_timestamp(),
                };
                self.route_cache.insert(cache_key, cached_route);

                Ok((geojson, distance_miles))
            }
            Err(e) => {
                // Try to parse as error response
                if let Ok(error_resp) = serde_json::from_str::<OSRMErrorResponse>(&response_text) {
                    Err(TripPlannerError::Geocoding(format!(
                        "OSRM API error: {} - {}",
                        error_resp.code.unwrap_or_else(|| "Unknown".to_string()),
                        error_resp.message.unwrap_or_else(|| "No message".to_string())
                    )))
                } else {
                    Err(TripPlannerError::Json(e))
                }
            }
        }
    }

    fn calculate_haversine_distance(&self, coords: &[Coordinates]) -> f64 {
        coords
            .windows(2)
            .map(|window| haversine_distance(window[0], window[1]))
            .sum()
    }

    async fn process_trip(&mut self, trip: &Trip) -> Result<Vec<TripResult>, TripPlannerError> {
        let mut results = Vec::new();

        for segment in &trip.segments {
            let coords = self.geocode_waypoints(&segment.waypoints).await?;
            let method = segment.get_method();

            let (geojson, distance) = match method {
                "car" => self.get_route_data(&coords).await
                    .unwrap_or_else(|_| (create_linestring_geojson(&coords), self.calculate_haversine_distance(&coords))),
                "plane" => {
                    let dist = self.calculate_haversine_distance(&coords);
                    (create_geodesic_geojson(&coords), dist)
                }
                _ => {
                    let dist = self.calculate_haversine_distance(&coords);
                    (create_linestring_geojson(&coords), dist)
                }
            };

            results.push(TripResult {
                trip_name: trip.name.clone(),
                segment_method: method.to_string(),
                coords,
                geojson,
                distance,
            });
        }

        Ok(results)
    }

}

struct TripResult {
    trip_name: String,
    segment_method: String,
    coords: Vec<Coordinates>,
    geojson: String,
    distance: f64,
}

struct MapGenerator;

impl MapGenerator {
    fn generate_html(trip_results: &[TripResult]) -> Result<String, TripPlannerError> {
        const TEMPLATE: &str = include_str!("map_template.html");
        
        let (car_features, plane_features) = Self::group_features_by_method(trip_results);
        let markers_js = Self::generate_markers_js(trip_results);

        // Replace placeholders in template
        let html = TEMPLATE
            .replace("{{CAR_FEATURES}}", &car_features.join(",\n      "))
            .replace("{{PLANE_FEATURES}}", &plane_features.join(",\n      "))
            .replace("{{MARKERS}}", &markers_js);

        Ok(html)
    }

    fn group_features_by_method(trip_results: &[TripResult]) -> (Vec<String>, Vec<String>) {
        let mut car_features = Vec::new();
        let mut plane_features = Vec::new();

        for (i, result) in trip_results.iter().enumerate() {
            let feature = format!(
                r#""feature-{}": {{
          "type": "Feature",
          "geometry": {},
          "properties": {{
            "name": "{}",
            "method": "{}",
            "distance": {}
          }}
        }}"#,
                i,
                result.geojson,
                result.trip_name.replace('"', "\\\""),
                result.segment_method.clone(),
                result.distance
            );

            match result.segment_method.as_str() {
                "plane" => plane_features.push(feature),
                _ => car_features.push(feature),
            }
        }

        (car_features, plane_features)
    }

    fn generate_markers_js(trip_results: &[TripResult]) -> String {
        let mut markers_js = String::new();

        for result in trip_results {
            if result.coords.is_empty() {
                continue;
            }

            for coord in &result.coords {
                markers_js.push_str(&format!(
                    "    L.circleMarker([{:.6}, {:.6}], {{radius: 3, color: 'red', fillColor: 'red', fillOpacity: 0.8}}).addTo(map);\n",
                    coord.1, coord.0
                ));
            }

            let trip_name = result.trip_name.replace('\'', "\\'");
            let start = result.coords[0];
            let end = result.coords[result.coords.len() - 1];

            // Start and end markers
            markers_js.push_str(&format!(
                "    L.marker([{}, {}]).addTo(map).bindPopup('Start: {}');\n",
                start.1, start.0, trip_name
            ));
            markers_js.push_str(&format!(
                "    L.marker([{}, {}]).addTo(map).bindPopup('End: {}');\n",
                end.1, end.0, trip_name
            ));

            // Intermediate waypoints
            // for (i, &(lon, lat)) in result.coords.iter().enumerate().skip(1).take(result.coords.len() - 2) {
            //     // let waypoint_label = result.trip.waypoints.get(i).map_or("Waypoint".to_string(), |w| w.replace('\'', "\\'"));
            //     markers_js.push_str(&format!(
            //         "    L.circleMarker([{}, {}], {{radius: 3, color: 'red', fillColor: 'blue', fillOpacity: 0.8}}).addTo(map).bindPopup('Waypoint: {}');\n",
            //         lat, lon, waypoint_label
            //     ));
            // }
        }

        markers_js
    }

    fn write_to_file(html: &str) -> Result<(), TripPlannerError> {
        let mut file = File::create(OUTPUT_FILE)?;
        file.write_all(html.as_bytes())?;
        println!("Map written to {}", OUTPUT_FILE);
        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<(), TripPlannerError> {
    let trip_file = parse_yaml("trips.yaml")?;
    let mut processor = TripProcessor::new();

    let mut trip_results = Vec::new();
    let mut totals: HashMap<String, f64> = HashMap::new();

    // Process all trips
    for trip in &trip_file.trips {
        let results = processor.process_trip(trip).await?;
        for result in results {
            *totals.entry(result.segment_method.clone()).or_insert(0.0) += result.distance;
            trip_results.push(result);
        }
    }

    // Save updated caches
    processor.save_all_caches()?;

    // Generate and write map
    let html = MapGenerator::generate_html(&trip_results)?;
    MapGenerator::write_to_file(&html)?;

    // Print totals
    println!("\n=== Distance Totals ===");
    for (method, miles) in &totals {
        println!("{}: {:.1} miles", method.to_uppercase(), miles);
    }

    Ok(())
}

fn parse_yaml<P: AsRef<Path>>(path: P) -> Result<TripFile, TripPlannerError> {
    let content = fs::read_to_string(path)?;
    let parsed: TripFile = serde_yaml::from_str(&content)?;
    Ok(parsed)
}

fn haversine_distance(coord1: Coordinates, coord2: Coordinates) -> f64 {
    let (lon1, lat1) = (coord1.0.to_radians(), coord1.1.to_radians());
    let (lon2, lat2) = (coord2.0.to_radians(), coord2.1.to_radians());

    let dlon = lon2 - lon1;
    let dlat = lat2 - lat1;

    let a = dlat.sin().powi(2) + lat1.cos() * lat2.cos() * dlon.sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    EARTH_RADIUS_MILES * c
}

fn create_linestring_geojson(coords: &[Coordinates]) -> String {
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
fn create_geodesic_geojson(coords: &[Coordinates]) -> String {
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
fn interpolate_geodesic(start: Coordinates, end: Coordinates, segments: usize) -> Vec<Coordinates> {
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