// main.rs
use open;
use reqwest::Client;
use serde::Deserialize;
use std::collections::HashMap;
use std::error::Error;
use std::fs::File;
use std::io::Write;
use std::{fs, path::Path};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let trips: TripFile = parse_yaml("trips.yaml")?;
    let client = Client::new();

    let mut all_geojsons = Vec::new();
    let mut all_coords = Vec::new();
    let mut totals: HashMap<String, f64> = HashMap::new();

    // Load geocode cache
    let mut geocode_cache = load_geocode_cache();

    for trip in &trips.trips {
        println!("Processing trip: {}", trip.name);
        let coords = geocode_all_cached(&client, &trip.waypoints, &mut geocode_cache).await?;
        let method = trip.method.clone().unwrap_or_else(|| "car".to_string());
        let distance = match method.as_str() {
            "car" => get_route_distance(&client, &coords).await?,
            _ => coords.windows(2).map(|w| haversine(w[0], w[1])).sum(),
        };
        *totals.entry(method.clone()).or_insert(0.0) += distance;

        let route_geojson = match trip.method.as_deref() {
            Some("car") | None => get_route(&client, &coords).await?,
            _ => make_linestring_geojson(&coords),
        };
        all_geojsons.push(route_geojson);
        all_coords.push(coords);
    }

    // Save geocode cache
    save_geocode_cache(&geocode_cache);

    // Write combined map
    write_combined_html_map(&trips.trips, &all_geojsons, &all_coords)?;

    // Print totals
    println!("");
    for (method, miles) in &totals {
        println!("Total {} miles: {:.1}", method, miles);
    }

    Ok(())
}

fn parse_yaml<P: AsRef<Path>>(path: P) -> Result<TripFile, Box<dyn Error>> {
    let content = fs::read_to_string(path)?;
    let parsed: TripFile = serde_yaml::from_str(&content)?;
    Ok(parsed)
}

fn sanitize(name: &str) -> String {
    name.replace(" ", "_").replace("/", "-")
}

async fn geocode(client: &Client, query: &str) -> Result<(f64, f64), Box<dyn Error>> {
    let url = format!(
        "https://nominatim.openstreetmap.org/search?q={}&format=json&limit=1",
        urlencoding::encode(query)
    );
    let resp = client
        .get(&url)
        .header("User-Agent", "trip-planner-rust")
        .send()
        .await?
        .json::<Vec<NominatimResult>>()
        .await?;
    if let Some(first) = resp.get(0) {
        Ok((first.lon.parse()?, first.lat.parse()?))
    } else {
        Err(format!("Location not found for query: '{}', url: {}", query, url).into())
    }
}

async fn get_route(client: &Client, coords: &[(f64, f64)]) -> Result<String, Box<dyn Error>> {
    let coord_strings: Vec<String> = coords
        .iter()
        .map(|(lon, lat)| format!("{},{}", lon, lat))
        .collect();
    let url = format!(
        "https://router.project-osrm.org/route/v1/driving/{}?overview=full&geometries=geojson",
        coord_strings.join(";")
    );

    let resp = client
        .get(&url)
        .header("User-Agent", "trip-planner-rust")
        .send()
        .await?
        .json::<osrm::RouteResponse>()
        .await?;

    let geojson = serde_json::to_string(&resp.routes[0].geometry)?;
    Ok(geojson)
}

fn make_linestring_geojson(coords: &[(f64, f64)]) -> String {
    let coord_pairs: Vec<String> = coords
        .iter()
        .map(|(lon, lat)| format!("[{},{}]", lon, lat))
        .collect();
    format!(
        r#"{{ "type": "LineString", "coordinates": [{}] }}"#,
        coord_pairs.join(",")
    )
}

fn write_combined_html_map(
    trips: &[Trip],
    all_geojsons: &[String],
    all_coords: &[Vec<(f64, f64)>],
) -> Result<(), Box<dyn Error>> {
    let mut file = File::create("combined_map.html")?;
    let mut geojson_features = Vec::new();

    for (i, geojson) in all_geojsons.iter().enumerate() {
        // Wrap geometry in a Feature for Leaflet
        geojson_features.push(format!(
            r#""feature-{}": {{ "type": "Feature", "geometry": {}, "properties": {{ "name": "{}" }} }}"#,
            i, geojson, trips[i].name.replace('"', "\\\"")
        ));
    }

    let mut markers_js = String::new();
    for (i, coords) in all_coords.iter().enumerate() {
        if !coords.is_empty() {
            let start = coords.first().unwrap();
            let end = coords.last().unwrap();
            let trip_name = trips[i].name.replace('\'', "\\'");
            // Start marker (default red)
            markers_js.push_str(&format!(
                "L.marker([{lat}, {lon}]).addTo(map).bindPopup('Start: {trip_name}');\n",
                lat = start.1,
                lon = start.0,
                trip_name = trip_name
            ));
            // End marker (default red)
            markers_js.push_str(&format!(
                "L.marker([{lat}, {lon}]).addTo(map).bindPopup('End: {trip_name}');\n",
                lat = end.1,
                lon = end.0,
                trip_name = trip_name
            ));
            // Intermediate waypoints (blue)
            if coords.len() > 2 {
                for (j, (lon, lat)) in coords.iter().enumerate().skip(1).take(coords.len() - 2) {
                    let waypoint_label = trips[i].waypoints[j].replace('\'', "\\'");
                    markers_js.push_str(&format!(
                        "L.circleMarker([{lat}, {lon}], {{radius: 3, color: 'red', fillColor: 'blue', fillOpacity: 0.8}}).addTo(map).bindPopup('Waypoint: {waypoint_label}');\n",
                        lat = lat,
                        lon = lon,
                        waypoint_label = waypoint_label
                    ));
                }
            }
        }
    }

    let html = format!(
        r#"<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Combined Trip Map</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
  <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
</head>
<body>
  <div id="map" style="width: 100%; height: 100vh;"></div>
  <script>
    var map = L.map('map').setView([0, 0], 2);
    L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
      attribution: '&copy; OpenStreetMap contributors'
    }}).addTo(map);
    var bounds = L.latLngBounds();
    var geojsons = {{
      {geojsons}
    }};
    for (var key in geojsons) {{
      var layer = L.geoJSON(geojsons[key]).addTo(map);
      layer.bindPopup(geojsons[key].properties.name);
      layer.eachLayer(function(l) {{
        if (l.getBounds) {{
          bounds.extend(l.getBounds());
        }}
      }});
    }}
    {markers_js}
    if (bounds.isValid()) {{
      map.fitBounds(bounds);
    }}
  </script>
</body>
</html>"#,
        geojsons = geojson_features.join(",\n"),
        markers_js = markers_js
    );
    file.write_all(html.as_bytes())?;
    Ok(())
}

fn load_geocode_cache() -> HashMap<String, (f64, f64)> {
    let path = "geocode_cache.json";
    if let Ok(data) = std::fs::read_to_string(path) {
        serde_json::from_str(&data).unwrap_or_default()
    } else {
        HashMap::new()
    }
}

fn save_geocode_cache(cache: &HashMap<String, (f64, f64)>) {
    let data = serde_json::to_string_pretty(cache).unwrap();
    std::fs::write("geocode_cache.json", data).unwrap();
}

async fn geocode_all_cached(
    client: &Client,
    waypoints: &[String],
    cache: &mut HashMap<String, (f64, f64)>,
) -> Result<Vec<(f64, f64)>, Box<dyn Error>> {
    let mut coords = vec![];
    for loc in waypoints {
        if let Some(&coord) = cache.get(loc) {
            coords.push(coord);
        } else {
            let coord = geocode(client, loc).await?;
            cache.insert(loc.clone(), coord);
            coords.push(coord);
        }
    }
    Ok(coords)
}

#[derive(Debug, Deserialize)]
struct TripFile {
    trips: Vec<Trip>,
}

#[derive(Debug, Deserialize)]
struct Trip {
    name: String,
    date: Option<String>,
    method: Option<String>,
    waypoints: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct NominatimResult {
    lat: String,
    lon: String,
}

mod osrm {
    use serde::Deserialize;

    #[derive(Debug, Deserialize, Clone)]
    pub struct RouteResponse {
        pub routes: Vec<Route>,
    }

    #[derive(Debug, Deserialize, Clone)]
    pub struct Route {
        pub geometry: serde_json::Value,
        pub distance: f64,
    }
}

fn haversine(coord1: (f64, f64), coord2: (f64, f64)) -> f64 {
    let (lon1, lat1) = (coord1.0.to_radians(), coord1.1.to_radians());
    let (lon2, lat2) = (coord2.0.to_radians(), coord2.1.to_radians());

    let dlon = lon2 - lon1;
    let dlat = lat2 - lat1;

    let a = dlat.sin().powi(2) + lat1.cos() * lat2.cos() * dlon.sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    3958.8 * c // Returns distance in miles
}

async fn get_route_distance(client: &Client, coords: &[(f64, f64)]) -> Result<f64, Box<dyn Error>> {
    let coord_strings: Vec<String> = coords
        .iter()
        .map(|(lon, lat)| format!("{},{}", lon, lat))
        .collect();
    let url = format!(
        "https://router.project-osrm.org/route/v1/driving/{}?overview=full",
        coord_strings.join(";")
    );

    let resp = client
        .get(&url)
        .header("User-Agent", "trip-planner-rust")
        .send()
        .await?
        .json::<osrm::RouteResponse>()
        .await?;

    Ok(resp.routes[0].distance / 1609.34) // meters to miles
}
