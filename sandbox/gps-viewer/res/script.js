
var map
var polylines = []

function setView(lat, lng, zoom) {
    map.setView([lat, lng], zoom)
}

function createPolyLine(color) {
    let polyline = L.polyline([], {
        color: color
    }).addTo(map);

    polylines.push(polyline)
}

function appendPolyLine(lat, lng, index) {
    polylines[index].addLatLng([lat, lng])
}

function addPoint(lat, lng, color) {
    let point = L.circleMarker([lat, lng], {
        color: color,
        radius: 1
    }).addTo(map)
}

window.addEventListener('DOMContentLoaded', function() {
    map = L.map('map')

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors',
        maxZoom: 19,
    }).addTo(map)
})
