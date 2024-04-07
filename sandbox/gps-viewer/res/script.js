
var map
var markers = []

function addMarker(lat, lng) {
    let marker = L.marker([lat, lng])

    marker.addTo(map)

    markers.push(marker)

    if(markers.length>100) {
        let oldestMarker = markers.shift()
        map.removeLayer(oldestMarker)
    }
}

function setView(lat, lng, zoom) {
    map.setView([lat, lng], zoom)
}

window.addEventListener('DOMContentLoaded', function() {
    map = L.map('map')

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors',
        maxZoom: 19,
    }).addTo(map)
})
