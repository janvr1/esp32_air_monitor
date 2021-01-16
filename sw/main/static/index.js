get_measurements();

setInterval(get_measurements, 5000);

function get_measurements() {
    fetch('/api/measurements')
        .then(response => response.json())
        .then(data => {
            console.log(data);
            temp = document.getElementById("temperature");
            temp.innerText = data.T;
            temp.innerText += " °C";
            hum = document.getElementById("humidity");
            hum.innerText = Math.round(data.RH);
            hum.innerText += " %RH";
            co2 = document.getElementById("co2");
            co2.innerText = data.CO2;
            co2.innerText += " ppm";
            pres = document.getElementById("pressure");
            pres.innerText = Math.round(data.p / 100);
            pres.innerText += " mbar";
            illu = document.getElementById("illuminance");
            illu.innerText = data.I;
            illu.innerText += " lux";
        }).catch((error) => {
            console.error('Error:', error);
        });
}

