get_info();
update_datetime();

setInterval(update_datetime, 1000);

function get_info() {
    fetch('/api/info')
        .then(response => response.json())
        .then(data => {
            console.log(data);
            var dev_name = document.getElementById("device_name");
            dev_name.innerText = data.name;
            var dev_location = document.getElementById("device_location");
            dev_location.innerText = data.location;
            var dev_ssid = document.getElementById("device_ssid");
            dev_ssid.innerText = data.ssid;
            var dev_ip = document.getElementById("device_ip");
            dev_ip.innerText = int2ip(data.ip);
            var dev_mac = document.getElementById("device_mac");
            var mac_str = '';
            data.mac.forEach(el => {
                mac_str += el.toString(16)
                mac_str += ":"
            });
            dev_mac.innerText = mac_str.toUpperCase().slice(0, -1);
            var dev_asc = document.getElementById("device_asc");
            dev_asc.innerText = data.asc ? "Enabled" : "Disabled";

            var button_asc = document.getElementById("button_asc");
            if (button_asc == null) {
                return
            } else {
                if (data.asc) {
                    button_asc.value = "disable"
                    button_asc.innerText = "Disable"
                    button_asc.classList.remove("is-link");
                    button_asc.classList.add("is-warning");
                } else {
                    button_asc.value = "enable"
                    button_asc.innerText = "Enable"
                    button_asc.classList.remove("is-warning");
                    button_asc.classList.add("is-link");
                }
            }

        });
}

function int2ip(ipInt) {
    return ((ipInt >>> 24) + '.' + (ipInt >> 16 & 255) + '.' + (ipInt >> 8 & 255) + '.' + (ipInt & 255));
}

function update_datetime() {
    var dt = new Date();
    document.getElementById("datetime").innerHTML = dt.toLocaleString();
}
document.getElementById("navbar_burger").addEventListener("click", function () {
    this.classList.toggle("is-active")
    document.getElementById("navbar_menu").classList.toggle("is-active")
})