const form_wifi = document.getElementById("form_wifi");
form_wifi.addEventListener("submit", postWifiInfo);

const form_frc = document.getElementById("form_frc");
form_frc.addEventListener("submit", postScdFrc);

const form_zrak = document.getElementById("form_zrak");
form_zrak.addEventListener("submit", postZrak);

function postZrak(e) {
    e.preventDefault();
    var data = {};
    data["user"] = document.getElementById("input_zrak_user").value;
    data["pass"] = document.getElementById("input_zrak_pass").value;
    data["dev_id"] = parseInt(document.getElementById("input_zrak_dev_id").value);
    console.log(data);
    fetch('/api/zrak', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(data),
    })
        .then(response => {
            if (response.status == 200) {
                console.log('Success:', response);
            } else {
                console.log('Error:', response);
            }
            response.text().then(text => {
                console.log("Response text:", text);
                alert(text);
            })
        })
        .catch((error) => {
            console.error('Error:', error);
        });
}

function postDevInfo() {
    var data = {};
    var new_name = document.getElementById("input_dev_name").value;
    if (new_name != "") {
        data["dev_name"] = new_name;
    }
    var new_loc = document.getElementById("input_dev_location").value;
    if (new_loc != "") {
        data["dev_location"] = new_loc;
    }
    console.log(data);
    fetch('/api/dev_info', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(data),
    })
        .then(response => {
            if (response.status == 200) {
                console.log('Success:', response);
            } else {
                console.log('Error:', response);
            }
            response.text().then(text => {
                console.log("Response text:", text);
                alert(text);
                if (response.status == 200) {
                    location.reload();
                }
            })
        })
        .catch((error) => {
            console.error('Error:', error);
        });
}

function postWifiInfo(e) {
    e.preventDefault();
    var data = {};
    data["ssid"] = document.getElementById("input_ssid").value;
    data["pass"] = document.getElementById("input_pass").value;
    console.log(data);
    fetch('/api/wifi', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(data),
    })
        .then(response => {
            if (response.status == 200) {
                console.log('Success:', response);
            } else {
                console.log('Error:', response);
            }
            response.text().then(text => {
                console.log("Response text:", text);
                alert(text);
            })
        })
        .catch((error) => {
            console.error('Error:', error);
        });
}

function postScdFrc(e) {
    e.preventDefault();
    var val = document.getElementById("input_frc").value;
    var msg = "Are you sure?\nThis will recalibrate the sensor! Make sure the current CO2 concentration really is " + val + " ppm.";
    if (!confirm(msg)) {
        return;
    }
    fetch('/api/scd_frc?ref=' + val, {
        method: 'POST',
    })
        .then(response => {
            if (response.status == 200) {
                console.log('Success:', response);
            } else {
                console.log('Error:', response);
            }
            response.text().then(text => {
                console.log("Response text:", text);
                alert(text);
            })
        })
        .catch((error) => {
            console.error('Error:', error);
        });
}

function postScdAsc() {
    var val;
    if (document.getElementById("button_asc").value == "enable") {
        val = 1;
    } else {
        val = 0;
    };

    fetch('/api/scd_asc?enable=' + val, {
        method: 'POST',
    })
        .then(response => {
            if (response.status == 200) {
                console.log('Success:', response);
            } else {
                console.log('Error:', response);
            }
            response.text().then(text => {
                console.log("Response text:", text);
                alert(text);
                if (response.status == 200) {
                    location.reload();
                }
            })
        })
        .catch((error) => {
            console.error('Error:', error);
        });
}