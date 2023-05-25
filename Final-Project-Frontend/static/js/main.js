const backendIPAddress = '127.0.0.1:3000';

const fetchDeviceStatusShadow = async () => {
  const updateUIWithData = (dust, temp, humidity, lux) => {
    dust = Math.round(dust);
    document.getElementById('dust-value').innerHTML = dust + ' AQI';
    document.getElementById('temp-value').innerHTML = temp + ' °C';
    document.getElementById('humidity-value').innerHTML = humidity + '%';
    document.getElementById('lux-value').innerHTML = lux + ' lux';

    const statusImage = document.getElementById('status-image');
    const dustDescription = document.getElementById('dust-description');
    const tempImage = document.getElementById("temp-image");
    if (dust >= 301) {
      statusImage.src = 'static/images/status/dust/icon-maroon.png';
      dustDescription.innerHTML = 'Hazardous';
    } else if (dust >= 201) {
      statusImage.src = 'static/images/status/dust/icon-purple.png';
      dustDescription.innerHTML = 'Very Unhealthy';
    } else if (dust >= 151) {
      statusImage.src = 'static/images/status/dust/icon-red.png';
      dustDescription.innerHTML = 'Unhealthy';
    } else if (dust >= 101) {
      statusImage.src = 'static/images/status/dust/icon-orange.png';
      dustDescription.innerHTML = 'Unhealthy for Sensitive Groups';
    } else if (dust >= 51) {
      statusImage.src = 'static/images/status/dust/icon-yellow.png';
      dustDescription.innerHTML = 'Moderate';
    } else {
      statusImage.src = 'static/images/status/dust/icon-green.png';
      dustDescription.innerHTML = 'Good';
    }
    if (temp >= 40) {
      tempImage.src = 'static/images/status/temp/icon-hot.png'
    }
    else if (temp >= 25) {
      tempImage.src = 'static/images/status/temp/icon-warm.png'
    }
    else {
      tempImage.src = 'static/images/status/temp/icon-cool.png'
    }
  };

  try {
    const options = {
      method: 'GET',
      //headers: {
        //'Authorization': 'Device 4513aa93-b88c-4e9b-92d3-11ba2920a2ee:Z7XT8mdToAGAYFU8898Xym7sgyBiC64K',
      //},
      credentials: 'include',
    };
    const response = await fetch(`http://${backendIPAddress}/api/device/shadow/data`, options);
    const data = await response.json();
    console.log(data);
    const dataTable = data.data;
    updateUIWithData(dataTable.dust, dataTable.temp, dataTable.humidity, dataTable.lux);
  } catch (error) {
    console.error('Error:', error);
  }
};

fetchDeviceStatusShadow();

setInterval(fetchDeviceStatusShadow, 500);