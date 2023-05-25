const axios = require('axios');
const { Buffer } = require('buffer');

exports.getDeviceStatus = async (req, res) => {
  const apiUrl = 'https://api.netpie.io/v2/device/status';
  const clientID = process.env.CLIENT_ID;
  const token = process.env.TOKEN;
  try {
    const authHeader = `Basic ${Buffer.from(`${clientID}:${token}`).toString('base64')}`;

    const response = await axios.get(apiUrl, {
      headers: {
        Authorization: authHeader,
      },
    });
    
    res.json(response.data);
  } catch (error) {
    console.error('Error:', error);
    res.status(500).json({ error: 'fetch error krub, so sad' });
  }
};

exports.getDeviceStatusShadow = async (req, res) => {
  const apiUrl = 'https://api.netpie.io/v2/device/shadow/data';
  const clientID = process.env.CLIENT_ID;
  const token = process.env.TOKEN;
  try {
    const authHeader = `Basic ${Buffer.from(`${clientID}:${token}`).toString('base64')}`;

    const response = await axios.get(apiUrl, {
      headers: {
        Authorization: authHeader,
      },
    });
    
    res.json(response.data);
  } catch (error) {
    console.error('Error:', error);
    res.status(500).json({ error: 'fetch error krub, so sad' });
  }
};