const express = require('express');
const routes = require('./routes');
const cors = require('cors');
require('dotenv').config();

const app = express();

app.use((req, res, next) => {
  res.header('Access-Control-Allow-Origin', 'http://127.0.0.1:5500');
  res.header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept, Authorization');
  res.header('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE');
  res.header('Access-Control-Allow-Credentials', 'true');
  next();
});

app.use(express.json());
app.use('/api', routes);

const port = process.env.PORT || 3000; 

app.listen(port, () => {
  console.log(`Server is running on port ${port}`);
});