require('dotenv').config();
const express = require('express');
const router = express.Router();
const controller = require('./controller');

router.get('/device/status', controller.getDeviceStatus);
router.get('/device/shadow/data', controller.getDeviceStatusShadow);

module.exports = router;
